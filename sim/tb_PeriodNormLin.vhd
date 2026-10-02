--------------------------------------------------------------------------------
-- Self-checking testbench for PeriodNormLin (VHDL-2008)
--
--   ghdl -a --std=08 src/PeriodNormLin_pkg.vhd src/PeriodNormLin.vhd sim/tb_PeriodNormLin.vhd
--   ghdl -r --std=08 tb_PeriodNormLin
--
-- The DUT is compared with a floating-point evaluation of
--   x = Period * Gain / 2**36 + Offset / 2**24
--   y = sum(k = 1..6) Tk(x) * Coeff(k)
-- The allowed error is the analytical worst case derived in PeriodNormLin.vhd:
--   0.5 LSB (final rounding)
--   + 6 * 2**-9 LSB (rounding of the six T*C products)
--   + sum |C(k)| * (k**2 + k*(k-1)/2) * 2**-33   (rounding of x and of the
--                                                  Chebyshev recurrence)
-- Also checked: latency, pulse width of ValidxSO, level behaviour of the
-- handshake, and the corner cases x = +1 and x = -1 exactly.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library work;
use work.PeriodNormLin_pkg.all;

entity tb_PeriodNormLin is
end entity tb_PeriodNormLin;

architecture sim of tb_PeriodNormLin is

  constant CLK_PERIOD : time    := 10 ns;
  constant LATENCY    : integer := 61;  -- clock edges from the sampling edge to the output update

  signal ClkxC       : std_logic := '0';
  signal PeriodEcs   : unsigned(23 downto 0) := (others => '0');
  signal PeriodValid : std_logic := '0';
  signal GainNorm    : signed(31 downto 0) := (others => '0');
  signal OffsetNorm  : signed(31 downto 0) := (others => '0');
  signal Coeff       : ChebyshevCoeff_t(1 to 6) := (others => (others => '0'));
  signal Valid       : std_logic;
  signal PeriodOut   : signed(31 downto 0);
  signal SimDone     : boolean := false;

  type CoefInt_t is array (1 to 6) of integer;

  -- real -> Q8.24 raw
  function Raw(v : real) return integer is
  begin
    return integer(round(v * 2.0 ** 24));
  end function Raw;

  -- Gain/offset that map 1.0e6 .. 2.0e6 to about -1 .. +1
  constant GAIN_TYP   : integer := 137438;                                      -- ~2e-6 in Q-4.36
  constant OFFSET_TYP : integer := integer(round(-1.5e6 * real(GAIN_TYP) / 4096.0)); -- Q8.24
  -- Gain/offset for which Period = 2**20 -> x = -1 and Period = 2**21 -> x = +1 exactly
  constant GAIN_EDGE   : integer := 2 ** 17;
  constant OFFSET_EDGE : integer := -3 * 2 ** 24;

  constant SET_A : CoefInt_t := (Raw(45.0), Raw(3.25), Raw(-1.5), Raw(0.125), Raw(-0.0625), Raw(0.01));
  constant SET_B : CoefInt_t := (Raw(50.0), Raw(-30.0), Raw(20.0), Raw(-10.0), Raw(8.0), Raw(-5.0));
  constant SET_C : CoefInt_t := (Raw(1.0), 0, 0, 0, 0, 0);          -- y = x
  constant SET_D : CoefInt_t := (0, 0, 0, 0, 0, Raw(1.0));          -- y = T6(x)
  constant SET_E : CoefInt_t := (Raw(-20.0), Raw(15.0), Raw(-12.0), Raw(9.0), Raw(-6.0), Raw(3.0));
  constant SET_Z : CoefInt_t := (0, 0, 0, 0, 0, 0);

begin

  p_Clk : process
  begin
    while not SimDone loop
      ClkxC <= '0';
      wait for CLK_PERIOD / 2;
      ClkxC <= '1';
      wait for CLK_PERIOD / 2;
    end loop;
    wait;
  end process p_Clk;

  dut : entity work.PeriodNormLin
    port map (
      ClkxCI             => ClkxC,
      PeriodEcsxDI       => PeriodEcs,
      PeriodEcsValidxSI  => PeriodValid,
      GainNormxDI        => GainNorm,
      OffsetNormxDI      => OffsetNorm,
      ChebyshevCoeffxDI  => Coeff,
      ValidxSO           => Valid,
      PeriodxDO          => PeriodOut
    );

  p_Stim : process
    variable g_cur      : integer := 0;
    variable o_cur      : integer := 0;
    variable c_cur      : CoefInt_t := (others => 0);
    variable n_checks   : natural := 0;
    variable worst_err  : real := 0.0;   -- in output LSB
    variable worst_rel  : real := 0.0;   -- error / allowed bound
    variable seed1      : positive := 4711;
    variable seed2      : positive := 815;
    variable rnd        : real;
    variable per        : natural;

    procedure SetParams(g : integer; o : integer; c : CoefInt_t) is
    begin
      g_cur := g;
      o_cur := o;
      c_cur := c;
      GainNorm   <= to_signed(g, 32);
      OffsetNorm <= to_signed(o, 32);
      for k in 1 to 6 loop
        Coeff(k) <= to_signed(c(k), 32);
      end loop;
    end procedure SetParams;

    -- floating point reference, returns y [LSB] and the allowed error [LSB]
    procedure RefModel(period : natural; y : out real; xr : out real; bound : out real) is
      variable t : real_vector(0 to 6);
      variable s : real := 0.0;
      variable b : real := 0.5 + 6.0 * 2.0 ** (-9) + 0.001;
    begin
      xr := real(period) * real(g_cur) / 2.0 ** 36 + real(o_cur) / 2.0 ** 24;
      t(0) := 1.0;
      t(1) := xr;
      for k in 2 to 6 loop
        t(k) := 2.0 * xr * t(k - 1) - t(k - 2);
      end loop;
      for k in 1 to 6 loop
        s := s + real(c_cur(k)) * t(k);
        b := b + abs(real(c_cur(k))) * (real(k * k) + real(k * (k - 1)) / 2.0) * 2.0 ** (-33);
      end loop;
      y     := s;
      bound := b;
    end procedure RefModel;

    procedure Compare(period : natural; label_s : string) is
      variable y_ref, x_ref, bnd, err : real;
      variable got : integer;
    begin
      RefModel(period, y_ref, x_ref, bnd);
      assert abs(x_ref) <= 1.0
        report "TB error: x out of [-1,1] for period " & integer'image(period)
        severity failure;
      assert abs(y_ref) < 2.0 ** 31
        report "TB error: reference output does not fit Q8.24" severity failure;
      got := to_integer(PeriodOut);
      err := real(got) - y_ref;
      n_checks := n_checks + 1;
      if abs(err) > worst_err then
        worst_err := abs(err);
      end if;
      if abs(err) / bnd > worst_rel then
        worst_rel := abs(err) / bnd;
      end if;
      assert abs(err) <= bnd
        report label_s & ": period " & integer'image(period) & " got " & integer'image(got)
               & " ref " & real'image(y_ref) & " err " & real'image(err)
               & " LSB > bound " & real'image(bnd)
        severity failure;
    end procedure Compare;

    -- one computation triggered by a single-cycle PeriodEcsValidxSI pulse
    procedure RunPulse(period : natural; label_s : string) is
      variable t0  : time;
      variable lat : integer;
    begin
      PeriodEcs <= to_unsigned(period, 24);
      wait until rising_edge(ClkxC);
      PeriodValid <= '1';
      wait until rising_edge(ClkxC);            -- DUT samples PeriodValid = '1' here
      t0 := now;
      PeriodValid <= '0';
      wait until Valid = '1' for 5 us;
      assert Valid = '1' report label_s & ": timeout waiting for ValidxSO" severity failure;
      lat := (now - t0) / CLK_PERIOD;
      assert lat = LATENCY
        report label_s & ": latency " & integer'image(lat) & " /= " & integer'image(LATENCY)
        severity failure;
      Compare(period, label_s);
      -- ValidxSO must be a single-cycle pulse
      wait for CLK_PERIOD / 2;
      assert Valid = '1' report label_s & ": ValidxSO fell early" severity failure;
      wait for CLK_PERIOD;
      assert Valid = '0' report label_s & ": ValidxSO longer than one cycle" severity failure;
      wait until rising_edge(ClkxC);
    end procedure RunPulse;

    procedure Sweep(label_s : string; n_random : natural) is
      variable err_before : real := worst_err;
    begin
      worst_err := 0.0;
      RunPulse(1000000, label_s & " Pmin");
      RunPulse(2000000, label_s & " Pmax");
      RunPulse(1500000, label_s & " Pmid");
      RunPulse(1000001, label_s & " Pmin+1");
      RunPulse(1999999, label_s & " Pmax-1");
      for i in 1 to n_random loop
        uniform(seed1, seed2, rnd);
        per := 1000000 + integer(floor(rnd * 1000001.0));
        RunPulse(per, label_s & " rnd");
      end loop;
      report label_s & ": worst error " & real'image(worst_err) & " LSB";
      if err_before > worst_err then
        worst_err := err_before;
      end if;
    end procedure Sweep;

    variable rc : CoefInt_t;

  begin
    wait for 5 * CLK_PERIOD;
    assert Valid = '0' report "ValidxSO not '0' after power-up" severity failure;
    assert PeriodOut = 0 report "PeriodxDO not 0 after power-up" severity failure;

    ------------------------------------------------------------------------
    -- 1. functional sweeps, typical normalisation
    ------------------------------------------------------------------------
    SetParams(GAIN_TYP, OFFSET_TYP, SET_A);  Sweep("setA", 60);
    SetParams(GAIN_TYP, OFFSET_TYP, SET_B);  Sweep("setB", 60);
    SetParams(GAIN_TYP, OFFSET_TYP, SET_C);  Sweep("setC", 30);
    SetParams(GAIN_TYP, OFFSET_TYP, SET_D);  Sweep("setD", 30);
    SetParams(GAIN_TYP, OFFSET_TYP, SET_E);  Sweep("setE", 60);
    SetParams(GAIN_TYP, OFFSET_TYP, SET_Z);  Sweep("setZ", 5);

    -- random coefficient sets, sum |C| <= 120 so the output always fits Q8.24
    for s in 1 to 8 loop
      for k in 1 to 6 loop
        uniform(seed1, seed2, rnd);
        rc(k) := Raw((rnd - 0.5) * 40.0);
      end loop;
      SetParams(GAIN_TYP, OFFSET_TYP, rc);
      Sweep("rndset", 30);
    end loop;

    ------------------------------------------------------------------------
    -- 2. corner: x = -1 and x = +1 exactly (2*x*T = +/-2.0 inside the recurrence)
    ------------------------------------------------------------------------
    SetParams(GAIN_EDGE, OFFSET_EDGE, SET_B);
    RunPulse(2 ** 20, "edge x=-1 B");
    RunPulse(2 ** 21, "edge x=+1 B");
    RunPulse(3 * 2 ** 19, "edge x=0 B");
    SetParams(GAIN_EDGE, OFFSET_EDGE, SET_E);
    RunPulse(2 ** 20, "edge x=-1 E");
    RunPulse(2 ** 21, "edge x=+1 E");
    SetParams(GAIN_EDGE, OFFSET_EDGE, SET_D);
    RunPulse(2 ** 20, "edge x=-1 D");
    RunPulse(2 ** 21, "edge x=+1 D");
    -- full-scale coefficients, only a single term so that the output stays in range
    SetParams(GAIN_EDGE, OFFSET_EDGE, (Raw(127.99), 0, 0, 0, 0, 0));
    RunPulse(2 ** 21, "edge fullscale +");
    SetParams(GAIN_EDGE, OFFSET_EDGE, (0, 0, 0, 0, 0, Raw(-127.99)));
    RunPulse(2 ** 21, "edge fullscale -");

    ------------------------------------------------------------------------
    -- 3. level handshake: PeriodEcsValidxSI held high
    ------------------------------------------------------------------------
    SetParams(GAIN_TYP, OFFSET_TYP, SET_A);
    PeriodEcs <= to_unsigned(1234567, 24);
    wait until rising_edge(ClkxC);
    PeriodValid <= '1';
    wait until Valid = '1' for 5 us;
    assert Valid = '1' report "level: timeout" severity failure;
    Compare(1234567, "level first");
    for i in 1 to 400 loop                      -- > 6 back-to-back computations
      wait until rising_edge(ClkxC);
      assert Valid = '1' report "level: ValidxSO dropped while PeriodEcsValidxSI = '1'" severity failure;
    end loop;
    Compare(1234567, "level held");
    PeriodValid <= '0';
    wait until Valid = '0' for 5 us;
    assert Valid = '0' report "level: ValidxSO did not return to '0'" severity failure;

    wait for 10 * CLK_PERIOD;
    report "TB PASSED: " & integer'image(n_checks) & " results checked, worst error "
           & real'image(worst_err) & " LSB (Q8.24), worst error/bound "
           & real'image(worst_rel);
    SimDone <= true;
    wait;
  end process p_Stim;

end architecture sim;
