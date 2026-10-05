-------------------------------------------------------------------------------
-- tb_ChevyPoly : self-checking testbench for ChevyPoly (VHDL-2008)
--
-- Instantiates ChevyPoly for several ORDER values and compares each result with
-- a double precision reference of the formula
--     x = P*G/2**36 + O/2**24 ,  y = sum c_k * T_k(x)
-- The DUT must be within 0.6 LSB of the exact value (0.5 LSB final rounding +
-- internal fixed-point noise). Also checks: exactly one ValidxSO pulse per start,
-- 1 clock wide, and the latency.
--
-- Run (GHDL):
--   ghdl -a --std=08 src/ChevyPolyPkg.vhd src/ChevyPoly.vhd tb/tb_ChevyPoly.vhd
--   ghdl -e --std=08 tb_ChevyPoly
--   ghdl -r --std=08 tb_ChevyPoly --assert-level=error
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.env.all;
use work.ChevyPolyPkg.all;

entity tb_ChevyPoly is
end entity tb_ChevyPoly;

architecture sim of tb_ChevyPoly is

  constant CLK_PERIOD : time     := 10 ns;      -- 100 MHz
  constant FRAC       : natural  := 24;
  constant MAX_ORDER  : positive := 8;
  constant WAIT_CYC   : natural  := 120;        -- idle clocks after each start
  constant MAX_ERR    : real     := 0.6;        -- LSB

  type OrderArr_t is array (natural range <>) of positive;
  constant ORDERS : OrderArr_t(0 to 4) := (1, 2, 3, 6, 8);

  signal ClkxC      : std_logic := '0';
  signal PeriodxD   : unsigned(23 downto 0) := (others => '0');
  signal StartxS    : std_logic := '0';
  signal GainxD     : signed(31 downto 0)   := (others => '0');
  signal OffsetxD   : signed(31 downto 0)   := (others => '0');
  signal CoeffxD    : ChebyshevCoeff_t(1 to MAX_ORDER) := (others => (others => '0'));
  signal DonexS     : boolean := false;

  -- exact reference (double precision)
  function expected(P : unsigned(23 downto 0); G, O : signed(31 downto 0);
                    C : ChebyshevCoeff_t; N : positive) return real is
    variable x, tp, tc, tn, s : real;
  begin
    x  := real(to_integer(P)) * real(to_integer(G)) / 2.0**36
          + real(to_integer(O)) / 2.0**24;
    tp := 1.0;
    tc := x;
    s  := 0.0;
    for k in 1 to N loop
      s  := s + real(to_integer(C(k))) * tc;
      tn := 2.0 * x * tc - tp;
      tp := tc;
      tc := tn;
    end loop;
    return s;
  end function expected;

begin

  ClkxC <= not ClkxC after CLK_PERIOD / 2;

  ---------------------------------------------------------------------------
  -- DUTs + checkers
  ---------------------------------------------------------------------------
  g_dut : for i in ORDERS'range generate
    constant N      : positive := ORDERS(i);
    signal ValidxS  : std_logic;
    signal ResultxD : signed(31 downto 0);
  begin

    u_dut : entity work.ChevyPoly
      generic map (ORDER => N, FRAC => FRAC)
      port map (
        ClkxCI            => ClkxC,
        PeriodEcsxDI      => PeriodxD,
        PeriodEcsValidxSI => StartxS,
        GainNormxDI       => GainxD,
        OffsetNormxDI     => OffsetxD,
        ChebyshevCoeffxDI => CoeffxD(1 to N),
        ValidxSO          => ValidxS,
        PeriodxDO         => ResultxD);

    p_check : process(ClkxC)
      variable v_Prev     : std_logic := '0';
      variable v_Busy     : boolean   := false;
      variable v_Starts   : natural   := 0;
      variable v_Dones    : natural   := 0;
      variable v_Cyc      : natural   := 0;
      variable v_Lat      : natural   := 0;
      variable v_Exp      : real;
      variable v_Err      : real;
      variable v_MaxErr   : real      := 0.0;
      variable v_Reported : boolean   := false;
    begin
      if rising_edge(ClkxC) then
        if StartxS = '1' then
          v_Starts := v_Starts + 1;
          v_Cyc    := 0;
          v_Busy   := true;
        elsif v_Busy then
          v_Cyc := v_Cyc + 1;
        end if;

        if ValidxS = '1' then
          assert v_Prev = '0'
            report "ORDER " & integer'image(N) & ": ValidxSO is longer than 1 clock"
            severity error;
          assert v_Busy
            report "ORDER " & integer'image(N) & ": ValidxSO without a start"
            severity error;
          v_Dones := v_Dones + 1;
          v_Lat   := v_Cyc;
          v_Busy  := false;
          v_Exp   := expected(PeriodxD, GainxD, OffsetxD, CoeffxD(1 to N), N);
          v_Err   := abs(real(to_integer(ResultxD)) - v_Exp);
          if v_Err > v_MaxErr then
            v_MaxErr := v_Err;
          end if;
          assert v_Err <= MAX_ERR
            report "ORDER " & integer'image(N) & ": result " & integer'image(to_integer(ResultxD))
                   & " expected " & real'image(v_Exp) & " (error " & real'image(v_Err) & " LSB)"
            severity error;
        end if;
        v_Prev := ValidxS;

        if DonexS and not v_Reported then
          v_Reported := true;
          assert v_Dones = v_Starts
            report "ORDER " & integer'image(N) & ": " & integer'image(v_Starts)
                   & " starts but " & integer'image(v_Dones) & " valid pulses"
            severity error;
          report "ORDER " & integer'image(N) & ": " & integer'image(v_Dones)
                 & " results checked, max error " & real'image(v_MaxErr)
                 & " LSB, latency " & integer'image(v_Lat) & " clocks";
        end if;
      end if;
    end process p_check;

  end generate g_dut;

  ---------------------------------------------------------------------------
  -- Stimulus
  ---------------------------------------------------------------------------
  p_stim : process
    variable s1, s2 : positive := 1;
    variable v_Coef : ChebyshevCoeff_t(1 to MAX_ORDER);
    variable v_G    : integer;
    variable v_Sc   : real;
    variable v_N    : natural := 0;

    impure function urand(lo, hi : real) return real is
      variable u : real;
    begin
      uniform(s1, s2, u);
      return lo + u * (hi - lo);
    end function urand;

    -- offset that centres the period range 1e6..2e6 on 0 for a given gain
    function center_offset(G : integer) return integer is
    begin
      return -integer(round(1.5e6 * real(G) / 2.0**36 * 2.0**24));
    end function center_offset;

    procedure run_case(P : natural; G, O : integer; C : ChebyshevCoeff_t) is
      variable x : real;
    begin
      x := real(P) * real(G) / 2.0**36 + real(O) / 2.0**24;
      assert abs(x) <= 1.0 report "stimulus error: |PeriodNorm| > 1" severity failure;
      wait until rising_edge(ClkxC);
      PeriodxD <= to_unsigned(P, 24);
      GainxD   <= to_signed(G, 32);
      OffsetxD <= to_signed(O, 32);
      CoeffxD  <= C;
      StartxS  <= '1';
      wait until rising_edge(ClkxC);
      StartxS  <= '0';
      for j in 1 to WAIT_CYC loop
        wait until rising_edge(ClkxC);
      end loop;
      v_N := v_N + 1;
    end procedure run_case;

  begin
    wait for 5 * CLK_PERIOD;

    -- 1) unit coefficient c_k = 1.0 (only T_k), sweep over the period range.
    --    Gain 2e-6 truncated, offset centres the range -> x in about [-0.99999, 0.99999]
    v_G := 137438;
    for k in 1 to MAX_ORDER loop
      v_Coef := (others => (others => '0'));
      v_Coef(k) := to_signed(2**FRAC, 32);
      for P in 0 to 4 loop
        run_case(1_000_000 + P * 250_000, v_G, center_offset(v_G), v_Coef);
      end loop;
    end loop;

    -- 2) x = +1 and x = -1 exactly (P = 2**21, G = 2**15 -> P*G = 2**36)
    for k in 1 to MAX_ORDER loop
      v_Coef := (others => (others => '0'));
      v_Coef(k) := to_signed(2**FRAC, 32);
      run_case(2**21, 2**15, 0, v_Coef);
      run_case(2**21, 2**15, -2 * 2**24, v_Coef);
    end loop;
    for k in 1 to MAX_ORDER loop
      v_Coef := (others => to_signed(1000 * k, 32));
      run_case(2**21, 2**15, 0, v_Coef);
      run_case(2**21, 2**15, -2 * 2**24, v_Coef);
    end loop;

    -- 3) full-scale single coefficient (|c*T_k| <= 2**31-1 fits the output)
    v_G := 137438;
    for k in 1 to MAX_ORDER loop
      v_Coef := (others => (others => '0'));
      v_Coef(k) := to_signed(integer'high, 32);
      run_case(1_000_000, v_G, center_offset(v_G), v_Coef);
      run_case(1_500_000, v_G, center_offset(v_G), v_Coef);
      run_case(1_873_211, v_G, center_offset(v_G), v_Coef);
      run_case(2**21, 2**15, 0, v_Coef);
      v_Coef(k) := to_signed(-integer'high, 32);
      run_case(1_321_987, v_G, center_offset(v_G), v_Coef);
    end loop;

    -- 4) random gain / period / coefficients (coefficient scale 2**10 .. 2**27,
    --    sum of 8 terms always fits 32 bit)
    for n in 1 to 500 loop
      v_G  := integer(urand(100000.0, 137438.0));
      v_Sc := exp(urand(10.0, 27.0) * log(2.0));
      for k in 1 to MAX_ORDER loop
        v_Coef(k) := to_signed(integer(urand(-v_Sc, v_Sc)), 32);
      end loop;
      run_case(integer(urand(1.0e6, 2.0e6)), v_G, center_offset(v_G), v_Coef);
    end loop;

    DonexS <= true;
    wait for 5 * CLK_PERIOD;
    report "tb_ChevyPoly: " & integer'image(v_N) & " test vectors done";
    finish;
  end process p_stim;

end architecture sim;
