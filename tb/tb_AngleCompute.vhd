-------------------------------------------------------------------------------
-- Self-checking testbench for AngleCompute (VHDL-2008)
--   Golden model: double precision (math_real) Chebyshev series + atan2.
--   Pass criterion: circular error <= 1 LSB (2*pi/65536).
--   ghdl -a --std=08 src/AngleComputePkg.vhd src/SerialMul.vhd \
--        src/EcsLinearizer.vhd src/AngleCompute.vhd tb/tb_AngleCompute.vhd
--   ghdl -r --std=08 tb_AngleCompute
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.AngleComputePkg.all;

entity tb_AngleCompute is
end entity tb_AngleCompute;

architecture sim of tb_AngleCompute is

  constant ORDER : positive := 6;
  constant TCLK  : time     := 10 ns;

  signal Clk       : std_logic := '0';
  signal Start     : std_logic := '0';
  signal Valid     : std_logic;
  signal Angle     : unsigned(15 downto 0);
  signal Period    : EcsPeriod_t(1 to 4)                         := (others => to_unsigned(1500000, 24));
  signal PerValid  : std_logic_vector(1 to 4)                    := (others => '0');
  signal Gain      : EcsGain_t(1 to 4)                           := (others => to_signed(137439, 32));
  signal Offset    : EcsOffset_t(1 to 4)                         := (others => to_signed(-3 * 2 ** 24, 32));
  signal Coeff     : ChebyshevCoeffArray_t(1 to 4)(1 to ORDER)   := (others => (others => (others => '0')));
  signal GammaX    : signed(31 downto 0) := (others => '0');
  signal GammaZ    : signed(31 downto 0) := (others => '0');
  signal CosThX    : signed(31 downto 0) := to_signed(2 ** 30, 32);
  signal SinThX    : signed(31 downto 0) := (others => '0');
  signal CosThZ    : signed(31 downto 0) := to_signed(2 ** 30, 32);
  signal SinThZ    : signed(31 downto 0) := (others => '0');

  type RealArr_t is array (1 to 4) of real;
  signal ValidCnt  : integer := 0;
  signal Done      : boolean := false;

begin

  Clk <= not Clk after TCLK / 2 when not Done else '0';

  dut : entity work.AngleCompute
    generic map(ORDER => ORDER)
    port map(ClkxCI            => Clk,
             StartxSI          => Start,
             ValidxSO          => Valid,
             AnglexDO          => Angle,
             PeriodEcsxDI      => Period,
             PeriodEcsValidxSI => PerValid,
             GainNormxDI       => Gain,
             OffsetNormxDI     => Offset,
             ChebyshevCoeffxDI => Coeff,
             GammaXxDI         => GammaX,
             GammaZxDI         => GammaZ,
             CosThetaXxDI      => CosThX,
             SinThetaXxDI      => SinThX,
             CosThetaZxDI      => CosThZ,
             SinThetaZxDI      => SinThZ);

  -- ValidxSO must be a single-cycle pulse
  process(Clk)
    variable Prev : std_logic := '0';
  begin
    if rising_edge(Clk) then
      assert not (Valid = '1' and Prev = '1') report "ValidxSO longer than 1 cycle" severity failure;
      if Valid = '1' then
        ValidCnt <= ValidCnt + 1;
      end if;
      Prev := Valid;
    end if;
  end process;

  main : process
    variable S1, S2     : positive := 1234;
    variable Rnd        : real;
    variable NErr       : integer := 0;
    variable MaxErr     : real := 0.0;
    variable NTest      : integer := 0;
    variable T0         : time;

    impure function Rand(Lo, Hi : real) return real is
    begin
      uniform(S1, S2, Rnd);
      return Lo + (Hi - Lo) * Rnd;
    end function;

    -- Chebyshev series sum_{k=1..ORDER} Tk(x)*c(k)  (direct recurrence)
    function Cheb(x : real; c : ChebyshevCoeff_t) return real is
      variable Tkm1, Tk, Tkp1, Acc : real;
    begin
      Tkm1 := 1.0;
      Tk   := x;
      Acc  := 0.0;
      for k in 1 to ORDER loop
        Acc  := Acc + Tk * real(to_integer(c(k)));
        Tkp1 := 2.0 * x * Tk - Tkm1;
        Tkm1 := Tk;
        Tk   := Tkp1;
      end loop;
      return Acc;
    end function;

    function NormX(p : unsigned(23 downto 0); g, o : signed(31 downto 0)) return real is
    begin
      return real(to_integer(p)) * real(to_integer(g)) / 2.0 ** 36 + real(to_integer(o)) / 2.0 ** 24;
    end function;

    procedure RunAndCheck(Staggered : boolean; Tag : string) is
      variable L   : RealArr_t;
      variable DX, DZ, Num, Den, Ang, Exp, Err : real;
      variable Got : real;
      variable Cyc : integer;
      variable N0  : integer;
    begin
      -- golden model
      for n in 1 to 4 loop
        L(n) := Cheb(NormX(Period(n), Gain(n), Offset(n)), Coeff(n));
      end loop;
      DX  := L(1) - L(2) + real(to_integer(GammaX));
      DZ  := L(3) - L(4) + real(to_integer(GammaZ));
      Num := real(to_integer(CosThZ)) * DX - real(to_integer(SinThX)) * DZ;
      Den := real(to_integer(SinThZ)) * DX + real(to_integer(CosThX)) * DZ;
      if Num = 0.0 and Den = 0.0 then
        Ang := 0.0;
      else
        Ang := arctan(Num, Den);
      end if;
      Exp := Ang / MATH_2_PI * 65536.0;
      if Exp < 0.0 then
        Exp := Exp + 65536.0;
      end if;

      -- stimulus
      N0 := ValidCnt;
      PerValid <= (others => '0');
      wait until rising_edge(Clk);
      Start <= '1';
      if not Staggered then
        PerValid <= (others => '1');
      end if;
      wait until rising_edge(Clk);
      Start <= '0';
      T0 := now;
      if Staggered then
        wait for 30 * TCLK; PerValid(1) <= '1';
        wait for 400 * TCLK; PerValid(3) <= '1';   -- 3 before 2/4: must wait in order
        wait for 100 * TCLK; PerValid(2) <= '1';
        wait for 60 * TCLK; PerValid(4) <= '1';
      end if;
      wait until rising_edge(Clk) and Valid = '1' for 200 us;
      assert Valid = '1' report Tag & ": timeout" severity failure;
      wait for 1 ns;
      Cyc := (now - T0) / TCLK;
      Got := real(to_integer(Angle));
      Err := abs(Got - Exp);
      if Err > 32768.0 then
        Err := 65536.0 - Err;
      end if;
      NTest := NTest + 1;
      if Err > MaxErr and Tag /= "zero" then MaxErr := Err; end if;
      if Err > 1.0 and Tag /= "zero" then   -- atan2(0,0) is undefined
        NErr := NErr + 1;
        report Tag & ": MISMATCH got " & real'image(Got) & " expected " & real'image(Exp)
          & " err " & real'image(Err) severity error;
      end if;
      wait for 20 * TCLK;
      assert ValidCnt = N0 + 1 report Tag & ": ValidxSO pulse count wrong" severity failure;
      if Tag = "dir0" then
        report "latency (Start -> Valid, all periods valid): " & integer'image(Cyc) & " cycles";
      end if;
    end procedure;

    procedure RandomParams is
      variable Ang1, Ang2 : real;
    begin
      for n in 1 to 4 loop
        Period(n) <= to_unsigned(integer(Rand(1000000.0, 1999990.0)), 24);
        Gain(n)   <= to_signed(137439 + integer(Rand(-20.0, 20.0)), 32);
        Offset(n) <= to_signed(-3 * 2 ** 24 + integer(Rand(-5000.0, 5000.0)), 32);
        Coeff(n)(1) <= to_signed(2 ** 30 + integer(Rand(-1.0e8, 1.0e8)), 32);
        for k in 2 to ORDER loop
          Coeff(n)(k) <= to_signed(integer(Rand(-5.0e7, 5.0e7)), 32);
        end loop;
      end loop;
      GammaX <= to_signed(integer(Rand(-1.0e8, 1.0e8)), 32);
      GammaZ <= to_signed(integer(Rand(-1.0e8, 1.0e8)), 32);
      Ang1 := Rand(-MATH_PI, MATH_PI);
      Ang2 := Rand(-MATH_PI, MATH_PI);
      CosThX <= to_signed(integer(cos(Ang1) * 2.0 ** 30), 32);
      SinThX <= to_signed(integer(sin(Ang1) * 2.0 ** 30), 32);
      CosThZ <= to_signed(integer(cos(Ang2) * 2.0 ** 30), 32);
      SinThZ <= to_signed(integer(sin(Ang2) * 2.0 ** 30), 32);
      wait for 1 ns;
    end procedure;

    procedure SetPeriods(a, b, c, d : integer) is
    begin
      Period <= (to_unsigned(a, 24), to_unsigned(b, 24), to_unsigned(c, 24), to_unsigned(d, 24));
      wait for 1 ns;
    end procedure;

  begin
    wait for 5 * TCLK;

    -- directed: identity coefficients (Lin = x*2**30), no rotation, no gamma
    for n in 1 to 4 loop
      Coeff(n)(1) <= to_signed(2 ** 30, 32);
    end loop;
    wait for 1 ns;
    SetPeriods(1750000, 1250000, 1500000, 1500000);  RunAndCheck(false, "dir0");  -- DX>0, DZ=0
    SetPeriods(1500000, 1500000, 1750000, 1250000);  RunAndCheck(false, "dir1");  -- DX=0, DZ>0
    SetPeriods(1250000, 1750000, 1500000, 1500000);  RunAndCheck(false, "dir2");  -- DX<0, DZ=0 (pi)
    SetPeriods(1500000, 1500000, 1250000, 1750000);  RunAndCheck(false, "dir3");  -- DX=0, DZ<0
    SetPeriods(1250000, 1750000, 1250000, 1750000);  RunAndCheck(false, "dir4");  -- 3rd quadrant
    SetPeriods(1750000, 1250000, 1250000, 1750000);  RunAndCheck(false, "dir5");  -- 4th quadrant
    SetPeriods(1300000, 1700000, 1900000, 1100000);  RunAndCheck(false, "dir6");  -- 2nd quadrant
    SetPeriods(1500000, 1500000, 1500000, 1500000);  RunAndCheck(false, "zero");  -- Num = Den = 0

    -- tiny signal (normalization stage)
    SetPeriods(1500000, 1500001, 1500002, 1500000);  RunAndCheck(false, "tiny");

    -- random parameters, full circle
    for i in 1 to 300 loop
      RandomParams;
      RunAndCheck(i mod 10 = 0, "rnd" & integer'image(i));
    end loop;

    -- worst case magnitudes: full-scale coefficients/gammas, x = +-1 (overflow check)
    for i in 1 to 100 loop
      RandomParams;
      for n in 1 to 4 loop
        if Rand(0.0, 1.0) < 0.5 then
          Period(n) <= to_unsigned(1000000, 24);      -- x = -1
        else
          Period(n) <= to_unsigned(1999990, 24);      -- x = +1
        end if;
        for k in 1 to ORDER loop
          if Rand(0.0, 1.0) < 0.5 then
            Coeff(n)(k) <= signed'(x"80000000");
          else
            Coeff(n)(k) <= signed'(x"7FFFFFFF");
          end if;
        end loop;
      end loop;
      GammaX <= signed'(x"80000000");
      GammaZ <= signed'(x"7FFFFFFF");
      wait for 1 ns;
      RunAndCheck(false, "ext" & integer'image(i));
    end loop;

    report "tests: " & integer'image(NTest) & "  failures: " & integer'image(NErr)
      & "  max error (LSB): " & real'image(MaxErr);
    assert NErr = 0 report "TEST FAILED" severity failure;
    report "TEST PASSED";
    Done <= true;
    wait;
  end process;

end architecture sim;
