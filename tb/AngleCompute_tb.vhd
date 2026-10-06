-- File-driven testbench for AngleCompute.
--
-- Reads one test vector per line from VECTOR_FILE (whitespace separated integers):
--   P(1..4)  Gain(1..4)  Offset(1..4)  Coeff(ch1, 1..ORDER) ... Coeff(ch4, 1..ORDER)
--   GammaX GammaZ  CosThetaX SinThetaX CosThetaZ SinThetaZ
--   ValidDelay(1..4)  ValidMode          (0 = valid held high, 1 = valid is a 1-clock pulse)
-- ValidDelay(i) is the number of clocks after the start pulse at which the period
-- valid of channel i is raised, so every arrival order can be exercised.
-- Writes "<angle> <clock cycles from start to ValidxSO>" per vector to RESULT_FILE.
-- Compare with tb/check_results.py.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library std;
use std.textio.all;
library work;
use work.EcsTypes_pkg.all;

entity AngleCompute_tb is
  generic(
    ORDER       : positive := 6;
    VECTOR_FILE : string   := "vectors.txt";
    RESULT_FILE : string   := "results.txt"
  );
end entity AngleCompute_tb;

architecture sim of AngleCompute_tb is

  constant CLK_PERIOD : time := 10 ns;                     -- 100 MHz
  constant TIMEOUT    : natural := 20000;                  -- clocks

  signal ClkxC    : std_logic := '0';
  signal StartxS  : std_logic := '0';
  signal ValidxS  : std_logic;
  signal AnglexD  : unsigned(15 downto 0);
  signal PeriodxD : EcsPeriod_t(1 to 4) := (others => (others => '0'));
  signal PerValxS : std_logic_vector(1 to 4) := (others => '0');
  signal GainxD   : EcsGain_t(1 to 4) := (others => (others => '0'));
  signal OffsetxD : EcsOffset_t(1 to 4) := (others => (others => '0'));
  signal CoeffxD  : ChebyshevCoeffArray_t(1 to 4)(1 to ORDER) := (others => (others => (others => '0')));
  signal GammaXxD, GammaZxD : signed(31 downto 0) := (others => '0');
  signal CosXxD, SinXxD, CosZxD, SinZxD : signed(31 downto 0) := (others => '0');

  signal ValidSeenxS : boolean := false;

begin

  ClkxC <= not ClkxC after CLK_PERIOD / 2;

  dut : entity work.AngleCompute
    generic map(ORDER => ORDER)
    port map(
      ClkxCI            => ClkxC,
      StartxSI          => StartxS,
      ValidxSO          => ValidxS,
      AnglexDO          => AnglexD,
      PeriodEcsxDI      => PeriodxD,
      PeriodEcsValidxSI => PerValxS,
      GainNormxDI       => GainxD,
      OffsetNormxDI     => OffsetxD,
      ChebyshevCoeffxDI => CoeffxD,
      GammaXxDI         => GammaXxD,
      GammaZxDI         => GammaZxD,
      CosThetaXxDI      => CosXxD,
      SinThetaXxDI      => SinXxD,
      CosThetaZxDI      => CosZxD,
      SinThetaZxDI      => SinZxD
    );

  -- ValidxSO must be exactly one clock wide
  p_valid_width : process(ClkxC)
    variable Prev : std_logic := '0';
  begin
    if rising_edge(ClkxC) then
      assert not (ValidxS = '1' and Prev = '1')
        report "ValidxSO is longer than one clock cycle" severity failure;
      Prev := ValidxS;
    end if;
  end process;

  p_stim : process
    file fin  : text open read_mode  is VECTOR_FILE;
    file fout : text open write_mode is RESULT_FILE;
    variable L, LO   : line;
    variable V       : integer;
    variable Delay   : integer_vector(1 to 4);
    variable Mode    : integer;
    variable Cyc     : natural;
    variable Done    : boolean;
    variable Count   : natural := 0;
  begin
    wait for 10 * CLK_PERIOD;
    while not endfile(fin) loop
      readline(fin, L);
      if L'length = 0 or L(L'left) = '#' then
        next;
      end if;
      for i in 1 to 4 loop read(L, V); PeriodxD(i) <= to_unsigned(V, 24); end loop;
      for i in 1 to 4 loop read(L, V); GainxD(i)   <= to_signed(V, 32);   end loop;
      for i in 1 to 4 loop read(L, V); OffsetxD(i) <= to_signed(V, 32);   end loop;
      for i in 1 to 4 loop
        for k in 1 to ORDER loop
          read(L, V); CoeffxD(i)(k) <= to_signed(V, 32);
        end loop;
      end loop;
      read(L, V); GammaXxD <= to_signed(V, 32);
      read(L, V); GammaZxD <= to_signed(V, 32);
      read(L, V); CosXxD   <= to_signed(V, 32);
      read(L, V); SinXxD   <= to_signed(V, 32);
      read(L, V); CosZxD   <= to_signed(V, 32);
      read(L, V); SinZxD   <= to_signed(V, 32);
      for i in 1 to 4 loop read(L, Delay(i)); end loop;
      read(L, Mode);

      -- start pulse
      wait until rising_edge(ClkxC);
      wait until rising_edge(ClkxC);
      StartxS <= '1';
      wait until rising_edge(ClkxC);
      StartxS <= '0';

      Cyc  := 0;
      Done := false;
      while not Done loop
        wait until rising_edge(ClkxC);
        Cyc := Cyc + 1;
        -- ValidxS / AnglexD are sampled before this edge's updates (registered outputs)
        if ValidxS = '1' then
          Done := true;
          write(LO, to_integer(AnglexD));
          write(LO, string'(" "));
          write(LO, Cyc);
          writeline(fout, LO);
        end if;
        for i in 1 to 4 loop
          if Mode = 0 then
            if Cyc >= Delay(i) then PerValxS(i) <= '1'; end if;
          else
            if Cyc = Delay(i) then PerValxS(i) <= '1'; else PerValxS(i) <= '0'; end if;
          end if;
        end loop;
        assert Cyc < TIMEOUT report "timeout waiting for ValidxSO" severity failure;
      end loop;
      PerValxS <= (others => '0');
      Count := Count + 1;
    end loop;
    report "done: " & integer'image(Count) & " vectors";
    std.env.finish;
  end process;

end architecture sim;
