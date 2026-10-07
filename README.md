# AngleCompute

VHDL-2008 implementation of `AngleCompute` for a Microchip RTG4 at 100 MHz: four ECS periods are
normalised (`P*G+O`) and linearised (Chebyshev polynomial of order `ORDER`), combined into
`DeltaECSX/Z`, rotated by the sin/cos terms and converted to a 16-bit full-circle angle
(`65536 = 2*pi`).  `ValidxSO` pulses for one clock when `AnglexDO` is valid.

| file | content |
|---|---|
| `src/AngleComputePkg.vhd` | port types (`EcsPeriod_t`, `EcsParam_t`, `ChebyshevCoeff_t`, `ChebyshevCoeffArray_t`), fixed-point constants |
| `src/MulSigned35.vhd` | pipelined 35x35 multiplier = four 18x18 products (4 RTG4 math blocks) |
| `src/WideAddSub.vhd` | 72-bit add/subtract in two clocks |
| `src/PairNormalizer.vhd` | common left shift of two words (block floating point) |
| `src/CordicAtan2.vhd` | full-circle atan2 (CORDIC, shift and add) |
| `src/EcsLinearizer.vhd` | `P*G+O` and the Chebyshev polynomial of one ECS (reused for the four ECS) |
| `src/AngleCompute.vhd` | the requested entity |
| `tb/tb_AngleCompute.vhd` | self-checking testbench (double-precision reference, no files) |
| `sim/run_ghdl.sh` | compile and run the testbench with GHDL: `sim/run_ghdl.sh [ORDER [NRANDOM]]` |

Compile order is the order of the table.  If the four port types already exist in another
package of your project, delete them from `AngleComputePkg` and `use` your package instead.

## How the specification was interpreted

* **Angle**: `atan2(Num, Den)` with `Num = CosThetaZ*DX - SinThetaX*DZ`,
  `Den = SinThetaZ*DX + CosThetaX*DZ` (full circle, as "on full circle" asks), rounded to nearest and
  wrapped into 0..65535.  A zero vector gives 0.
* **Formats**: Gain is read as Q-4.36 (raw/2^36), Offset as Q8.24 (raw/2^24).  No format is given for
  the Chebyshev coefficients, Gamma and sin/cos: they are used as plain integers.  The angle is a
  ratio, so only the common scale of the coefficients/Gamma and of the four sin/cos values matters,
  not the position of the binary point.
* **Protocol**: `StartxSI` is sampled as a level while idle (a held Start re-arms the computation
  after `ValidxSO`).  `PeriodEcsValidxSI(n)` may be a pulse or a level.  All inputs are used
  directly (not latched), as allowed.  There is no reset port: all registers have initial values.
* **No clamping or saturation**: all words are sized for the worst case (`Tk*ck` is accumulated
  exactly); modular arithmetic is only used where the true result is known to fit.
* **Latency** after the last period valid: about `17*ORDER + 430` clocks (530 for `ORDER = 6`, 5.3 us).

## Accuracy

The result is the correctly rounded angle (error about 0.5 LSB of 2*pi/65536, worst measured 0.502) except
within ~0.002 LSB of a rounding boundary, as long as the linearised differences are not tiny compared
with the coefficients.  With extreme common-mode cancellation (|DeltaECS| about 1e-6 of the coefficient
scale, i.e. ~120 dB) the error grows to about 1 LSB.

## Verification status

* `tb_AngleCompute` passes for `ORDER` = 1..9, 12 and 13 (axes and diagonal with exact expected angles,
  x = +1/0/-1, extreme coefficients/Gamma/sin-cos, random sensor set-ups, three handshake styles,
  Start while busy, held Start, `ValidxSO` width).  Injected bugs (sign errors, missing pre-rotation,
  wrong formats, two-clock `ValidxSO`, ...) are all caught.
* The RTL was also compared bit by bit with a Python integer model of the same arithmetic over about
  40 000 system-level and 1.4 million block-level vectors (model and vector generators are not part
  of this repository).
* Synthesised only with an open-source flow (yosys + GHDL, ECP5 mapping: about 2.4k LUT4, 3.0k flip-flops,
  8 18x18 multipliers, no latches, no combinational loops).  **Timing closure on RTG4 was not run (no
  Libero here).**  By construction every datapath path is: operand register, at most one LUT level, one
  carry chain of at most 38 bits, result register (multipliers are 18x18 with registered inputs and
  outputs); the remaining paths are state decode and small 4:1 multiplexers (two or three LUT levels).
  Please confirm with Libero; the paths to watch are the control logic and the wide adders.
