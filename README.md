# AngleCompute (VHDL-2008, Microchip RTG4 @ 100 MHz)

Computes the angle from four ECS period measurements:

1. per channel: `X = P*Gain + Offset`, then `Lin = Σ Tk(X)·Ck` (Chebyshev, `ORDER` terms)
2. `ΔX = LinXP − LinXN + GammaX`, `ΔZ = LinZP − LinZN + GammaZ`  (channels 1..4 = XP, XN, ZP, ZN)
3. `Num = CosZ·ΔX − SinX·ΔZ`, `Den = SinZ·ΔX + CosX·ΔZ`
4. `AnglexDO = atan2(Num, Den)` on the full circle (65536 = 2π), `ValidxSO` pulses for one clock

| File | Content |
|---|---|
| `src/AngleCompute.vhd` | top level (entity as specified), sequencing, ΔX/ΔZ, final products |
| `src/ChebyshevLinearize.vhd` | normalisation + Chebyshev polynomial of one channel (instantiated 4×) |
| `src/CordicAtan2.vhd` | scaling + iterative vectoring CORDIC, 16-bit full-circle output |
| `src/MulSigned.vhd` | 3-stage pipelined signed multiplier (maps onto RTG4 math blocks) |
| `src/EcsTypes_pkg.vhd` | the array types of the port list (skip if your project already has them) |
| `tb/` | file-driven testbench, Python golden model / vector generator, `run_sim.sh` |

Compile order: `EcsTypes_pkg, MulSigned, ChebyshevLinearize, CordicAtan2, AngleCompute`.

## Interpretation choices (please check)

* **Full-circle angle**: the spec says "angle on full circle", so the quadrant is resolved
  (`atan2`) instead of a plain `arctan(Num/Den)`, which would only cover ±π/2.
* **Coefficient / Gamma / sin-cos formats are not specified** and are not needed: all
  Chebyshev coefficients and Gamma are treated as integers with one common (arbitrary) scale, and
  the four sin/cos inputs likewise. Both scales cancel in the arctangent.
* **No clamping/saturation, nothing overflows**: all widths are the worst case for 32-bit
  coefficients (`LIN_W = 32+⌈log2(ORDER+1)⌉`, `DELTA_W = LIN_W+1`, `PROD_W = DELTA_W+32`, products
  and `Num/Den` are exact). The only reductions are rounding/truncation of LSBs
  (Chebyshev recurrence in Q2.30, 4 guard bits in the sum, CORDIC on a 32-bit scaled vector).
* The four channels run **in parallel**, each waiting for its own `PeriodEcsValidxSI(i)`; the
  result is identical to the sequential flow and does not depend on the arrival order.
* Inputs are not latched (as allowed): hold them stable from `StartxSI` to `ValidxSO`.
  `StartxSI` is level-sensitive while idle. Registers rely on initial values (no reset port).
* Output: `AnglexDO` is updated in the same clock as `ValidxSO` and held afterwards.

## Verification

`tb/run_sim.sh [ORDER] [RANDOM_VECTORS] [SEED]` (needs GHDL ≥ 2.0 and Python 3) simulates the RTL
and compares it against an exact-rational Python model (ideal angle, no fixed-point effects).
Directed vectors: all octants/axes, normalised period exactly ±1, all coefficients/Gamma/sin/cos at
full scale (|Δ| = (2·ORDER+1)·(2³¹−1), the width worst case). Random vectors vary the gain/offset,
coefficient style/scale, sin/cos, and the order in which the valids arrive (level or 1-clock pulses).

Result: ORDER = 1, 2, 3, 4, 6, 7, 8 all pass, **max error 0.52 LSB of the 16-bit output** (the floor
is 0.5 LSB from the final rounding); 746 vectors for ORDER = 6. Mutations (one bit too narrow
`DELTA_W`/`VEC_W`, wrong sign, missing recurrence term) are detected by the test set.
GHDL `--synth` accepts the design; Yosys (generic LUT4 mapping) gives about 5.4k LUT4 + 1.7k FF
plus 4 × (32×32) and 1 × (36×32) multipliers — a rough estimate, not RTG4 place & route.

Timing at 100 MHz has **not** been verified with Libero/Synplify (not available here). The longest
paths are the 68-bit `Num/Den` add/subtract and the ~40-bit sign-copy detection in the CORDIC
scaling; the multipliers have 3 register stages for the math blocks. If a path fails, those are
the places to pipeline further (latency is irrelevant here).
