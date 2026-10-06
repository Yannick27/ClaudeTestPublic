#!/usr/bin/env bash
# Compile and run the testbenches with GHDL (VHDL-2008).
#   sim/run_ghdl.sh            run everything
#   sim/run_ghdl.sh mul        SerialMulAcc unit tests only
#   sim/run_ghdl.sh cordic     Atan2Cordic unit test only
#   sim/run_ghdl.sh top        AngleCompute system test only
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK="${WORK:-$ROOT/sim/work}"
STD="--std=08"
mkdir -p "$WORK"
cd "$WORK"

SRC="EcsTypesPkg AngleComputePkg SerialMulAcc EcsLinearizer Atan2Cordic AngleCompute"
for f in $SRC; do ghdl -a $STD --workdir="$WORK" "$ROOT/src/$f.vhd"; done
for f in tb_SerialMulAcc tb_Atan2Cordic tb_AngleCompute; do
  ghdl -a $STD --workdir="$WORK" "$ROOT/tb/$f.vhd"
done

run() {  # run <entity> [generic overrides...]
  local ent="$1"; shift
  ghdl -e $STD --workdir="$WORK" "$ent"
  ghdl -r $STD --workdir="$WORK" "$ent" --assert-level=error "$@"
}

what="${1:-all}"

if [[ $what == all || $what == mul ]]; then
  run tb_SerialMulAcc -gWA=32 -gWB=32 -gNT=1
  run tb_SerialMulAcc -gWA=41 -gWB=32 -gNT=2
  run tb_SerialMulAcc -gWA=24 -gWB=32 -gNT=1
  run tb_SerialMulAcc -gWA=5  -gWB=7  -gNT=3
  run tb_SerialMulAcc -gWA=16 -gWB=17 -gNT=2
  run tb_SerialMulAcc -gWA=48 -gWB=33 -gNT=4
fi
if [[ $what == all || $what == cordic ]]; then
  run tb_Atan2Cordic
fi
if [[ $what == all || $what == top ]]; then
  run tb_AngleCompute
fi
