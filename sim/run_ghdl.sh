#!/bin/sh
# Compile the AngleCompute sources and run the self-checking testbench with GHDL.
#   usage: sim/run_ghdl.sh [ORDER [NRANDOM]]      (defaults: ORDER = 6, NRANDOM = 300)
# Prints "TEST PASSED" and exits with 0 on success.  Work files go to $WORK (default: a temp dir).
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
ORDER=${1:-6}
NRANDOM=${2:-300}
WORK=${WORK:-$(mktemp -d)}
cd "$WORK"
ghdl -a --std=08 \
  "$ROOT/src/AngleComputePkg.vhd" \
  "$ROOT/src/MulSigned35.vhd" \
  "$ROOT/src/WideAddSub.vhd" \
  "$ROOT/src/PairNormalizer.vhd" \
  "$ROOT/src/CordicAtan2.vhd" \
  "$ROOT/src/EcsLinearizer.vhd" \
  "$ROOT/src/AngleCompute.vhd" \
  "$ROOT/tb/tb_AngleCompute.vhd"
ghdl -e --std=08 tb_AngleCompute
ghdl -r --std=08 tb_AngleCompute -gORDER="$ORDER" -gNRANDOM="$NRANDOM"
