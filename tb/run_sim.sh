#!/usr/bin/env bash
# Regression: GHDL (VHDL-2008) simulation of AngleCompute against the Python golden model.
#   tb/run_sim.sh [ORDER=6] [RANDOM_VECTORS=300] [SEED=1]
# Work files go to $WORK (default: a fresh temp dir).
set -euo pipefail
ORDER=${1:-6}; COUNT=${2:-300}; SEED=${3:-1}
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$HERE/../src
WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK" && cd "$WORK"
for f in EcsTypes_pkg MulSigned ChebyshevLinearize CordicAtan2 AngleCompute; do
  ghdl -a --std=08 --workdir=. "$SRC/$f.vhd"
done
ghdl -a --std=08 --workdir=. "$HERE/AngleCompute_tb.vhd"
python3 -I "$HERE/angle_model.py" gen --order "$ORDER" --count "$COUNT" --seed "$SEED" --out vectors.txt
ghdl -r --std=08 --workdir=. AngleCompute_tb -gORDER="$ORDER" \
     -gVECTOR_FILE=vectors.txt -gRESULT_FILE=results.txt
python3 -I "$HERE/angle_model.py" check --order "$ORDER" --vectors vectors.txt --results results.txt
