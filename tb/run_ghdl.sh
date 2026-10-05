#!/bin/sh
# Compile and run the AngleCompute testbench with GHDL (VHDL-2008)
set -e
cd "$(dirname "$0")/.."
W=${TMPDIR:-/tmp}/AngleCompute_ghdl
mkdir -p "$W"
ghdl -a --std=08 --workdir="$W" src/AngleCompute_pkg.vhd src/MulPipe.vhd src/BlockNormalizer.vhd \
     src/ChebyLinearizer.vhd src/CordicAtan2.vhd src/AngleCompute.vhd tb/AngleCompute_tb.vhd
ghdl -e --std=08 --workdir="$W" AngleCompute_tb
ghdl -r --std=08 --workdir="$W" AngleCompute_tb "$@"
