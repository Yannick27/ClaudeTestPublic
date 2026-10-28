#!/usr/bin/env python3
"""VUnit run script of AngleCompute_tb.

Usage: python run.py [VUnit options]
  SRC_DIR : directory of BamPkg.vhd, Mult.vhd, PeriodNormLin.vhd, AngleCompute.vhd
            (default ../src)
  CRC_DIR : directory of the crc library sources (CRCPkg, used by BamPkg)
            (default ../crc)
"""
import os
from pathlib import Path

from vunit import VUnit

ROOT = Path(__file__).resolve().parent
SRC_DIR = Path(os.environ.get("SRC_DIR", ROOT / ".." / "src"))
CRC_DIR = Path(os.environ.get("CRC_DIR", ROOT / ".." / "crc"))

vu = VUnit.from_argv(compile_builtins=False)
vu.add_vhdl_builtins()

crc = vu.add_library("crc")
crc.add_source_files(CRC_DIR / "*.vhd")

lib = vu.add_library("lib")
for name in ("BamPkg.vhd", "Mult.vhd", "PeriodNormLin.vhd", "AngleCompute.vhd"):
    lib.add_source_file(SRC_DIR / name)
lib.add_source_file(ROOT / "AngleCompute_tb.vhd")

# second seed for the random test
tb = lib.test_bench("AngleCompute_tb")
test = tb.test("random_parameters")
test.add_config(name="seed1", generics=dict(SEED=1))
test.add_config(name="seed2", generics=dict(SEED=20260407, N_RANDOM=500))

vu.main()
