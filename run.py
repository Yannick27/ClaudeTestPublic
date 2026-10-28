#!/usr/bin/env python3
"""VUnit run script of the AngleCompute testbench.

Usage:
    python run.py [vunit options]          e.g.  python run.py -p 0 -v

The DUT sources (AngleCompute.vhd, PeriodNormLin.vhd, Mult.vhd, BamPkg.vhd,
Crc.vhd) are taken from ./src, or from the directory given by the environment
variable ANGLECOMPUTE_SRC. The random seed can be set with SEED (default 1).
"""

import os
from pathlib import Path

from vunit import VUnit

ROOT = Path(__file__).resolve().parent
SRC = Path(os.environ.get("ANGLECOMPUTE_SRC", ROOT / "src"))

try:
    vu = VUnit.from_argv(compile_builtins=False)
    vu.add_vhdl_builtins()
except (TypeError, AttributeError):  # VUnit < 5
    vu = VUnit.from_argv()
vu.add_osvvm()

crc = vu.add_library("crc")
crc.add_source_files(SRC / "Crc.vhd")

lib = vu.add_library("lib")
lib.add_source_files(
    [
        SRC / "BamPkg.vhd",
        SRC / "Mult.vhd",
        SRC / "PeriodNormLin.vhd",
        SRC / "AngleCompute.vhd",
    ]
)
lib.add_source_files(ROOT / "tb" / "AngleCompute_tb.vhd")

lib.entity("AngleCompute_tb").set_generic("G_SEED", int(os.environ.get("SEED", "1")))

vu.main()
