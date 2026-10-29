#!/usr/bin/env python3
"""VUnit run script for AngleCompute.

Usage:  python run.py [vunit options]          e.g.  python run.py -p 4
        ANGLE_TB_SEED=1234 python run.py       to change the random seed
"""
import os
from pathlib import Path

from vunit import VUnit

ROOT = Path(__file__).parent

vu = VUnit.from_argv(compile_builtins=False)
vu.add_vhdl_builtins()
vu.add_osvvm()

lib = vu.add_library("lib")
lib.add_source_files(ROOT / "src" / "*.vhd")
lib.add_source_files(ROOT / "tb" / "*.vhd")

tb = lib.test_bench("AngleCompute_tb")
tb.set_generic("SEED", int(os.environ.get("ANGLE_TB_SEED", "1")))

vu.main()
