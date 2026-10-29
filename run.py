#!/usr/bin/env python3
"""VUnit run script for AngleCompute.

Usage examples:
    python3 run.py                       # run all tests
    python3 run.py "*test_random*" -v    # run one test, verbose
    SEED=42 python3 run.py               # change the OSVVM random seed
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
tb.set_generic("SEED", int(os.environ.get("SEED", "1")))

vu.main()
