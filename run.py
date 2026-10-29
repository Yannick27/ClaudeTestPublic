"""VUnit run script for AngleCompute.

Usage : python run.py [VUnit options]   (e.g. python run.py -p 0 -v)
The random seed of the testbench can be set with the SEED environment variable.
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
if "SEED" in os.environ:
    tb.set_generic("SEED", int(os.environ["SEED"]))

vu.main()
