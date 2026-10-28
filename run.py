#!/usr/bin/env python3
"""VUnit run script for AngleCompute.

Usage:   python3 run.py            (all tests, all ORDER configurations)
         python3 run.py -l         (list tests)
         python3 run.py --gtkwave-fmt ghw -g ...   (see VUnit docs)

Needs: pip install vunit_hdl, and a VHDL-2008 simulator (GHDL, Questa, ...).
OSVVM is the copy bundled with VUnit.
"""
from pathlib import Path

from vunit import VUnit

ROOT = Path(__file__).parent

vu = VUnit.from_argv(vhdl_standard="2008")
vu.add_vhdl_builtins()
vu.add_osvvm()

lib = vu.add_library("lib")
lib.add_source_files(ROOT / "src" / "*.vhd")
lib.add_source_files(ROOT / "tb" / "*.vhd")

# Run the whole test suite for several Chebyshev orders
tb = lib.test_bench("tb_AngleCompute")
for order in (1, 2, 6, 9):
    tb.add_config(name=f"ORDER_{order}", generics=dict(ORDER=order))

vu.main()
