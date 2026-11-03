"""VUnit run script for AngleCompute.

Usage : python run.py [VUnit options]   (e.g. python run.py -p 8 -v)
Each test case is run NUM_SEEDS times, each run with a different random seed
drawn at every invocation of this script. The seed is part of the test name
(e.g. lib.AngleCompute_tb.seed_1234567.random_computations), so a failing run
can be reproduced by setting SEED to that value in the testbench generic.
"""
import random
from pathlib import Path

from vunit import VUnit

ROOT = Path(__file__).parent
NUM_SEEDS = 10

vu = VUnit.from_argv(compile_builtins=False)
vu.add_vhdl_builtins()
vu.add_osvvm()

lib = vu.add_library("lib")
lib.add_source_files(ROOT / "src" / "*.vhd")
lib.add_source_files(ROOT / "tb" / "*.vhd")

# SEED is a VHDL natural : 0 to 2**31 - 1
seeds = random.SystemRandom().sample(range(1, 2**31), NUM_SEEDS)

tb = lib.test_bench("AngleCompute_tb")
for seed in seeds:
    tb.add_config(name=f"seed_{seed}", generics={"SEED": seed})

vu.main()
