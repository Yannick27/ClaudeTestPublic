#!/usr/bin/env python3
"""Golden model + vector generator/checker for AngleCompute.

  angle_model.py gen   --order 6 --count 500 --seed 1 --out vectors.txt
  angle_model.py check --order 6 --vectors vectors.txt --results results.txt

The model uses exact rational arithmetic for the normalisation, the Chebyshev
recurrence and the linear combination, then floating point atan2 for the angle.
The expected angle is the *ideal* one (no fixed-point rounding at all), so the
reported error is the true error of the hardware, in LSBs of the 16 bit output.
"""
import argparse
import math
import random
import sys
from fractions import Fraction

PMIN, PMAX = 1_000_000, 2_000_000
I32MAX = 2**31 - 1


# --------------------------------------------------------------------------
# model
# --------------------------------------------------------------------------
def cheb_lin(order, period, gain, offset, coeffs):
    """sum_k T_k(X) * C_k, exact. gain is Q-4.36, offset is Q8.24."""
    x = Fraction(period * gain, 2**36) + Fraction(offset, 2**24)
    assert -1 <= x <= 1, f"normalised period out of range: {float(x)}"
    t_prev, t_cur = Fraction(1), x
    acc = Fraction(0)
    for k in range(1, order + 1):
        acc += t_cur * coeffs[k - 1]
        t_prev, t_cur = t_cur, 2 * x * t_cur - t_prev
    return acc


def expected_angle(order, v):
    lin = [cheb_lin(order, v["P"][i], v["G"][i], v["O"][i], v["C"][i]) for i in range(4)]
    dx = lin[0] - lin[1] + v["GX"]
    dz = lin[2] - lin[3] + v["GZ"]
    num = v["CZ"] * dx - v["SX"] * dz
    den = v["SZ"] * dx + v["CX"] * dz
    ang = math.atan2(float(num), float(den)) / (2 * math.pi) * 65536
    return ang % 65536, float(num), float(den)


# --------------------------------------------------------------------------
# vector file
# --------------------------------------------------------------------------
def fmt(order, v):
    f = []
    f += v["P"] + v["G"] + v["O"]
    for ch in v["C"]:
        f += ch
    f += [v["GX"], v["GZ"], v["CX"], v["SX"], v["CZ"], v["SZ"]]
    f += v["D"] + [v["mode"]]
    return " ".join(str(int(x)) for x in f)


def parse(order, line):
    n = [int(t) for t in line.split()]
    v = {"P": n[0:4], "G": n[4:8], "O": n[8:12]}
    o = 12
    v["C"] = []
    for _ in range(4):
        v["C"].append(n[o:o + order])
        o += order
    v["GX"], v["GZ"], v["CX"], v["SX"], v["CZ"], v["SZ"] = n[o:o + 6]
    o += 6
    v["D"] = n[o:o + 4]
    v["mode"] = n[o + 4]
    return v


# --------------------------------------------------------------------------
# generator
# --------------------------------------------------------------------------
def channel(rng, p_mode, exact=None):
    """gain / offset like the real calibration + a period in [PMIN, PMAX]."""
    umax = rng.uniform(0.85, 0.999999)
    gain = int(2 / (PMAX - PMIN) * umax * 2**36)
    offset = round(-(PMIN + PMAX) / 2 * gain / 2**12)
    if p_mode == "min":
        p = PMIN
    elif p_mode == "max":
        p = PMAX
    elif p_mode == "mid":
        p = (PMIN + PMAX) // 2
    else:
        p = rng.randint(PMIN, PMAX)
    if exact is not None:
        # gain = k*4096 so that P*G is exactly representable in Q8.24: X = +-1 exactly
        k = rng.randint(1, 30)
        gain = k * 4096
        p = rng.randint(PMIN, PMAX)
        offset = (2**24 if exact > 0 else -2**24) - p * k
    return p, gain, offset


def coeffs(rng, order, style):
    if style == "decay":     # looks like a real linearisation: big C1, decaying higher orders
        s = 2 ** rng.randint(24, 30)
        return [int(rng.uniform(-1, 1) * s / (4 ** (k))) for k in range(order)]
    if style == "full":
        return [rng.randint(-I32MAX, I32MAX) for _ in range(order)]
    if style == "small":
        return [rng.randint(-2**22, 2**22) for _ in range(order)]
    raise ValueError(style)


def trig(rng, scale):
    tx, tz = rng.uniform(-math.pi, math.pi), rng.uniform(-math.pi, math.pi)
    if rng.random() < 0.5:  # typical: small misalignment
        tx, tz = rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2)
    return (round(math.cos(tx) * scale), round(math.sin(tx) * scale),
            round(math.cos(tz) * scale), round(math.sin(tz) * scale))


def delays(rng):
    mode = rng.randint(0, 1)
    lo = 6 if mode else 0
    return [rng.choice([lo, lo + 1, rng.randint(lo, 120), rng.randint(lo, 400)]) for _ in range(4)], mode


def random_vector(rng, order):
    style = rng.choice(["decay", "decay", "full", "small"])
    pm = rng.choice(["rand", "rand", "rand", "min", "max", "mid"])
    ch = [channel(rng, pm if rng.random() < 0.5 else "rand") for _ in range(4)]
    cx, sx, cz, sz = trig(rng, rng.choice([I32MAX, 2**30, 2**28, 2**20]))
    d, mode = delays(rng)
    return {
        "P": [c[0] for c in ch], "G": [c[1] for c in ch], "O": [c[2] for c in ch],
        "C": [coeffs(rng, order, style) for _ in range(4)],
        "GX": rng.randint(-I32MAX, I32MAX) if style != "small" else rng.randint(-2**22, 2**22),
        "GZ": rng.randint(-I32MAX, I32MAX) if style != "small" else rng.randint(-2**22, 2**22),
        "CX": cx, "SX": sx, "CZ": cz, "SZ": sz, "D": d, "mode": mode,
    }


def base_vector(rng, order):
    ch = [channel(rng, "mid") for _ in range(4)]
    return {
        "P": [c[0] for c in ch], "G": [c[1] for c in ch], "O": [c[2] for c in ch],
        "C": [[0] * order for _ in range(4)], "GX": 0, "GZ": 0,
        "CX": I32MAX, "SX": 0, "CZ": I32MAX, "SZ": 0, "D": [0, 0, 0, 0], "mode": 0,
    }


def directed(rng, order):
    out = []
    # 1. every octant, axis and several magnitudes: zero coefficients, identity
    #    trig -> angle = atan2(GammaX, GammaZ)
    for mag in (2**31 - 1, 2**20, 12345):
        for a in range(0, 360, 15):
            v = base_vector(rng, order)
            v["GX"] = round(math.sin(math.radians(a)) * mag)
            v["GZ"] = round(math.cos(math.radians(a)) * mag)
            out.append(v)
    for gx, gz in ((0, 1_000_000), (1_000_000, 0), (0, -1_000_000), (-1_000_000, 0),
                   (I32MAX, I32MAX), (-I32MAX, -I32MAX), (I32MAX, -I32MAX), (-I32MAX, I32MAX), (1, 1)):
        v = base_vector(rng, order)
        v["GX"], v["GZ"] = gx, gz
        out.append(v)
    # 2. normalised period exactly +1 / -1 with every Chebyshev term at full scale
    #    and P / N channels of opposite polarity: LinP = +-ORDER*(2^31-1),
    #    LinN = -+ORDER*(2^31-1), Gamma = +-(2^31-1)  ->  |Delta| = (2*ORDER+1)*(2^31-1),
    #    the worst case the datapath widths are sized for. X and Z polarities are
    #    varied independently, and several (non-singular) sin/cos sets are used.
    a30, a50 = math.radians(30), math.radians(-50)
    trig_sets = (
        (I32MAX, -I32MAX, -I32MAX, I32MAX),
        (I32MAX, I32MAX, I32MAX, I32MAX),
        (-I32MAX, I32MAX, I32MAX, -I32MAX),
        (round(math.cos(a30) * I32MAX), round(math.sin(a30) * I32MAX),
         round(math.cos(a50) * I32MAX), round(math.sin(a50) * I32MAX)),
    )
    for xs in ((1, 1, 1, 1), (-1, -1, -1, -1), (1, -1, 1, -1), (-1, 1, -1, 1)):
        for polx, polz in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            for trig_set in trig_sets:
                v = base_vector(rng, order)
                for i in range(4):
                    p, g, o = channel(rng, "rand", exact=xs[i])
                    v["P"][i], v["G"][i], v["O"][i] = p, g, o
                    pol = polx if i < 2 else polz            # channels 1,2 = X ; 3,4 = Z
                    side = 1 if i in (0, 2) else -1          # channels 1,3 = P ; 2,4 = N
                    # sign of T_k(x) at x = +-1 is +1 for x = +1, (-1)^k for x = -1
                    v["C"][i] = [pol * side * (1 if (xs[i] > 0 or k % 2 == 0) else -1) * I32MAX
                                 for k in range(1, order + 1)]
                v["GX"], v["GZ"] = polx * I32MAX, polz * I32MAX
                v["CX"], v["SX"], v["CZ"], v["SZ"] = trig_set
                out.append(v)
    # 3. most negative values
    v = base_vector(rng, order)
    v["C"] = [[-2**31] * order for _ in range(4)]
    v["GX"] = v["GZ"] = -2**31
    v["CX"], v["SX"], v["CZ"], v["SZ"] = -2**31, -2**31, -2**31, -2**31
    out.append(v)
    return out


def cmd_gen(a):
    rng = random.Random(a.seed)
    vecs = directed(rng, a.order) + [random_vector(rng, a.order) for _ in range(a.count)]
    for n, v in enumerate(vecs):
        _, num, den = expected_angle(a.order, v)
        if num == 0 and den == 0:
            raise SystemExit(f"vector {n}: zero result vector, angle undefined")
    with open(a.out, "w") as f:
        for v in vecs:
            f.write(fmt(a.order, v) + "\n")
    print(f"{len(vecs)} vectors -> {a.out}")


def cmd_check(a):
    vecs = [parse(a.order, l) for l in open(a.vectors) if l.strip() and not l.startswith("#")]
    res = [tuple(int(t) for t in l.split()) for l in open(a.results) if l.strip()]
    if len(vecs) != len(res):
        print(f"FAIL: {len(vecs)} vectors but {len(res)} results")
        return 1
    worst, bad, cycles = 0.0, 0, []
    for n, (v, (ang, cyc)) in enumerate(zip(vecs, res)):
        exp, num, den = expected_angle(a.order, v)
        err = (ang - exp + 32768) % 65536 - 32768
        cycles.append(cyc)
        mag = math.hypot(num, den)
        if abs(err) > worst:
            worst = abs(err)
        if abs(err) > a.tol:
            bad += 1
            print(f"  vector {n}: dut={ang} expected={exp:.3f} err={err:+.3f} LSB |v|={mag:.3g}")
    print(f"{len(vecs)} vectors, max |error| = {worst:.3f} LSB (tolerance {a.tol}), "
          f"{bad} outside tolerance, cycles/result min={min(cycles)} max={max(cycles)}")
    print("PASS" if bad == 0 else "FAIL")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("gen")
    g.add_argument("--order", type=int, default=6)
    g.add_argument("--count", type=int, default=500)
    g.add_argument("--seed", type=int, default=1)
    g.add_argument("--out", default="vectors.txt")
    c = sub.add_parser("check")
    c.add_argument("--order", type=int, default=6)
    c.add_argument("--vectors", default="vectors.txt")
    c.add_argument("--results", default="results.txt")
    c.add_argument("--tol", type=float, default=0.75)
    a = ap.parse_args()
    sys.exit(cmd_gen(a) if a.cmd == "gen" else cmd_check(a))
