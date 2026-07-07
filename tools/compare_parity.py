#!/usr/bin/env python3
"""Compare the Python and C++ headless CSV outputs column by column."""
import sys
import numpy as np

a_path = sys.argv[1] if len(sys.argv) > 1 else "parity_python.csv"
b_path = sys.argv[2] if len(sys.argv) > 2 else "parity_cpp.csv"
a = np.genfromtxt(a_path, delimiter=",", names=True)
b = np.genfromtxt(b_path, delimiter=",", names=True)
assert a.shape == b.shape, f"row count differs: {a.shape} vs {b.shape}"
print(f"{'column':>6s}  {'max |diff|':>12s}")
worst = 0.0
for name in a.dtype.names:
    d = float(np.max(np.abs(a[name] - b[name])))
    worst = max(worst, d)
    print(f"{name:>6s}  {d:12.3e}")
print(f"\nworst column difference: {worst:.3e}")
print("PARITY OK" if worst < 1e-6 else "PARITY MARGINAL — inspect columns above")
