'''
For some reason people may give traj files as timestamp, tx, ty, tz, qW, qx, qy, qz.

However, Evo specifies them as timestamp, tx, ty, tz, qX, qy, qz, qw

This script just does the conversion.
'''

from pathlib import Path
import sys
import os
import csv

assert len(sys.argv) > 1, f"Usage: {sys.argv[0]} <input_file> <optional_output_file>"

INPUT_FILE = sys.argv[1]
assert Path(INPUT_FILE).exists(), f"{INPUT_FILE} does not exist"

if len(sys.argv) > 2:
    OUT_FILE = sys.argv[2]
else:
    OUT_FILE = INPUT_FILE + ".tum"

print("Reading from ", INPUT_FILE, "Saving to", OUT_FILE)

with open(INPUT_FILE, 'r') as f:
    reader = csv.reader(f, delimiter=',', skipinitialspace=True)
    with open(OUT_FILE, 'w') as f_out:
        for line in reader:
            if '#' in line[0]:
                continue

            t, x, y, z, qw, qx, qy, qz = line[:8]
            t = float(t) / 1e9  # from ns to seconds
            f_out.write(f"{t} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
