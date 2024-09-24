'''
The VINS-MONO output is a weird form of TUM data. Needs to be preprocessed to be read in EVO format.

Simply edit the VINS_MONO_DATA path, and specify the files to be processed.
'''

import os
from pathlib import Path
import csv

VINS_MONO_DATA = Path(
    "/user/vins_mono_out")
assert VINS_MONO_DATA.exists()


def correct_vins_out(file: Path):
    out_file = file.parent / (file.stem + '_processed' + file.suffix)
    print("Saving to", out_file)

    with open(file, 'r') as f:
        reader = csv.reader(f, delimiter=',', skipinitialspace=True)
        with open(out_file, 'w') as f_out:
            for line in reader:
                t, x, y, z, qx, qy, qz, qw = line[:8]
                t = float(t) / 1e9
                f_out.write(f"{t} {x} {y} {z} {qx} {qy} {qz} {qw}\n")


for file in ['vins_result_loop.csv', 'vins_result_no_loop.csv']:
    try:
        correct_vins_out(VINS_MONO_DATA / file)
    except FileNotFoundError:
        print(f"{VINS_MONO_DATA/file} not found")
        continue
