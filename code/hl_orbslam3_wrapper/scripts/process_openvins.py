import sys

file_i = "/tmp/traj_estimate.txt"
file_o = "/user/openvins_traj_estimate.tum"
if len(sys.argv) > 1:
    file_i = sys.argv[1]
if len(sys.argv) > 2:
    file_o = sys.argv[2]

print("Input file: ", file_i)
print("Output file: ", file_o)

with open(file_i, 'r') as f:
    lines = f.readlines()

with open(file_o, 'w') as f:
    for line in lines:
        if '#' in line:
            continue

        line = line.strip().split()
        line = [line.strip(',') for line in line]

        # Check if values are words
        try:
            timestamp = int(line[0])
            timestamp = float(timestamp / 1e9)  # convert to seconds
        except ValueError as e:
            try:
                timestamp = float(line[0])
            except ValueError as e:
                print("Skipping line: ", line)
                continue

        # print(line)
        x, y, z, qx, qy, qz, qw, *_ = [float(x) for x in line[1:]]

        f.write(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
