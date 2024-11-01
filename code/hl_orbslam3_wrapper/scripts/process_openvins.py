file_i = "/tmp/traj_estimate.txt"
file_o = "/user/openvins_traj_estimate.tum"

with open(file_i, 'r') as f:
    lines = f.readlines()

with open(file_o, 'w') as f:
    for line in lines:
        if '#' in line:
            continue

        line = line.strip().split()

        f.write(f"{line[0]} {line[1]} {line[2]} {line[3]} {line[4]} {line[5]} {line[6]} {line[7]}\n")
