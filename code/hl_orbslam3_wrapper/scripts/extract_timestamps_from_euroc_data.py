import sys

assert len(sys.argv) == 3, "Usage: python extract_timestamps_from_euroc_data.py <euroc_data_file> <timestamp_file>"

with open(sys.argv[1], 'r') as f:
    in_lines = f.readlines()


with open(sys.argv[2], 'w') as out_f:
    for line in in_lines:
        if line[0] == '#':
            continue

        timestamp = int(float(line.split(',')[0]))
        print(timestamp)
        out_f.write(f"{timestamp}\n")
