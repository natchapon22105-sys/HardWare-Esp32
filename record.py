import sys, time, serial

if len(sys.argv) < 4:
    print('usage: python record.py PORT LABEL SECONDS')
    sys.exit(1)

port, label, seconds = sys.argv[1], sys.argv[2], float(sys.argv[3])
ser = serial.Serial(port, 115200, timeout=1)
fname = label + '.' + str(int(time.time())) + '.csv'
rows = 0
t_end = time.time() + seconds
print('recording', fname)

with open(fname, 'w') as f:
    print('timestamp,ax,ay,az,gx,gy,gz', file=f)
    while time.time() < t_end:
        line = ser.readline().decode(errors='ignore').strip()
        parts = line.split(',')
        if len(parts) != 7:
            continue
        try:
            [float(p) for p in parts]
        except ValueError:
            continue
        print(line, file=f)
        rows += 1

print('saved', rows, 'rows to', fname)