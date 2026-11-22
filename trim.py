import csv, glob, math, os

os.makedirs('trimmed', exist_ok=True)

for src in sorted(glob.glob('fall*.csv')):
    with open(src) as f:
        rows = list(csv.reader(f))[1:]
    data = [[float(x) for x in r[:7]] for r in rows if len(r) >= 7]
    if len(data) < 300:
        print('skip (too short):', src)
        continue
    svm = [math.sqrt(r[1]**2 + r[2]**2 + r[3]**2) for r in data]
    peak = max(range(len(svm)), key=lambda i: svm[i])
    start, end = peak - 100, peak + 200
    if start < 0 or end > len(data):
        print('skip (peak too close to the edge):', src)
        continue
    t0 = data[start][0]
    base = os.path.splitext(os.path.basename(src))[0].replace('.', '_')
    dst = os.path.join('trimmed', 'fall.' + base + '.csv')
    with open(dst, 'w') as f:
        print('timestamp,ax,ay,az,gx,gy,gz', file=f)
        for r in data[start:end]:
            print(f'{r[0]-t0:.0f},{r[1]:.3f},{r[2]:.3f},{r[3]:.3f},{r[4]:.2f},{r[5]:.2f},{r[6]:.2f}', file=f)
    print(dst, 'peak', round(svm[peak], 2), 'g')