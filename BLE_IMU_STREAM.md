# BLE IMU stream for hybrid fall detection

The watch keeps its on-device threshold detector and `FALL` notification. It
also sends IMU samples for optional fall inference in the companion app.

## GATT

- Service: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- Existing time-write characteristic: `6e400002-b5a3-f393-e0a9-e50e24dcca9e`
- Existing fall-alert characteristic: `6e400003-b5a3-f393-e0a9-e50e24dcca9e`
- IMU stream characteristic (notify/read): `6e400004-b5a3-f393-e0a9-e50e24dcca9e`

Subscribe to the IMU characteristic's notifications. The app should negotiate
an ATT MTU of at least 79 bytes; the watch requests MTU 100. If the negotiated
MTU is smaller, the maximum five-sample packet may not fit.

## Packet format

All multi-byte values are little-endian. Each notification contains:

| Offset | Size | Meaning |
|---:|---:|---|
| 0 | 1 | Protocol marker `0xA1` |
| 1 | 1 | Sample count, 1–5 |
| 2 | 4 | Timestamp in milliseconds for the first sample |
| 6 onward | 14 per sample | Timestamp delta (u16 ms), then ax, ay, az, gx, gy, gz (i16) |

Decode each sample as follows:

- Timestamp: first timestamp + sample delta (unsigned 32-bit millisecond wrap is possible).
- `ax`, `ay`, `az`: signed value / 1000, in g.
- `gx`, `gy`, `gz`: signed value / 100, in rad/s.
- Channel order is `ax, ay, az, gx, gy, gz`.

The maximum frame is 76 bytes and is normally sent every 50 ms, containing up
to five samples from the approximately 100 Hz IMU task. The model app should
use timestamps to detect missing samples and resample to its training rate
before inference.

## Hybrid behavior

The ESP32 continues to run the existing threshold detector and send `FALL`
after its local alert countdown. The app can independently classify streamed
samples. Do not treat either detector as a certified emergency or medical
system; verify fall and non-fall performance with representative real-world
data before relying on alerts.
