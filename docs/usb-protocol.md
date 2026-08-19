# TinyKnob USB Protocol

Host software can be written from this document alone. Firmware implements exactly these layouts.

## Device identity

| Field | Value |
|-------|-------|
| VID | `0xACDC` |
| PID | `0x4011` |
| USB | Full Speed 2.0 |
| Manufacturer | `Volwave` |
| Product | `VBTT Device` |

PID bits: `0x4000 | CDC | VENDOR` (HID removed).

## Interfaces

Configuration 1 has two functional interfaces:

1. **CDC ACM** (IAD) — human-readable logs and optional text debug commands. Not required for a binary host.
2. **Vendor** (class `0xFF`) — binary control + telemetry over Bulk endpoints.

### Vendor endpoints (fixed)

| Direction | Address | Type | `wMaxPacketSize` |
|-----------|---------|------|------------------|
| OUT (host → device) | `0x01` | Bulk | 64 |
| IN (device → host) | `0x81` | Bulk | 64 |

Interface string: `VBTT Vendor` (index 5).

All multi-byte fields are **little-endian**.

## Enumeration (host)

1. Open device `VID=0xACDC`, `PID=0x4011`.
2. Claim the **Vendor** interface (the non-CDC interface; typically interface index 2 after CDC control+data).
3. Submit continuous Bulk IN transfers on `0x81` (or use a read loop).
4. Send commands on Bulk OUT `0x01`.

On Linux with libusb, prefer matching by VID/PID then selecting the interface with `bInterfaceClass == 0xFF`.

## Telemetry stream (device → host, Bulk IN)

The device pushes fixed 16-byte frames at about **1 kHz** when the host keeps Bulk IN outstanding and the bus is free. Do not assume a hard realtime guarantee.

### Frame layout

| Offset | Type | Name | Meaning |
|--------|------|------|---------|
| 0 | `u8` | `magic` | Always `0xA5` |
| 1 | `u8` | `mode` | Motor mode (see table below) |
| 2–5 | `i32` | `angle_mrad` | Mechanical angle in milliradians |
| 6–7 | `i16` | `duty_a` | Phase A PWM duty, Q15 |
| 8–9 | `i16` | `duty_b` | Phase B PWM duty, Q15 |
| 10–11 | `i16` | `duty_c` | Phase C PWM duty, Q15 |
| 12–13 | `u16` | `seq` | Monotonic counter (wraps) |
| 14–15 | `u16` | `reserved` | `0` |

Total size: **16 bytes**.

### Units

- **Physical angle**: `angle_mrad = round(pos * (2π / 16384) * 1000)` where `pos` is the unwrapped MT6701 count (14-bit sensor, unwrapped in firmware). Positive direction follows encoder increase. Not wrapped to ±π; it grows with turns.
- **Duty Q15**: duty cycle in `[0, 1]` mapped as `q15 = (int16_t)(duty * 32767)`. Midpoint `0.5` ≈ `16383` is zero line-to-line voltage (idle, deadzone, and `U≈0`). Active drive swings around this midpoint.

### Mode values (`mode`)

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `MOTOR_IDLE` | Armed idle / brake |
| 1 | `MOTOR_ALIGN_RAMP` | Align ramp |
| 2 | `MOTOR_ALIGN_HOLD` | Align hold |
| 3 | `MOTOR_DIR_PULSE` | Direction sense pulse |
| 4 | `MOTOR_ALIGN_DOWN` | Align ramp down |
| 5 | `MOTOR_TEST` | ±360° test move |
| 6 | `MOTOR_SPRING` | Virtual spring |
| 7 | `MOTOR_SPIN` | Voltage-mode flywheel |
| 8 | `MOTOR_FAULT` | Fault (CRC etc.); brake |

## Command frames (host → device, Bulk OUT)

Each command is one Bulk OUT transfer. Byte 0 is the opcode; payload follows immediately. Extra trailing bytes are ignored.

| `cmd` | Name | Payload | Effect |
|-------|------|---------|--------|
| `0x01` | `START` | none | Run align, then idle (armed) |
| `0x02` | `STOP` | none | Brake / idle; keep align |
| `0x03` | `SPRING` | none | Enter spring (needs prior align) |
| `0x04` | `SPIN` | none | Enter spin (needs prior align) |
| `0x05` | `TEST` | none | Enter TEST (needs prior align) |
| `0x20` | `SET_K` | `u8 k_x10` | Spring stiffness `K = k_x10 / 10` (clamped 0…8) |
| `0x21` | `SET_REST` | none | Set spring rest angle to current position |

### Short ACK (optional)

After a command, the device may write a 3-byte packet on Bulk IN:

| Offset | Type | Meaning |
|--------|------|---------|
| 0 | `u8` | magic `0x5A` |
| 1 | `u8` | echoed `cmd` |
| 2 | `u8` | `status`: `1` = ok, `0` = rejected (e.g. SPRING/SPIN/TEST before align) |

Hosts **may ignore** ACKs. Distinguish from telemetry by `magic`: `0xA5` = telem, `0x5A` = ack. If a read returns a length other than 16, or magic is `0x5A`, treat as ack (or skip).

### Failure semantics

- `START` / `STOP` / `SET_K` / `SET_REST` always succeed at the protocol layer (`status=1`).
- `SPRING` / `SPIN` / `TEST` return `status=0` if the motor is not aligned/armed yet (same as CDC `need START`).
- Unknown `cmd`: no ACK; ignored.
- Empty OUT transfer: ignored.

## CDC side channel (debug only)

CDC remains for logs. Text lines ending in `\n` or `\r` still accept: `START`, `STOP`, `SPRING`, `SPIN`, `TEST`, `DUMP`, `UPLOAD`. Binary hosts should use Vendor Bulk only.

## Host sketch (Python + pyusb)

```python
import struct
import usb.core
import usb.util

VID, PID = 0xACDC, 0x4011
EP_OUT, EP_IN = 0x01, 0x81

dev = usb.core.find(idVendor=VID, idProduct=PID)
assert dev is not None
dev.set_configuration()
cfg = dev.get_active_configuration()
intf = usb.util.find_descriptor(cfg, bInterfaceClass=0xFF)
usb.util.claim_interface(dev, intf.bInterfaceNumber)

def send_cmd(cmd: int, *payload: int) -> None:
    dev.write(EP_OUT, bytes([cmd, *payload]), timeout=1000)

def read_telem():
    data = bytes(dev.read(EP_IN, 64, timeout=1000))
    if len(data) >= 3 and data[0] == 0x5A:
        return ("ack", data[1], data[2])  # cmd, status
    if len(data) < 16 or data[0] != 0xA5:
        return None
    magic, mode, angle_mrad, da, db, dc, seq, _ = struct.unpack_from("<BBi3hHH", data, 0)
    return {
        "mode": mode,
        "angle_mrad": angle_mrad,
        "duty": (da / 32767.0, db / 32767.0, dc / 32767.0),
        "seq": seq,
    }

send_cmd(0x01)           # START
send_cmd(0x03)           # SPRING
send_cmd(0x20, 25)       # SET_K → 2.5
while True:
    frame = read_telem()
    if frame and frame != ("ack",) and isinstance(frame, dict):
        print(frame)
```

## C struct (device / host)

```c
#define TELEM_MAGIC 0xA5
#define ACK_MAGIC   0x5A

typedef struct __attribute__((packed)) {
	uint8_t  magic;       /* 0xA5 */
	uint8_t  mode;
	int32_t  angle_mrad;
	int16_t  duty_a;      /* Q15 */
	int16_t  duty_b;
	int16_t  duty_c;
	uint16_t seq;
	uint16_t reserved;
} tinyknob_telem_t; /* 16 bytes */

typedef struct __attribute__((packed)) {
	uint8_t magic;  /* 0x5A */
	uint8_t cmd;
	uint8_t status;
} tinyknob_ack_t; /* 3 bytes */
```

## Command opcodes (summary)

```c
enum {
	TK_CMD_START    = 0x01,
	TK_CMD_STOP     = 0x02,
	TK_CMD_SPRING   = 0x03,
	TK_CMD_SPIN     = 0x04,
	TK_CMD_TEST     = 0x05,
	TK_CMD_SET_K    = 0x20,
	TK_CMD_SET_REST = 0x21,
};
```
