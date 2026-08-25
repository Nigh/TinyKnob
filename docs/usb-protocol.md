# TinyKnob USB Protocol

Host software can be written from this document alone. Firmware implements exactly these layouts.

## Device identity

| Field | Value |
|-------|-------|
| VID | `0xACDC` |
| PID | `0x4011` |
| USB | Full Speed 2.0 |
| Manufacturer | `HelloWorks` |
| Product | `TinyRoller` |

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

Interface string: `TinyRoller Vendor` (index 5).

All multi-byte fields are **little-endian**.

## Enumeration (host)

1. Open device `VID=0xACDC`, `PID=0x4011`.
2. Claim the **Vendor** interface (the non-CDC interface; typically interface index 2 after CDC control+data).
3. Submit continuous Bulk IN transfers on `0x81` (or use a read loop).
4. Send commands on Bulk OUT `0x01`.

On Linux with libusb, prefer matching by VID/PID then selecting the interface with `bInterfaceClass == 0xFF`.

## Telemetry stream (device → host, Bulk IN)

The device pushes fixed **25-byte** frames at about **1 kHz** when the host keeps Bulk IN outstanding and the bus is free. Do not assume a hard realtime guarantee.

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
| 14–15 | `i16` | `id_mA` | Measured Id (milliamps) |
| 16–17 | `i16` | `iq_mA` | Measured Iq (milliamps) |
| 18–19 | `i16` | `iq_ref_mA` | Iq reference (milliamps) |
| 20–21 | `u16` | `vbus_mV` | Bus voltage from 0.1×VCC sense (millivolts) |
| 22–23 | `i16` | `uq_q15` | Current-loop Uq output (normalized, Q15; 32767 ≈ full) |
| 24 | `i8` | `dir` | Motor/encoder direction (`-1` or `+1`); use it to normalize `iq_ref_mA` to the pre-direction command domain |

Total size: **25 bytes**. The first 24 bytes remain backward-compatible. Bytes 0–13 match the previous 16-byte frame prefix (reserved was replaced).

### Units

- **Physical angle**: `angle_mrad = round(pos * (2π / 16384) * 1000)` where `pos` is the unwrapped MT6701 count (14-bit sensor, unwrapped in firmware). Positive direction follows encoder increase. Not wrapped to ±π; it grows with turns.
- **Duty Q15**: duty cycle in `[0, 1]` mapped as `q15 = (int16_t)(duty * 32767)`. Midpoint `0.5` ≈ `16383` is zero line-to-line voltage (idle, deadzone, and `U≈0`). Active drive swings around this midpoint.
- **Currents**: milliamps, little-endian `i16`. With `CUR_LOOP_EN=0`, Id/Iq stay near 0. With `CUR_LOOP_EN=1` and `CUR_LOOP_CTRL=0` (sense-only), Id/Iq update while outer loops stay voltage-mode; `iq_ref_mA` is still the would-be current command for comparison.
- **Vbus**: millivolts from the 0.1× divider (`Vbus = Vad / 0.1`).
- **Uq Q15**: last current-loop voltage command on q-axis (`uq_out * 32767`), useful to see PI wind-up while debugging shake.

### Mode values (`mode`)

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `MOTOR_IDLE` | Armed idle / brake |
| 1 | `MOTOR_ALIGN_RAMP` | Align ramp |
| 2 | `MOTOR_ALIGN_HOLD` | Align hold |
| 3 | `MOTOR_DIR_PULSE` | Direction sense pulse |
| 4 | `MOTOR_ALIGN_DOWN` | Align ramp down |
| 5 | `MOTOR_TEST` | ±360° test move |
| 6 | `MOTOR_SPRING` | Virtual spring inside ±π; hard wall beyond ±π (no wrap-through) |
| 7 | `MOTOR_SPIN` | Low-damping voltage feed-forward and cog compensation; current PI for capped overspeed braking |
| 8 | `MOTOR_FAULT` | Fault (CRC etc.); brake |
| 9 | `MOTOR_POS` | Track absolute angle (streaming setpoint) |
| 10 | `MOTOR_STRESS` | Burn-in: +full / stop / −full / stop with smooth Uq ramps |
| 11 | `MOTOR_COG_CAL` | Slow +1 mech rev; learn cogging LUT then IDLE |
| 12 | `MOTOR_GEAR` | 24 detents aligned to the dominant physical cog harmonic |

## Command frames (host → device, Bulk OUT)

Each command is one Bulk OUT transfer. Byte 0 is the opcode; payload follows immediately. Extra trailing bytes are ignored.

| `cmd` | Name | Payload | Effect |
|-------|------|---------|--------|
| `0x01` | `START` | none | Run align, then idle (armed). **Not** automatic at boot — device powers up IDLE until START. |
| `0x02` | `STOP` | none | Brake / idle; keep align |
| `0x03` | `SPRING` | none | Enter spring (needs prior align) |
| `0x04` | `SPIN` | none | Enter spin (needs prior align) |
| `0x05` | `TEST` | none | Enter TEST (needs prior align) |
| `0x06` | `GOTO` | `i32 angle_mrad` (LE), optional `i32 velocity_mrad_s` (LE) | Enter/stay in `MOTOR_POS` and set tracking target (needs prior align) |
| `0x07` | `STRESS` | none | Enter burn-in loop (needs prior align): +full 3s, stop 1s, −full 3s, stop 1s; 500ms smoothstep Uq on start/stop |
| `0x08` | `COG_CAL` | none | Slow +1 rev position track; fill cogging FF LUT (needs prior align). ~`COG_CAL_MS` then IDLE with FF on |
| `0x09` | `COG_CLEAR` | none | Disable cogging FF and zero the LUT |
| `0x0A` | `GEAR` | none | Enter 24-tooth cog-aligned tactile gear mode (needs prior align) |
| `0x20` | `SET_K` | `u8 k_x10` | Spring stiffness `K = k_x10 / 10` (clamped 0…8) |
| `0x21` | `SET_REST` | none | Set spring rest angle to current position |
| `0x22` | `SET_COG_SCALE` | `u16 scale_x1000` (LE) | Set the current SPRING or SPIN scale to `scale_x1000 / 1000`, clamped to 0…2; RAM only |
| `0x7F` | `UPLOAD` | none | Reboot into UF2 bootloader (same as CDC `UPLOAD`) |

### GOTO / tracking details

- The required position uses the same unit as telemetry `angle_mrad` (unwrapped mechanical milliradians). An optional velocity field enables target-velocity feed-forward; omitted means zero for backward compatibility.
- Example: `+π` rad ≈ `3142` mrad → bytes `06 4E 0C 00 00` (`0x06` + LE `0x00000C4E`).
- **Tracking mode** (not a timed trajectory): each PWM tick (~20 kHz) applies P+D toward the latest target. Safe to stream at **hundreds of Hz to ~1 kHz** from the host to follow a simulated wheel.
- First accepted `GOTO` enters `MOTOR_POS`; later `GOTO`s only update the setpoint (no mode restart).
- Exit with `STOP` or another mode command (`SPRING` / `SPIN` / `GEAR` / `TEST` / `STRESS` / `COG_CAL` / `START`).
- Successful `GOTO` does **not** emit an ACK (keeps Bulk IN free for telemetry). Rejected `GOTO` still ACKs with `status=0`. Watch telem `mode == 9` to confirm tracking.

### Short ACK (optional)

After a command, the device may write a 3-byte packet on Bulk IN:

| Offset | Type | Meaning |
|--------|------|---------|
| 0 | `u8` | magic `0x5A` |
| 1 | `u8` | echoed `cmd` |
| 2 | `u8` | `status`: `1` = ok, `0` = rejected (e.g. SPRING/SPIN/TEST/GOTO before align, or `GOTO` with short payload) |

Hosts **may ignore** ACKs. A mode-scale event uses magic `0x5B` and must not be parsed as telemetry. Distinguish from telemetry by `magic`: `0xA5` = telem, `0x5A` = ack. Use the magic byte to distinguish packets; telemetry is currently 25 bytes and its original 24-byte prefix is stable.

Exception: successful `GOTO` (`0x06`, `status` would be 1) sends **no** ACK. `UPLOAD` (`0x7F`) also sends **no** ACK — the device reboots into bootrom before a reply can go out.

### Mode COG-scale event

On USB mount and whenever the device enters SPRING or SPIN, Bulk IN sends a 5-byte packet so the host can synchronize its control:

| Offset | Type | Meaning |
|--------|------|---------|
| 0 | `u8` | magic `0x5B` |
| 1 | `u8` | event `0x01` = mode COG scale |
| 2 | `u8` | mode (`6` = SPRING, `7` = SPIN) |
| 3 | `u16` LE | current mode scale × 1000 |

SPRING and SPIN keep independent RAM values. They initialize from the compiled `SPRING_COG_FF_SCALE` and `SPIN_COG_FF_SCALE` defaults; `SET_COG_SCALE` modifies only the currently active feel mode.

### Failure semantics

- `START` / `STOP` / `SET_K` / `SET_REST` always succeed at the protocol layer (`status=1`). `SET_COG_SCALE` succeeds with a 2-byte payload while in SPRING or SPIN; it returns `status=0` for a short payload or any other mode.
- `SPRING` / `SPIN` / `GEAR` / `TEST` / `STRESS` / `GOTO` return `status=0` if the motor is not aligned/armed yet (same as CDC `need START`), or if `GOTO` payload is fewer than 4 bytes. Successful `GOTO` skips ACK entirely.
- `UPLOAD` reboots into UF2; no ACK. Host should wait for the device to reappear as a mass-storage / picoboot target.
- Unknown `cmd`: no ACK; ignored.
- Empty OUT transfer: ignored.

## CDC side channel (debug only)

CDC remains for logs. Text lines ending in `\n` or `\r` still accept: `START`, `STOP`, `SPRING`, `SPIN`, `GEAR`, `TEST`, `STRESS`, `COGCAL`, `COGCLEAR`, `GOTO <mrad>`, `DUMP`, `UPLOAD`. Binary hosts should use Vendor Bulk only.

Example: stream `GOTO 3142` (or Bulk `0x06` + LE int32) at your sim rate to track about +π rad absolute.

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
    if len(data) < 24 or data[0] != 0xA5:
        return None
    magic, mode, angle_mrad, da, db, dc, seq, id_ma, iq_ma, iq_ref_ma, vbus_mv, uq_q15 = (
        struct.unpack_from("<BBi3hHhhhHh", data, 0)
    )
    return {
        "mode": mode,
        "angle_mrad": angle_mrad,
        "duty": (da / 32767.0, db / 32767.0, dc / 32767.0),
        "seq": seq,
        "id_A": id_ma / 1000.0,
        "iq_A": iq_ma / 1000.0,
        "iq_ref_A": iq_ref_ma / 1000.0,
        "vbus_V": vbus_mv / 1000.0,
        "uq": uq_q15 / 32767.0,
        "dir": struct.unpack_from("<b", data, 24)[0] if len(data) >= 25 else None,
    }

send_cmd(0x01)           # START
# Stream tracking setpoints (sim wheel). No ACK on success.
def goto_mrad(mrad: int) -> None:
    dev.write(EP_OUT, struct.pack("<Bi", 0x06, mrad), timeout=1000)

goto_mrad(3142)          # ~+π rad
while True:
    frame = read_telem()
    if isinstance(frame, dict):
        print(frame)
        # optionally: goto_mrad(sim_angle_mrad)
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
	int16_t  id_mA;
	int16_t  iq_mA;
	int16_t  iq_ref_mA;
	uint16_t vbus_mV;
	int16_t  uq_q15;      /* current-loop Uq, Q15 */
	int8_t   dir;         /* motor/encoder direction: -1 or +1 */
} tinyknob_telem_t; /* 25 bytes */

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
	TK_CMD_GOTO     = 0x06,
	TK_CMD_STRESS   = 0x07,
	TK_CMD_SET_K    = 0x20,
	TK_CMD_SET_REST = 0x21,
	TK_CMD_SET_COG_SCALE = 0x22,
	TK_CMD_UPLOAD   = 0x7F,
};
```
