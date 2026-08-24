# TinyKnob

Waveshare **RP2350-Zero** + DRV8316 + MT6701 force-feedback knob.

`PID=0x4011`
`VID=0xACDC`
Manufacturer `HelloWorks`, Product `TinyRoller`.

USB binary protocol (Vendor Bulk telemetry + commands): see [docs/usb-protocol.md](docs/usb-protocol.md).

## Wiring

```
RP2350-Zero          DRV8316
GPIO2  ------------- PWM_A / INHA
GPIO4  ------------- PWM_B / INHB
GPIO6  ------------- PWM_C / INHC
GPIO8  ------------- EN / nSLEEP (active high; not DRVOFF)
GPIO26 <------------ SOA / CSA
GPIO27 <------------ SOB / CSB
GPIO28 <------------ SOC / CSC
GPIO29 <------------ 0.1×VCC (bus sense; do not wire raw VCC to MCU)
GND    ------------- GND

RP2350-Zero          MT6701
GPIO9  ------------- CSN
GPIO10 ------------- CLK
GPIO11 <------------ DO
3V3    ------------- VCC
GND    ------------- GND
```

Firmware holds GPIO8 high from boot. DRV8316 is 3x PWM. Motor is a 4015 gimbal, 11 pole pairs. GPIO3/5/7 are left free for INLx later. WS2812 on GPIO16 (board default).

Current path (`pins.h`): `CUR_LOOP_EN=1` uses PWM-timed low-side ADC/DMA telemetry; `CUR_LOOP_CTRL=1` enables the validated current PI for TEST/POS/SPIN, while SPRING/STRESS stay voltage-driven for stable feel. Tune `CS_SIGN` / `CS_PHASE_ORD` / `CS_TE_OFF`, and match `CS_GAIN_V_PER_A` to the GAIN pin. Agent bring-up: [AGENTS.md](AGENTS.md).

## Prepare

### Get docker

```shell
docker pull xianii/pico-sdk:latest
```

Needs Pico SDK **≥2.0** (RP2350). Image `xianii/pico-sdk:2.3.0` / `latest` is fine.

### build

```shell
# build (docker, files owned by your user) → build/src/RP2350_TinyKnob.uf2
make
# remove root-owned build/ from earlier docker runs
make docker_clean
# or: make clean  (falls back to docker_clean if build/ is not writable)
make format
make rebuild
```

For rapid cogging-compensation feel tuning (requires `pyusb` and an auto-mounted
RP2350 UF2 volume), one command edits the SPIN default scale, builds, flashes, aligns, and
leaves the device in SPIN:

```shell
source ~/venv/bin/activate
python3 tools/set_cog_scale.py 0.60
```

The script also accepts a device already in UF2 mode. Set `RP2350_MOUNT` for a
nonstandard mount path. If the volume denies writes, only the `cp` step asks for sudo.

CMake project name: `RP2350_TinyKnob`. Board: `waveshare_rp2350_zero`.

## Usage

On power-up the firmware stays **IDLE** (green LED) until you send `START` (Bulk `0x01` or CDC). Align then returns to IDLE (armed). Switch feel with `SPRING` / `SPIN` / `TEST` / `STRESS` / `GOTO` / `STOP`. Continuous SSI CRC failures trip FAULT (red LED) and return to idle at 50% zero-voltage PWM. True phase Hi-Z needs INLx wired later.

LED (WS2812): orange blink = aligning; green = idle; blue = spring; cyan = spin; white blink = TEST; magenta blink = STRESS; yellow = POS; red = fault; purple = UPLOAD (before bootrom).

USB enumerates CDC (logs) and a Vendor Bulk interface. Hosts should use Vendor Bulk for realtime telemetry and mode commands; full layout is in [docs/usb-protocol.md](docs/usb-protocol.md).

### FOC sense-only check (host)

With `CUR_LOOP_CTRL=0`, run a GOTO sweep and auto-diagnose Park/TE alignment:

```shell
pip install pyusb numpy
python3 tools/foc_sense_check.py
python3 tools/foc_sense_check.py --revs 1 --seconds 5 --save /tmp/foc_sense.npz
```

Vendor Bulk OUT opcodes:

- `0x01` START (align)
- `0x02` STOP
- `0x03` SPRING
- `0x04` SPIN
- `0x05` TEST
- `0x06` GOTO, next 4 bytes = absolute `angle_mrad` (LE int32); tracking setpoint (streamable)
- `0x07` STRESS burn-in (+full 3s / stop 1s / −full 3s / stop 1s, smooth Uq ramps)
- `0x08` COG_CAL (slow +1 rev → cogging LUT)
- `0x09` COG_CLEAR
- `0x20` SET_K, byte 1 = K * 10
- `0x21` SET_REST to current angle
- `0x22` SET_COG_SCALE, bytes 1–2 = little-endian `u16(scale * 1000)`; updates current SPRING/SPIN scale immediately. Entering either mode emits `5B 01 mode scale_lo scale_hi`
- `0x7F` UPLOAD reboot to UF2 bootloader (no ACK; device disconnects)

Bulk IN pushes 24-byte frames (~1 kHz): magic `0xA5`, mode, angle, duties, seq, Id/Iq/Iq_ref (mA), Vbus (mV), Uq Q15.

CDC commands (line ending `\n` or `\r`) remain for debug:

- `START` run align, then idle (armed); **not** automatic at boot
- `SPRING` virtual spring at current angle (needs align)
- `SPIN` coast (needs align): light drag + speed cap, no flywheel assist; cog FF cancels
  detent. Flick harder → faster/longer spin. Idle = 50% zero-voltage PWM.
- `TEST` loop ±360° with 5th-order ease (~1.4s move + ≤200ms hold) (needs align)
- `STRESS` burn-in loop: +full 3s, stop 1s, −full 3s, stop 1s; 500ms smoothstep Uq on start/stop (needs align)
- `COGCAL` RAM LUT override (~8s); prefer `tools/cog_cal.py --host-learn --write src/cog_lut_default.h`
- `COGCLEAR` restore flash LUT; `COGDUMP` print active table
- `GOTO <mrad>` enter tracking mode / update absolute setpoint in milliradians (needs align)
- `STOP` brake / idle, keep align
- `DUMP` print one full SSI frame (bits + shift candidates), waits on CDC so lines are complete
- `UPLOAD` reboot to UF2 bootloader

Periodic status (~1 Hz, or 4 Hz during align/TEST/POS) is a short `m/p/o/f` line plus one `ssi` brief line. CDC TX uses a 1 KB ring drained in the main loop; the 4 Hz timer IRQ never writes USB.
