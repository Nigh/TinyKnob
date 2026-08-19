# TinyKnob

RP2040-Zero + DRV8316 + MT6701 force-feedback knob.

`PID=0x4011`
`VID=0xACDC`

USB binary protocol (Vendor Bulk telemetry + commands): see [docs/usb-protocol.md](docs/usb-protocol.md).

## Wiring

```
RP2040-Zero          DRV8316
GPIO2  ------------- PWM_A / INHA
GPIO4  ------------- PWM_B / INHB
GPIO6  ------------- PWM_C / INHC
GPIO8  ------------- EN / nSLEEP (active high; not DRVOFF)
GND    ------------- GND

RP2040-Zero          MT6701
GPIO9  ------------- CSN
GPIO10 ------------- CLK
GPIO11 <------------ DO
3V3    ------------- VCC
GND    ------------- GND
```

Firmware holds GPIO8 high from boot. DRV8316 is 3x PWM. Motor is a 4015 gimbal, 11 pole pairs. GPIO3/5/7 are left free for INLx later.

Current-sense (SOA/SOB/SOC) is reserved for a later ADC current loop.

## Prepare

### Get docker

```shell
docker pull xianii/pico-sdk:latest
```

### build

```shell
# build (docker, files owned by your user)
make
# remove root-owned build/ from earlier docker runs
make docker_clean
# or: make clean  (falls back to docker_clean if build/ is not writable)
make format
make rebuild
```

## Usage

On power-up the firmware runs align automatically (`START`), then returns to IDLE (armed, green LED). Switch feel with `SPRING` / `SPIN` / `TEST` / `GOTO` / `STOP`. Continuous SSI CRC failures trip FAULT (red LED) and return to idle at 50% zero-voltage PWM. True phase Hi-Z needs INLx wired later.

LED (WS2812): orange blink = aligning; green = idle; blue = spring; cyan = spin; white blink = TEST; yellow = POS; red = fault; purple = UPLOAD (before bootrom).

USB enumerates CDC (logs) and a Vendor Bulk interface. Hosts should use Vendor Bulk for realtime telemetry and mode commands; full layout is in [docs/usb-protocol.md](docs/usb-protocol.md).

Vendor Bulk OUT opcodes:

- `0x01` START (align)
- `0x02` STOP
- `0x03` SPRING
- `0x04` SPIN
- `0x05` TEST
- `0x06` GOTO, next 4 bytes = absolute `angle_mrad` (LE int32); tracking setpoint (streamable)
- `0x20` SET_K, byte 1 = K * 10
- `0x21` SET_REST to current angle
- `0x7F` UPLOAD reboot to UF2 bootloader (no ACK; device disconnects)

Bulk IN pushes 16-byte frames (~1 kHz): magic `0xA5`, mode, angle_mrad, phase duties Q15, seq.

CDC commands (line ending `\n` or `\r`) remain for debug:

- `START` re-run align, then idle (armed); also runs once at boot
- `SPRING` virtual spring at current angle (needs align)
- `SPIN` voltage-mode flywheel (needs align). Under-compensates BEMF so it will not
  self-spin; feel is draggy until an `Iq` current loop can hold `Iq≈0` (needs CSA/CSB/CSC).
  `Uq=0` / idle use 50% zero-voltage PWM (no INLx Hi-Z yet).
- `TEST` loop ±360° with 5th-order ease (~1.4s move + ≤200ms hold) (needs align)
- `GOTO <mrad>` enter tracking mode / update absolute setpoint in milliradians (needs align)
- `STOP` brake / idle, keep align
- `DUMP` print one full SSI frame (bits + shift candidates), waits on CDC so lines are complete
- `UPLOAD` reboot to UF2 bootloader

Periodic status (~1 Hz, or 4 Hz during align/TEST/POS) is a short `m/p/o/f` line plus one `ssi` brief line. CDC TX uses a 1 KB ring drained in the main loop; the 4 Hz timer IRQ never writes USB.
