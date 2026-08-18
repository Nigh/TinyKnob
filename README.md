# TinyKnob

RP2040-Zero + DRV8316 + MT6701 force-feedback knob.

`PID=0x4005`
`VID=0xACDC`

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

On power-up the firmware stays idle with INHx held low (no PWM switching; low quiescent power). CDC `START` + newline runs align, then waits. Switch feel with `SPRING` / `TEST` / `STOP`. Continuous SSI CRC failures trip FAULT and return to the same low-side brake idle. True phase Hi-Z needs INLx wired later.

USB enumerates CDC (logs) and HID. HID report byte 0:

- `0x10` get state (mode, dir, pos, offset, K)
- `0x20` set spring K, byte 1 = K * 10
- `0x21` set rest to current angle
- anything else: echo with each byte incremented by 1 (ping)

CDC commands (line ending `\n` or `\r`):

- `START` align, then idle (armed)
- `SPRING` virtual spring at current angle (needs START)
- `TEST` loop ±360° with 5th-order ease (~1.4s move + ≤200ms hold) (needs START)
- `STOP` brake / idle, keep align
- `DUMP` print one full SSI frame (bits + shift candidates), waits on CDC so lines are complete
- `UPLOAD` reboot to UF2 bootloader

Periodic status (~1 Hz, or 4 Hz during align/TEST) is a short `m/p/o/f` line plus one `ssi` brief line. CDC TX uses a 1 KB ring drained in the main loop; the 4 Hz timer IRQ never writes USB.
