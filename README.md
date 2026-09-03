# TinyKnob

DRV8316 + MT6701 force-feedback knob firmware. The complete implementation targets
Waveshare **RP2350-Zero**. An STM32G431CBU6 LL build/DFU skeleton is also available;
it deliberately has no board I/O or motor support until its pin map is supplied.

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

### WeAct STM32G431CBU6 mapping

The STM32 target uses the WeAct STM32G431 Core Board QFN48 V1.0 pinout. This is
the assigned TinyKnob wiring used by the minimal motor-test firmware.

| TinyKnob function | WeAct pin | STM32 peripheral |
|---|---|---|
| PWM_A / INHA | PA8 | TIM1_CH1 |
| PWM_B / INHB | PA9 | TIM1_CH2 |
| PWM_C / INHC | PA10 | TIM1_CH3 |
| DRV_EN / nSLEEP | PC4 | GPIO output |
| MT6701 CSN | PA4 | GPIO output |
| MT6701 CLK | PA5 | SPI1_SCK |
| MT6701 DO | PA6 | SPI1_MISO |
| DRV8316 SOA / CSA | PA0 | ADC input |
| DRV8316 SOB / CSB | PA1 | ADC input |
| DRV8316 SOC / CSC | PA2 | ADC input |
| 0.1x VCC bus sense | PA3 | ADC input |
| USB D- | PA11 | USB_DM |
| USB D+ | PA12 | USB_DP |
| Status LED | PC6 | On-board blue LED |
| User button | PC13 | On-board button |
| BOOT0 | PB8 | On-board BOOT0 button |

The board has an 8 MHz HSE and 32.768 kHz LSE. Keep USB-C solder bridges in
their factory state: SB3-SB7 remain open, while SB8/SB9 remain closed. Enabling
USB-C Power Delivery reserves PA9, PA10, PB2, PB4, and PB6, conflicting with the
PWM assignment above. Enter ROM DFU by holding BOOT0 while pressing and
releasing NRST.

Sources: [WeAct schematic and board files](https://github.com/WeActStudio/WeActStudio.STM32G431CoreBoard)
and [Zephyr board documentation](https://docs.zephyrproject.org/latest/boards/weact/stm32g431_core/doc/index.html).

## Prepare

Clone dependencies once before building STM32G4:

```shell
make submodules
# RP2350 Docker image
docker pull xianii/pico-sdk:latest
```

Needs Pico SDK **≥2.0** (RP2350). Image `xianii/pico-sdk:2.3.0` / `latest` is fine.

### build

```shell
# Default: RP2350 Docker build → build/rp2350/src/RP2350_TinyKnob.uf2
make
make TARGET=rp2350 build       # local Pico SDK build
make TARGET=rp2350 flash       # copy UF2 to RP2350_MOUNT

# STM32CubeG4 CMSIS/LL + Arm GNU toolchain
make TARGET=stm32g4 build
make TARGET=stm32g4 flash      # ROM USB DFU, VID:PID 0483:df11

make TARGET=rp2350 clean
make TARGET=stm32g4 clean
```

The STM32 build emits `.elf`, `.bin`, `.hex`, and `.map` files under
`build/stm32g4/platforms/stm32g4/`. To flash, connect USB D-/D+, enter the
STM32 system bootloader using BOOT0/reset, and install `dfu-util`.

### STM32G4 FOC motor test

The STM32G4 target uses TIM1 20 kHz center-aligned PWM and TIM1-TRGO2-triggered
ADC1 DMA sampling of all three DRV8316 current-sense outputs plus Vbus. The d/q
current PI runs at 10 kHz. START performs CSA offset calibration, rotor alignment,
direction detection, and electrical-zero calibration before returning to IDLE.

It uses the same Vendor Bulk endpoints and commands as RP2350 for STOP, START,
TEST, SPRING, SPIN, GOTO/POS, STRESS, GEAR, SET_K, SET_REST, SET_COG_SCALE, and
UPLOAD. `UPLOAD` (`0x7F`) disconnects the application and enters the STM32 ROM
USB DFU bootloader (`0483:DF11`). CDC is intentionally disabled.

Use a current-limited supply for initial validation:

```shell
source ~/venv/bin/activate
python3 tools/stm32g4_motor_test.py --seconds 12
python3 tools/stm32g4_motor_test.py --all-modes
python3 tools/stm32g4_motor_test.py --bootloader
```

The test checks Bulk ACK/telemetry, encoder motion, Vbus measurement, and the sign
agreement between Iq and Iq_ref. CRC failure, alignment motion failure, measured
phase current above 4 A, encoder stall, USB loss, or suspend disables the driver.

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
STM32 CMake project name: `STM32G431_TinyKnob`.

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
