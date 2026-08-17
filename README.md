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
GND    ------------- GND

RP2040-Zero          MT6701
GPIO9  ------------- CSN
GPIO10 ------------- CLK
GPIO11 <------------ DO
3V3    ------------- VCC
GND    ------------- GND
```

DRV8316 is already in 3x PWM with nSLEEP held high in hardware. Motor is a 4015 gimbal, 11 pole pairs.

Current-sense (SOA/SOB/SOC) is reserved for a later ADC current loop.

## Prepare

### Get docker

```shell
docker pull xianii/pico-sdk:latest
```

### build

```shell
# build
make
# clang-format
make format
# clear build
make clean
# rebuild
make rebuild
```

## Usage

On power-up the firmware locks the rotor to the d-axis, then closed-loop FOC turns one mechanical revolution forward and one back, then holds a virtual spring.

USB enumerates CDC (logs) and HID. HID report byte 0:

- `0x10` get state (mode, dir, pos, offset, K)
- `0x20` set spring K, byte 1 = K * 10
- `0x21` set rest to current angle
- anything else: echo with each byte incremented by 1 (ping)

CDC `UPLOAD` + newline reboots to UF2 bootloader.
