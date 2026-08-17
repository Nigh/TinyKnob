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

- Pico will enumerate two USB devices, specifically a `CDC` device and an `HID` device. 
- The log will be printed out via the `CDC` serial port. 
- Any data frame written to the `HID` device will be printed out from the `CDC` serial port, and the `HID` will also return a data frame with each byte incremented by 1.
