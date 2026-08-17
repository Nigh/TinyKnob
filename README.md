# pico USB CDC+HID template
rpi pico USB CDC+HID template

`PID=0x4005`
`VID=0xACDC`

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
