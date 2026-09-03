TARGET ?= rp2350
VALID_TARGETS := rp2350 stm32g4
BUILD_DIR := build/$(TARGET)
RP2350_MOUNT ?= /media/$(USER)/RP2350
STM32_DFU_ADDRESS ?= 0x08000000

ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error unsupported TARGET '$(TARGET)'; choose one of: $(VALID_TARGETS))
endif

.PHONY: default build flash clean rebuild format docker_build docker_clean submodules

ifeq ($(TARGET),rp2350)
default: docker_build
else
default: build
endif

build:
ifeq ($(TARGET),rp2350)
	cmake -G Ninja -B$(BUILD_DIR) -S. \
		-DTINYKNOB_TARGET=rp2350 -DPICO_BOARD=waveshare_rp2350_zero
else
	@test -f third_party/STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g431xx.h || \
		{ echo "STM32CubeG4 dependencies missing; run 'make submodules'" >&2; exit 1; }
	cmake -G Ninja -B$(BUILD_DIR) -S. \
		-DTINYKNOB_TARGET=stm32g4 \
		-DCMAKE_TOOLCHAIN_FILE=platforms/stm32g4/arm-none-eabi.cmake
endif
	ninja -C $(BUILD_DIR)

flash:
ifeq ($(TARGET),rp2350)
	@test -d "$(RP2350_MOUNT)" || \
		{ echo "RP2350 UF2 volume not found at $(RP2350_MOUNT)" >&2; exit 1; }
	cp $(BUILD_DIR)/src/RP2350_TinyKnob.uf2 "$(RP2350_MOUNT)/fw.uf2"
	sync
else
	@command -v dfu-util >/dev/null || { echo "dfu-util is required" >&2; exit 1; }
	@lsusb -d 0483:df11 >/dev/null 2>&1 || \
		{ echo "STM32 ROM DFU device 0483:df11 not found" >&2; exit 1; }
	@set +e; \
	dfu-util -d 0483:df11 -a 0 -R -s $(STM32_DFU_ADDRESS):leave \
		-D $(BUILD_DIR)/platforms/stm32g4/STM32G431_TinyKnob.bin; \
	rc=$$?; set -e; \
	if [ $$rc -eq 0 ]; then exit 0; fi; \
	for i in $$(seq 1 30); do \
		if lsusb -d acdc:4011 >/dev/null 2>&1; then \
			echo "STM32G4 application started (ROM DFU disconnected before final status)"; \
			exit 0; \
		fi; \
		sleep 0.1; \
	done; \
	exit $$rc
endif

# Host clean; if an RP2350 build is root-owned from Docker, use docker_clean.
clean:
	@if [ ! -d "./$(BUILD_DIR)" ]; then exit 0; fi; \
	if [ -w "./$(BUILD_DIR)" ]; then rm -rf "./$(BUILD_DIR)"; \
	elif [ "$(TARGET)" = rp2350 ]; then $(MAKE) TARGET=rp2350 docker_clean; \
	else echo "$(BUILD_DIR) is not writable" >&2; exit 1; fi

rebuild: clean build

format:
	bash format.sh

submodules:
	git submodule update --init third_party/STM32CubeG4 third_party/tinyusb
	git -C third_party/STM32CubeG4 submodule update --init \
		Drivers/CMSIS/Device/ST/STM32G4xx Drivers/STM32G4xx_HAL_Driver

docker_clean:
	docker run --rm \
	-v ${PWD}:/workspace \
	-w /workspace \
	xianii/pico-sdk:latest rm -rf build/rp2350

docker_build:
	@test "$(TARGET)" = rp2350 || \
		{ echo "docker_build only supports TARGET=rp2350" >&2; exit 1; }
	docker run --rm \
	-u $(shell id -u):$(shell id -g) \
	-e HOME=/tmp \
	-v ${PWD}:/workspace \
	-w /workspace \
	xianii/pico-sdk:latest /bin/bash -c \
		"cmake -G Ninja -Bbuild/rp2350 -S. -DTINYKNOB_TARGET=rp2350 -DPICO_BOARD=waveshare_rp2350_zero && ninja -C build/rp2350"
