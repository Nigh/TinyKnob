

BUILD_DIR = build

.PHONY: clean default build rebuild format docker_build docker_clean

default: docker_build

build:
	cmake . -G Ninja -B$(BUILD_DIR) -S.
	ninja -C $(BUILD_DIR)

# Host clean; if build/ is root-owned from docker, falls back to docker_clean.
clean:
	@if [ ! -d "./$(BUILD_DIR)" ]; then exit 0; fi; \
	if [ -w "./$(BUILD_DIR)" ]; then rm -rf "./$(BUILD_DIR)"; \
	else $(MAKE) docker_clean; fi

rebuild: clean build

format:
	bash format.sh

docker_clean:
	docker run --rm \
	-v ${PWD}:/workspace \
	-w /workspace \
	xianii/pico-sdk:latest rm -rf build

docker_build:
	docker run --rm \
	-u $(shell id -u):$(shell id -g) \
	-e HOME=/tmp \
	-v ${PWD}:/workspace \
	-w /workspace \
	xianii/pico-sdk:latest /bin/bash -c "cmake . -G Ninja -Bbuild -S. && ninja -C build"
