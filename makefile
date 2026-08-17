

BUILD_DIR = build

.PHONY: clean default build rebuild

default: docker_build

build:
	cmake . -G Ninja -B$(BUILD_DIR) -S.
	ninja -C $(BUILD_DIR)

clean:
	@if [ -d "./$(BUILD_DIR)" ]; then rm ./$(BUILD_DIR) -rf; fi

rebuild: clean build

format:
	bash format.sh

.PHONY: docker_build
docker_build:
	docker run --rm \
	-v ${PWD}:/workspace \
	-w /workspace \
	xianii/pico-sdk:latest /bin/bash -c "cmake . -G Ninja -Bbuild -S. && ninja -C build"
