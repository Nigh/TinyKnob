set(STM32CUBE_G4 "${CMAKE_SOURCE_DIR}/third_party/STM32CubeG4")
set(CMSIS_DEVICE "${STM32CUBE_G4}/Drivers/CMSIS/Device/ST/STM32G4xx")
set(STM32_LL "${STM32CUBE_G4}/Drivers/STM32G4xx_HAL_Driver")
set(TINYUSB "${CMAKE_SOURCE_DIR}/third_party/tinyusb")

foreach(required
		"${CMSIS_DEVICE}/Include/stm32g431xx.h"
		"${STM32_LL}/Inc/stm32g4xx_ll_rcc.h"
		"${TINYUSB}/src/tusb.c")
	if(NOT EXISTS "${required}")
		message(FATAL_ERROR "Firmware dependencies are missing; run 'make submodules'")
	endif()
endforeach()

set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
set(TINYUSB_SOURCES
	${TINYUSB}/src/tusb.c
	${TINYUSB}/src/common/tusb_fifo.c
	${TINYUSB}/src/device/usbd.c
	${TINYUSB}/src/class/vendor/vendor_device.c
	${TINYUSB}/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
	${TINYUSB}/src/portable/st/stm32_fsdev/fsdev_common.c)

add_executable(${PROJECT_NAME}
	main.c
	usb_descriptors.c
	${TINYUSB_SOURCES}
	${CMSIS_DEVICE}/Source/Templates/gcc/startup_stm32g431xx.s
	${CMSIS_DEVICE}/Source/Templates/system_stm32g4xx.c)
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")
set_source_files_properties(${TINYUSB_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")

target_include_directories(${PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${CMAKE_SOURCE_DIR}/src
	${TINYUSB}/src
	${STM32CUBE_G4}/Drivers/CMSIS/Core/Include
	${CMSIS_DEVICE}/Include
	${STM32_LL}/Inc)

target_compile_definitions(${PROJECT_NAME} PRIVATE
	STM32G431xx
	USE_FULL_LL_DRIVER
	CFG_TUSB_MCU=OPT_MCU_STM32G4)
target_compile_options(${PROJECT_NAME} PRIVATE
	${CPU_FLAGS} -ffunction-sections -fdata-sections -fno-common -Wall -Wextra -Werror)

# G431KB and G431CB share the same 128 KiB Flash, 32 KiB SRAM and 10 KiB CCM SRAM map.
set(VENDOR_LINKER_SCRIPT
	"${STM32CUBE_G4}/Projects/NUCLEO-G431KB/Templates_LL/STM32CubeIDE/STM32G431KBTX_FLASH.ld")
# ponytail: CubeG4 v1.6.3 emits GCC 11's READONLY syntax; strip only that marker for GCC 9+.
file(READ "${VENDOR_LINKER_SCRIPT}" LINKER_SCRIPT_CONTENT)
string(REPLACE " (READONLY)" "" LINKER_SCRIPT_CONTENT "${LINKER_SCRIPT_CONTENT}")
set(LINKER_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/STM32G431CBU6_FLASH.ld")
file(WRITE "${LINKER_SCRIPT}" "${LINKER_SCRIPT_CONTENT}")
target_link_options(${PROJECT_NAME} PRIVATE
	${CPU_FLAGS} -T${LINKER_SCRIPT} --specs=nano.specs --specs=nosys.specs
	-Wl,--gc-sections -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
target_link_libraries(${PROJECT_NAME} PRIVATE m)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
	COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
	COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
	COMMENT "Generating STM32G431 BIN and HEX images")
