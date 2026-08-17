cmake_minimum_required(VERSION 3.16)

add_definitions(-DG_LOG_ENABLED=1 -DEVT_LOG_ENABLED=0)
add_subdirectory(scheduler)

add_executable(${PROJECT_NAME} main.c)

file(GLOB SRC *.c)
target_sources(${PROJECT_NAME} PUBLIC ${SRC})

target_include_directories(${PROJECT_NAME} PUBLIC
	${CMAKE_CURRENT_LIST_DIR}
)
pico_set_binary_type(${PROJECT_NAME} copy_to_ram)
# Link with the pico stdlib
# hardware_clocks hardware_i2c hardware_pio hardware_interp hardware_rtc hardware_uart hardware_irq hardware_resets
# pico_unique_id
# tinyusb_host tinyusb_device tinyusb_board
target_link_libraries(${PROJECT_NAME} PUBLIC
	pico_stdlib
	pico_sync
	hardware_sync
	hardware_pio
	hardware_dma
	hardware_irq
	hardware_xosc
	hardware_flash
	tinyusb_device
	scheduler
)
# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})
