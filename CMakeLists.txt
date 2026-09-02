cmake_minimum_required(VERSION 3.16)

set(TINYKNOB_TARGET "rp2350" CACHE STRING "TinyKnob target: rp2350 or stm32g4")
set_property(CACHE TINYKNOB_TARGET PROPERTY STRINGS rp2350 stm32g4)

if(TINYKNOB_TARGET STREQUAL "rp2350")
	# Waveshare RP2350-Zero (same form-factor / GPIO map as prior RP2040-Zero TinyKnob).
	set(PICO_BOARD waveshare_rp2350_zero CACHE STRING "Pico board")
	if(NOT DEFINED ENV{PICO_SDK_PATH})
		message(FATAL_ERROR "PICO_SDK_PATH is required for TINYKNOB_TARGET=rp2350")
	endif()
	include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
	project(RP2350_TinyKnob C CXX ASM)
	pico_sdk_init()
	add_subdirectory(src)
elseif(TINYKNOB_TARGET STREQUAL "stm32g4")
	project(STM32G431_TinyKnob C ASM)
	add_subdirectory(platforms/stm32g4)
else()
	message(FATAL_ERROR
		"Unsupported TINYKNOB_TARGET='${TINYKNOB_TARGET}'; choose rp2350 or stm32g4")
endif()
