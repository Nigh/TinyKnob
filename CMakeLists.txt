cmake_minimum_required(VERSION 3.16)

# Waveshare RP2350-Zero (same form-factor / GPIO map as prior RP2040-Zero TinyKnob).
set(PICO_BOARD waveshare_rp2350_zero CACHE STRING "Pico board")

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(RP2350_TinyKnob C CXX ASM)

pico_sdk_init()

add_subdirectory(src)
