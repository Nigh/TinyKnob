set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(CMAKE_C_COMPILER arm-none-eabi-gcc)
find_program(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy)
find_program(CMAKE_SIZE arm-none-eabi-size)

foreach(tool CMAKE_C_COMPILER CMAKE_ASM_COMPILER CMAKE_OBJCOPY CMAKE_SIZE)
	if(NOT ${tool})
		message(FATAL_ERROR
			"${tool} was not found; add the Arm GNU toolchain bin directory to PATH")
	endif()
endforeach()

set(CMAKE_EXECUTABLE_SUFFIX ".elf")
set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_ASM_COMPILER_WORKS TRUE)
