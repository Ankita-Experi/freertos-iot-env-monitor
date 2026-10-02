# CMake toolchain file for GNU Arm Embedded (arm-none-eabi-gcc).
# Usage: cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
# If the toolchain is not on PATH, pass -DARM_TOOLCHAIN_DIR=/path/to/bin

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if(DEFINED ARM_TOOLCHAIN_DIR)
  set(_p "${ARM_TOOLCHAIN_DIR}/")
else()
  set(_p "")
endif()

set(CMAKE_C_COMPILER   ${_p}arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER ${_p}arm-none-eabi-gcc)
set(CMAKE_OBJCOPY      ${_p}arm-none-eabi-objcopy)
set(CMAKE_SIZE         ${_p}arm-none-eabi-size)

# Do not try to link a host executable while probing the compiler.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
