cmake_minimum_required(VERSION 3.20)

project(env_monitor C ASM)

# ---------------------------------------------------------------------------
# Build options
# ---------------------------------------------------------------------------
option(CAN_SELF_TEST "Run bxCAN in silent-loopback mode (single board, no transceiver needed)" OFF)
set(BME280_I2C_ADDR "0x76" CACHE STRING "BME280 7-bit I2C address (0x76 if SDO=GND, 0x77 if SDO=VDDIO)")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(HAL ${LIB}/stm32f4xx-hal-driver)
set(RTOS ${LIB}/FreeRTOS-Kernel)

foreach(_d ${HAL} ${RTOS} ${LIB}/cmsis-device-f4 ${LIB}/cmsis-core)
  if(NOT EXISTS ${_d}/LICENSE.md)
    message(FATAL_ERROR "Missing ${_d}. Run: git submodule update --init --depth 1")
  endif()
endforeach()

# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------
file(GLOB APP_SRC  CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/*.c)
file(GLOB CORE_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.c)

set(HAL_MODULES
  hal hal_cortex hal_rcc hal_rcc_ex hal_gpio hal_exti hal_dma hal_dma_ex
  hal_flash hal_flash_ex hal_pwr hal_pwr_ex hal_uart hal_i2c hal_i2c_ex
  hal_can hal_iwdg hal_tim hal_tim_ex)
set(HAL_SRC "")
foreach(_m ${HAL_MODULES})
  list(APPEND HAL_SRC ${HAL}/Src/stm32f4xx_${_m}.c)
endforeach()

set(RTOS_SRC
  ${RTOS}/tasks.c
  ${RTOS}/queue.c
  ${RTOS}/list.c
  ${RTOS}/event_groups.c
  ${RTOS}/timers.c
  ${RTOS}/portable/GCC/ARM_CM4F/port.c
  ${RTOS}/portable/MemMang/heap_4.c)

set(STARTUP_SRC
  ${LIB}/cmsis-device-f4/Source/Templates/gcc/startup_stm32f446xx.s
  ${LIB}/cmsis-device-f4/Source/Templates/system_stm32f4xx.c)

add_executable(${PROJECT_NAME}.elf ${APP_SRC} ${CORE_SRC} ${HAL_SRC} ${RTOS_SRC} ${STARTUP_SRC})

target_include_directories(${PROJECT_NAME}.elf PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
  ${CMAKE_CURRENT_SOURCE_DIR}/App/Inc
  ${HAL}/Inc
  ${LIB}/cmsis-device-f4/Include
  ${LIB}/cmsis-core/CMSIS/Core/Include
  ${RTOS}/include
  ${RTOS}/portable/GCC/ARM_CM4F)

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
  STM32F446xx
  USE_HAL_DRIVER
  BME280_I2C_ADDR=${BME280_I2C_ADDR}
  $<$<BOOL:${CAN_SELF_TEST}>:CAN_SELF_TEST=1>)

set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

target_compile_options(${PROJECT_NAME}.elf PRIVATE
  ${MCU_FLAGS}
  -ffunction-sections -fdata-sections -fno-common
  $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
  $<$<CONFIG:Debug>:-Og -g3>
  $<$<CONFIG:Release>:-O2 -g>)

# Strict warnings for our own code only; vendor code is left as shipped.
set_source_files_properties(${APP_SRC} ${CORE_SRC} PROPERTIES
  COMPILE_OPTIONS "-Wall;-Wextra;-Wshadow;-Wdouble-promotion;-Wundef")

target_link_options(${PROJECT_NAME}.elf PRIVATE
  ${MCU_FLAGS}
  -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F446RETx_FLASH.ld
  --specs=nano.specs --specs=nosys.specs
  -Wl,--gc-sections
  -Wl,-Map=${PROJECT_NAME}.map,--cref
  -Wl,--print-memory-usage
  -Wl,--no-warn-rwx-segments)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
  COMMAND ${CMAKE_OBJCOPY} -O ihex   ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
  COMMENT "Generating .bin / .hex")
