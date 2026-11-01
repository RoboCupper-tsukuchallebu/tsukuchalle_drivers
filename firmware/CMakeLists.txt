cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "RP2350-CAN matches the Pico 2 layout (RP2350A, 4 MB flash)")
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "")

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ddt_can_bridge C CXX ASM)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

add_executable(ddt_can_bridge
    src/main.cpp
    src/bridge_controller.cpp
    src/motor_protocol.cpp
    src/pc_protocol.cpp
    src/xl2515.cpp
)

target_compile_options(ddt_can_bridge PRIVATE -Wall -Wextra)
target_compile_definitions(ddt_can_bridge PRIVATE
    # A PC that stops reading must not stall the control loop or trip the watchdog.
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=2000
    # USB-triggered BOOTSEL would drop to ROM mid-run without stopping the motors.
    # Flashing therefore requires the BOOT button.
    PICO_ENABLE_USB_RESET_VIA_BAUD_RATE=0
    PICO_ENABLE_USB_RESET_VIA_VENDOR_INTERFACE=0
)

target_link_libraries(ddt_can_bridge
    pico_stdlib
    hardware_spi
    hardware_watchdog
)

pico_enable_stdio_usb(ddt_can_bridge 1)
pico_enable_stdio_uart(ddt_can_bridge 0)
pico_add_extra_outputs(ddt_can_bridge)
