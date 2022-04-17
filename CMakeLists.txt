cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# File and project name as variables, change as needed
set(project_name "pico_micro_ros_example")
set(project_file "pico_micro_ros_example.c")

project(${project_name} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(libmicroros)
add_executable(${project_name}
    ${project_file}
    pico_uart_transport.c
    libdcmotor/DC_Motor.h
    libdcmotor/DC_Motor.c
)
target_link_libraries(${project_name}
    pico_stdlib
    hardware_pwm
    microros
)

target_include_directories(${project_name} PUBLIC
    libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(${project_name} 1)
pico_enable_stdio_uart(${project_name} 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(${project_name})
