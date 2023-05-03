cmake_minimum_required(VERSION 3.26)

# UNDONE: Need to update the pico-sdk folder to match what I have in my Makefile.
if (PICO_SDK_PATH_OVERRIDE)
    set(PICO_SDK_PATH ${PICO_SDK_PATH_OVERRIDE})
else()
    file(REAL_PATH ../pico-sdk PICO_SDK_PATH BASE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}")
endif()

# Pull in SDK (must be before project)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(mri-swd C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(PICO_BOARD pico_w)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall -Wextra -Werror
        #-Wno-unused-function # we have some for the docs that aren't called
        -Wno-unused-parameter
        )

# UNDONE: I want asserts enabled in Release builds for early testing.
string(REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
string(REPLACE "-DNDEBUG" "" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")

# UNDONE: Put all optimization flags back to -O3.
# I prefer -g3 for better symbols and -O3 for better optimization.
string(REPLACE "-g" "-g3" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
string(REPLACE "-g" "-g3" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")
string(REPLACE "-O2" "-O3" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
string(REPLACE "-O2" "-O3" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")

add_executable(mri-swd)

# Include wifi_config.cmake if it exists. If it exists, it will have the user's WIFI credentials.
include(wifi_config.cmake OPTIONAL)

target_compile_definitions(mri-swd PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
)

pico_generate_pio_header(mri-swd ${CMAKE_CURRENT_LIST_DIR}/swd.pio)

target_sources(mri-swd PRIVATE
    main.cpp
    swd.cpp
    gdb_socket.cpp
    mri_platform.cpp
    )

target_include_directories(mri-swd PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

# mri submodule files.
target_sources(mri-swd PRIVATE
    mri/core/buffer.c
    mri/core/cmd_break_watch.c
    mri/core/cmd_common.c
    mri/core/cmd_continue.c
    mri/core/cmd_file.c
    mri/core/cmd_memory.c
    mri/core/cmd_query.c
    mri/core/cmd_registers.c
    mri/core/cmd_step.c
    mri/core/cmd_thread.c
    mri/core/cmd_vcont.c
    mri/core/context.c
    mri/core/gdb_console.c
    mri/core/libc.c
    mri/core/memory.c
    mri/core/mri.c
    mri/core/packet.c
    mri/core/token.c
    mri/core/try_catch.c
    )

target_include_directories(mri-swd PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/mri
    )

target_compile_definitions(mri-swd PRIVATE
    MRI_ALWAYS_USE_HARDWARE_BREAKPOINT=1
)

target_link_libraries(mri-swd PRIVATE
    pico_stdlib
    hardware_pio
    pico_cyw43_arch_lwip_threadsafe_background
    )
pico_add_extra_outputs(mri-swd)

pico_enable_stdio_usb(mri-swd 1)
pico_enable_stdio_uart(mri-swd 0)
