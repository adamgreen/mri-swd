cmake_minimum_required(VERSION 3.26)

# UNDONE: Need to update the pico-sdk folder to match what I have in my Makefile.
if (PICO_SDK_PATH_OVERRIDE)
    set(PICO_SDK_PATH ${PICO_SDK_PATH_OVERRIDE})
else()
    file(REAL_PATH pico-sdk PICO_SDK_PATH BASE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}")
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

# I prefer -g3 for better symbols and -O3 for better optimization.
string(REPLACE "-g" "-g3" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
string(REPLACE "-g" "-g3" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")
string(REPLACE "-O2" "-O3" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
string(REPLACE "-O2" "-O3" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")

add_executable(mri-swd)

# Include wifi_config.cmake which should contain the user's WIFI credentials.
include(wifi_config.cmake)

target_compile_definitions(mri-swd PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
)

# Disable the printf()s in the WiFi driver, even when NDEBUG isn't set.
set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -DCYW43_DEBUG\\\(...\\\)=\\\(void\\\)0")
target_compile_definitions(mri-swd PRIVATE
    PICO_CYW43_ARCH_DEBUG_ENABLED=0
)

pico_generate_pio_header(mri-swd ${CMAKE_CURRENT_LIST_DIR}/swd.pio)

target_sources(mri-swd PRIVATE
    main.cpp
    swd.cpp
    gdb_socket.cpp
    mri_platform.cpp
    cpu_core.cpp
    )

target_include_directories(mri-swd PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

# Sources for devices and device families supported by mri-swd.
target_sources(mri-swd PRIVATE
    devices/devices.cpp
    devices/rp2040.cpp
    devices/nrf52.cpp
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
    mri/core/mri.c
    mri/core/packet.c
    mri/core/token.c
    mri/core/try_catch.c
    mri/semihost/newlib/semihost_newlib.c
    mri/semihost/mbed/semihost_mbed.c
    )

target_include_directories(mri-swd PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/mri
    )

target_compile_definitions(mri-swd PRIVATE
    MRI_ALWAYS_USE_HARDWARE_BREAKPOINT=0
)

target_link_libraries(mri-swd PRIVATE
    pico_stdlib
    hardware_pio
    pico_cyw43_arch_lwip_threadsafe_background
    )
pico_add_extra_outputs(mri-swd)

pico_enable_stdio_usb(mri-swd 1)
pico_enable_stdio_uart(mri-swd 0)

# Show the size of the binary.
pico_find_compiler(PICO_OBJSIZE ${PICO_GCC_TRIPLE}-size)
add_custom_target(size_elf ALL
        COMMAND ${PICO_OBJSIZE} $<TARGET_FILE:${PROJECT_NAME}>
        COMMENT "Sizing  ${PROJECT_NAME}"
        VERBATIM
)

# bin2h tool required to build local flashing binary header files.
add_custom_target(bin2h_target DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/bin2h)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/bin2h
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/bin2h/main.c ${CMAKE_CURRENT_LIST_DIR}/bin2h/common.c ${CMAKE_CURRENT_LIST_DIR}/bin2h/try_catch.c
        COMMAND CC ${CMAKE_CURRENT_LIST_DIR}/bin2h/main.c ${CMAKE_CURRENT_LIST_DIR}/bin2h/common.c ${CMAKE_CURRENT_LIST_DIR}/bin2h/try_catch.c -o ${CMAKE_CURRENT_BINARY_DIR}/bin2h
        VERBATIM)

# Supported devices
# RP2040
add_custom_target(rp2040-LocalFlashing DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/rp2040-LocalFlashing.h)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/rp2040-LocalFlashing.h
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/devices/rp2040-LocalFlashing.c
        COMMAND ${PICO_COMPILER_CC} -mcpu=cortex-m0plus -mthumb -O3 -g3  -std=gnu11 -Wall -Wextra -Werror -Wno-unused-parameter -fno-jump-tables -fno-builtin -ffunction-sections -fdata-sections -nostartfiles -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/rp2040-LocalFlashing.map -Wl,--script=${CMAKE_CURRENT_LIST_DIR}/devices/rp2040-LocalFlashing.ld -Wl,-z,max-page-size=4096 -Wl,--gc-sections ${CMAKE_CURRENT_LIST_DIR}/devices/rp2040-LocalFlashing.c -o ${CMAKE_CURRENT_BINARY_DIR}/rp2040-LocalFlashing.elf
        COMMAND ${PICO_OBJCOPY} -Obinary ${CMAKE_CURRENT_BINARY_DIR}/rp2040-LocalFlashing.elf ${CMAKE_CURRENT_BINARY_DIR}/rp2040-LocalFlashing.bin
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/bin2h ${CMAKE_CURRENT_BINARY_DIR}/rp2040-LocalFlashing.bin ${CMAKE_CURRENT_BINARY_DIR}/rp2040-LocalFlashing.h g_rp2040_LocalFlashing
        VERBATIM)
add_dependencies(rp2040-LocalFlashing bin2h_target)
add_dependencies(mri-swd rp2040-LocalFlashing)
