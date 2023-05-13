cmake_minimum_required(VERSION 3.26)

# UNDONE: Need to update the pico-sdk folder to match what I have in my Makefile.
if (PICO_SDK_PATH_OVERRIDE)
    set(PICO_SDK_PATH ${PICO_SDK_PATH_OVERRIDE})
else()
    file(REAL_PATH ../../pico-sdk PICO_SDK_PATH BASE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}")
endif()

# Pull in SDK (must be before project)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(perf-picow C CXX ASM)
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

add_executable(perf-picow)

# Include wifi_config.cmake if it exists. If it exists, it will have the user's WIFI credentials.
include(wifi_config.cmake OPTIONAL)

target_compile_definitions(perf-picow PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
)

# Disable the printf()s in the WiFi driver, even when NDEBUG isn't set.
set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -DCYW43_DEBUG\\\(...\\\)=\\\(void\\\)0")
target_compile_definitions(perf-picow PRIVATE
    PICO_CYW43_ARCH_DEBUG_ENABLED=0
)

target_sources(perf-picow PRIVATE
    main.cpp
    )

target_include_directories(perf-picow PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

target_link_libraries(perf-picow PRIVATE
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    )
pico_add_extra_outputs(perf-picow)

pico_enable_stdio_usb(perf-picow 1)
pico_enable_stdio_uart(perf-picow 0)
