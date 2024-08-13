cmake_minimum_required(VERSION 3.26)

if (PICO_SDK_PATH_OVERRIDE)
    set(PICO_SDK_PATH ${PICO_SDK_PATH_OVERRIDE})
else()
    file(REAL_PATH pico-sdk PICO_SDK_PATH BASE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../..")
endif()

# Pull in SDK (must be before project)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(mri-swd-tests C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
set(PICO_BOARD pico)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall -Wextra -Werror
    -Wno-unused-parameter
)

# I prefer -g3 for better symbols.
string(REPLACE "-g" "-g3" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
string(REPLACE "-g" "-g3" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")

add_executable(mri-swd-tests)

target_sources(mri-swd-tests PRIVATE
    main.c
    tests.S
    # Compile the following 2 files to get semi-host support for stdin/stdout/stderr and other file APIs.
    ../../shared/newlib_retarget.c
    ../../shared/newlib_stubs.S
)

# Enable all semi-hosting thunks to redirect all file system calls to GDB.
target_compile_definitions(mri-swd-tests PRIVATE
    MRI_SEMIHOST_SUPPORT_FILE
)

target_link_libraries(mri-swd-tests PRIVATE
    pico_stdlib
    pico_multicore
)
pico_add_extra_outputs(mri-swd-tests)

pico_enable_stdio_usb(mri-swd-tests 0)
pico_enable_stdio_uart(mri-swd-tests 0)
pico_enable_stdio_semihosting(mri-swd-tests 1)

# Show the size of the binary.
pico_find_compiler(PICO_OBJSIZE ${PICO_GCC_TRIPLE}-size)
add_custom_target(size_elf ALL
    COMMAND ${PICO_OBJSIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Sizing  ${PROJECT_NAME}"
    VERBATIM
)
