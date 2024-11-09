/* Copyright 2024 Adam Green (https://github.com/adamgreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
// Configuration settings for mri_swd firmware.
#ifndef CONFIG_H_
#define CONFIG_H_


// GDB socket port number.
#define GDB_SOCKET_PORT_NUMBER 2331

// UART <-> WiFi bridge IP port number.
#define UART_WIFI_BRIDGE_PORT_NUMBER 2332

// SWD pin connections.
#define SWCLK_PIN 2
#define SWDIO_PIN 3

// UART pin connections / settings used for the UART<->WiFi Bridge.
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define UART_BAUD_RATE 230400

// OLED Display pin connections / setting.
#define OLED_MOSI_PIN      19
#define OLED_CLK_PIN       18
#define OLED_DC_PIN        20
#define OLED_CS_PIN        16
#define OLED_RESET_PIN     17
#define OLED_SCREEN_WIDTH  128
#define OLED_SCREEN_HEIGHT 32


// Should the debugger halt the CPU as soon as it attaches? This is useful to break out of an infinite reset loop.
#define HALT_ON_ATTACH false

// Timeout in milliseconds to use when attempting to connect to the WiFi router.
#define WIFI_ROUTER_TIMEOUT_MS 30000

// Default SWD clock rate used when initially probing the device. It will be increased later for known devices that
// support higher clock rates.
#define DEFAULT_SWD_CLOCK_RATE 8000000

// Maximum number of DPv2 targets supported on a single SWD bus. This is currently 2 for the dual core RP2040 device.
#define MAX_DPV2_TARGETS 2

// Amount of time in ms to delay between attempts to attach to a SWD target (ie. to recover from it not being powered
// up).
#define DELAY_BETWEEN_SWD_ATTACH_ATTEMPTS_MS 1000

// When attempting to attach to the SWD target with the above delay between iterations, output will be logged to stdout
// on this many iterations to let the user know that we are still trying to connect.
#define LOG_EVERY_N_SWD_ATTACH_ATTEMPTS 60

// Timeout in milliseconds to use when reading the DHCSR to obtain the halt and reset state.
#define READ_DHCSR_TIMEOUT_MS 1000

// Timeout in milliseconds to use when executing LocalFlashing operations.
#define LOCAL_FLASHING_OP_TIMEOUT 30000


// Set each of these to 1 or 0 to enable or disable error/debug logging for each of the modules.
// swd.cpp logging
#define LOGGING_SWD_ERROR_ENABLED 1
#define LOGGING_SWD_DEBUG_ENABLED 1

// main.cpp logging
#define LOGGING_MAIN_ERROR_ENABLED 1
#define LOGGING_MAIN_DEBUG_ENABLED 1

// platform.cpp logging
#define LOGGING_PLATFORM_ERROR_ENABLED 1
#define LOGGING_PLATFORM_DEBUG_ENABLED 1

// socket_server.cpp logging
#define LOGGING_SOCKET_SERVER_ERROR_ENABLED 1
#define LOGGING_SOCKET_SERVER_DEBUG_ENABLED 0

// gdb_socket.cpp logging
#define LOGGING_GDB_SOCKET_ERROR_ENABLED 1
#define LOGGING_GDB_SOCKET_DEBUG_ENABLED 0

// uart_wifi_bridge.cpp logging
#define LOGGING_UART_WIFI_BRIDGE_ERROR_ENABLED 1
#define LOGGING_UART_WIFI_BRIDGE_DEBUG_ENABLED 0

// Modules under the devices/ directory
#define LOGGING_DEVICE_ERROR_ENABLED 1
#define LOGGING_DEVICE_DEBUG_ENABLED 1

// cpu_core.cpp logging
#define LOGGING_CPU_CORE_ERROR_ENABLED 1
#define LOGGING_CPU_CORE_DEBUG_ENABLED 1

// ui.cpp logging
#define LOGGING_UI_ERROR_ENABLED 1
#define LOGGING_UI_DEBUG_ENABLED 1

// Size of the packet buffer to use for communicating with GDB.
// Must be large enough to contain a 'G' packet sent from GDB to update all of the CPU registers in the context at once.
// This is:
//      1 (byte for 'G' itself) +
//        [ 2 (text hex digits per byte) *
//          4 (bytes per 32-bit word) *
//          56 (registers store in context for CPU w/ FPU) ] +
//      4 (bytes for packet overhead of '$', '#', and 2 hex digit checksum)
//      = 1 + 2 * 4 * 56 + 4 = 453
// Using (2*7k)+4 for now as that aligns the reads up to the 1k boundaries that SWD uses with the TAR but is
// less than the 16k limit that GDB doesn't seem to like going over.
#define PACKET_SIZE ((2 * 7 * 1024)+4)

#endif // CONFIG_H_
