/* Copyright 2023 Adam Green (https://github.com/adamgreen/)

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

// UNDONE: Move other configurations parameters such as GDB packet size, etc.


// GDB socket port number.
#define GDB_SOCKET_PORT_NUMBER 4242


// SWD pin connections.
#define SWCLK_PIN 0
#define SWDIO_PIN 1


// Amount of time in ms to delay between attempts to attach to a SWD target (ie. to recover from it not being powered
// up).
#define DELAY_BETWEEN_SWD_ATTACH_ATTEMPTS_MS 1000

// Timeout in milliseconds to use when reading the DHCSR to obtain the halt and reset state.
#define READ_DHCSR_TIMEOUT_MS 1000


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

// gdb_socket.cpp logging
#define LOGGING_GDB_SOCKET_ERROR_ENABLED 1
#define LOGGING_GDB_SOCKET_DEBUG_ENABLED 0


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
