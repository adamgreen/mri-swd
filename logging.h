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
// Simple printf() logging module.
//
// Before including this header, the module should #define modname_MODULE to be its filename.
// For example swd.cpp might have this:
//  #define SWD_MODULE "swd.cpp"
//  #include "logging.h"
// The currently supported modules are:
//  SWD_MODULE (swd.cpp)
//  MAIN_MODULE (main.cpp)
//  PLATFORM_MODULE (mri_platform.cpp)
//  GDB_SOCKET_MODULE (gdb_socket.cpp)
//
// config.h is used to enable/disable error and/or debug logging for each of the modules.
// Example config.h which enables all logging for all modules:
// #define LOGGING_SWD_ERROR_ENABLED 1
// #define LOGGING_SWD_DEBUG_ENABLED 1
// #define LOGGING_MAIN_ERROR_ENABLED 1
// #define LOGGING_MAIN_DEBUG_ENABLED 1
// #define LOGGING_PLATFORM_ERROR_ENABLED 1
// #define LOGGING_PLATFORM_DEBUG_ENABLED 1
// #define LOGGING_GDB_SOCKET_ERROR_ENABLED 1
// #define LOGGING_GDB_SOCKET_DEBUG_ENABLED 1
#ifndef LOGGING_H_
#define LOGGING_H_

#include <stdio.h>
#include "config.h"

#if defined(SWD_MODULE)
    #define LOGGING_MODULE_FILENAME SWD_MODULE

    #if LOGGING_SWD_ERROR_ENABLED
        #define logError  logError_
        #define logErrorF logErrorF_
    #else
        #define logError(X)
        #define logErrorF(X, ...)
    #endif
    #if LOGGING_SWD_DEBUG_ENABLED
        #define logDebug  logDebug_
        #define logDebugF logDebugF_
    #else
        #define logDebug(X)
        #define logDebugF(X, ...)
    #endif
#elif defined (MAIN_MODULE)
    #define LOGGING_MODULE_FILENAME MAIN_MODULE

    #if LOGGING_MAIN_ERROR_ENABLED
        #define logError logError_
        #define logErrorF logErrorF_
    #else
        #define logError(X)
        #define logErrorF(X, ...)
    #endif
    #if LOGGING_MAIN_DEBUG_ENABLED
        #define logDebug logDebug_
        #define logDebugF logDebugF_
    #else
        #define logDebug(X)
        #define logDebugF(X, ...)
    #endif
#elif defined (PLATFORM_MODULE)
    #define LOGGING_MODULE_FILENAME PLATFORM_MODULE

    #if LOGGING_PLATFORM_ERROR_ENABLED
        #define logError logError_
        #define logErrorF logErrorF_
    #else
        #define logError(X)
        #define logErrorF(X, ...)
    #endif
    #if LOGGING_PLATFORM_DEBUG_ENABLED
        #define logDebug logDebug_
        #define logDebugF logDebugF_
    #else
        #define logDebug(X)
        #define logDebugF(X, ...)
    #endif
#elif defined (GDB_SOCKET_MODULE)
    #define LOGGING_MODULE_FILENAME GDB_SOCKET_MODULE

    #if LOGGING_GDB_SOCKET_ERROR_ENABLED
        #define logError logError_
        #define logErrorF logErrorF_
    #else
        #define logError(X)
        #define logErrorF(X, ...)
    #endif
    #if LOGGING_GDB_SOCKET_DEBUG_ENABLED
        #define logDebug logDebug_
        #define logDebugF logDebugF_
    #else
        #define logDebug(X)
        #define logDebugF(X, ...)
    #endif
#elif defined (DEVICE_MODULE)
    #define LOGGING_MODULE_FILENAME DEVICE_MODULE

    #if LOGGING_DEVICE_ERROR_ENABLED
        #define logError logError_
        #define logErrorF logErrorF_
    #else
        #define logError(X)
        #define logErrorF(X, ...)
    #endif
    #if LOGGING_DEVICE_DEBUG_ENABLED
        #define logDebug logDebug_
        #define logDebugF logDebugF_
    #else
        #define logDebug(X)
        #define logDebugF(X, ...)
    #endif
#endif // SWD_MODULE


#define logError_(X) g_logErrorF("error: %s:%u %s() - " X "\n", LOGGING_MODULE_FILENAME, __LINE__, __FUNCTION__)
#define logErrorF_(X, ...) g_logErrorF("error: %s:%u %s() - " X "\n", LOGGING_MODULE_FILENAME, __LINE__, __FUNCTION__, __VA_ARGS__)
#define logDebug_(X) g_logDebugF("debug: %s:%u %s() - " X "\n", LOGGING_MODULE_FILENAME, __LINE__, __FUNCTION__)
#define logDebugF_(X, ...) g_logDebugF("debug: %s:%u %s() - " X "\n", LOGGING_MODULE_FILENAME, __LINE__, __FUNCTION__, __VA_ARGS__)
#define logInfo(X) printf(" info: %s:%u %s() - " X "\n", LOGGING_MODULE_FILENAME, __LINE__, __FUNCTION__)
#define logInfoF(X, ...) printf(" info: %s:%u %s() - " X "\n", LOGGING_MODULE_FILENAME, __LINE__, __FUNCTION__, __VA_ARGS__)

static int (*g_logErrorF)(const char* format, ...) = printf;
static int (*g_logDebugF)(const char* format, ...) = printf;

static int dummyf(const char* format, ...)
{
    return 0;
}

static inline void logErrorDisable()
{
    g_logErrorF = dummyf;
}

static inline void logErrorEnable()
{
    g_logErrorF = printf;
}

static inline void logDebugDisable()
{
    g_logDebugF = dummyf;
}

static inline void logDebugEnable()
{
    g_logDebugF = printf;
}

#endif // LOGGING_H_
