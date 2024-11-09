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
// Structures shared between mri-swd debug hardware and FLASH programming code that runs locally on the target itself.
// This allows the FLASH erasing and programming to occur in parallel with GDB data transfer.
#ifndef LOCAL_FLASHING_H_
#define LOCAL_FLASHING_H_

#include <stdint.h>

// Set LocalFlashQueueEntry.ramAddress to this value for operations like LOCAL_FLASHING_OP_ERASE which don't use
// RAM.
#define LOCAL_FLASHING_RAM_UNUSED 0xFFFFFFFF

typedef enum
{
    LOCAL_FLASHING_OP_FREE = 0,
    LOCAL_FLASHING_OP_ERASE,
    LOCAL_FLASHING_OP_WRITE,
    LOCAL_FLASHING_OP_COMPLETE,
    LOCAL_FLASHING_OP_BREAKPOINT
} LocalFlashingOperation;

typedef struct
{
    LocalFlashingOperation op;
    uint32_t               errorCode;
    uint32_t               flashAddress;
    uint32_t               ramAddress;
    uint32_t               size;
    uint32_t               alloc;
} LocalFlashingQueueEntry;

typedef struct
{
    volatile uint32_t   writeIndex;
    volatile uint32_t   readIndex;
} LocalFlashingQueueIndices;

typedef struct
{
    LocalFlashingQueueIndices queueIndices;
    uint32_t                  queueAddress;
    uint32_t                  queueLength;
    uint32_t                  ramAddress;
    uint32_t                  ramLength;
    uint32_t                  stackAddress;
    uint32_t                  routineAddress;
    uint32_t                  loadAddress;
} LocalFlashingConfig;

// Object caches pointers to the ROM based FLASH related routines in the object context.
typedef struct LocalFlashingObject
{
    LocalFlashingConfig localFlashingConfig;
    uint32_t            indicesAddress;
    uint32_t            freeIndex;
    uint32_t            writeIndex;
    uint32_t            ramWrite;
    uint32_t            ramRead;
} LocalFlashingObject;


// The C code which runs on the target doesn't need to know about these C++ functions.
#ifdef __cplusplus
#include "cpu_core.h"

bool localFlashingInit(LocalFlashingObject* pObject, CpuCore* pCore, const uint8_t* pTargetCode, size_t targetCodeSize);
bool localFlashingQueueEraseOperation(LocalFlashingObject* pObject, CpuCore* pCore, uint32_t addressStart, uint32_t length);
bool localFlashingQueueProgramOperation(LocalFlashingObject* pObject, CpuCore* pCore, uint32_t addressStart, const void* pBuffer, size_t bufferSize, size_t alignedSize);
bool localFlashingUninit(LocalFlashingObject* pObject, CpuCore* pCore);
#endif // __cplusplus

#endif // LOCAL_FLASHING_H_
