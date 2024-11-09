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
// Functions which queue up FLASH programming operation to be executed on target locally.
// This allows the FLASH erasing and programming to occur in parallel with GDB data transfer.

// **********************************************************************************************************
// **** DEVICE_MODULE must be set to reflect the filename of this source file before including logging.h ****
// **********************************************************************************************************
#define DEVICE_MODULE "devices/LocalFlashing.cpp"
#include "logging.h"
#include "LocalFlashing.h"


// Forward function declarations.
static bool loadLocalFlashingCodeOnDevice(LocalFlashingObject* pObject, CpuCore* pCore, const uint8_t* pTargetCode, size_t targetCodeSize);
static bool startLocalFlashingCodeOnDevice(LocalFlashingObject* pObject, CpuCore* pCore);
static bool startLocalFlashingCodeOnDevice(LocalFlashingObject* pObject, CpuCore* pCore);
static uint32_t freeCompletedQueueEntries(LocalFlashingObject* pObject, CpuCore* pCore);
static bool readQueueEntry(LocalFlashingObject* pObject, CpuCore* pCore, uint32_t index, LocalFlashingQueueEntry* pOut);
static bool writeQueueEntry(LocalFlashingObject* pObject, CpuCore* pCore, uint32_t index, const LocalFlashingQueueEntry* pIn);
static void freeRamFromEntry(LocalFlashingObject* pObject, LocalFlashingQueueEntry* pEntry);
static bool addEntryToQueue(LocalFlashingObject* pObject, CpuCore* pCore, const LocalFlashingQueueEntry* pEntry);
static bool waitForFreeQueueEntry(LocalFlashingObject* pObject, CpuCore* pCore);
static bool readQueueIndices(LocalFlashingObject* pObject, CpuCore* pCore, LocalFlashingQueueIndices* pOut);
static bool writeWriteIndex(LocalFlashingObject* pObject, CpuCore* pCore, uint32_t writeIndex);
static uint32_t freeCompletedQueueEntriesToFreeNeededRam(LocalFlashingObject* pObject, CpuCore* pCore, uint32_t byteCount);
static bool canAllocateTargetRam(LocalFlashingObject* pObject, uint32_t bytesNeeded);
static uint32_t allocateTargetRam(LocalFlashingObject* pObject, uint32_t byteCount);


bool localFlashingInit(LocalFlashingObject* pObject, CpuCore* pCore, const uint8_t* pTargetCode, size_t targetCodeSize)
{
    if (!loadLocalFlashingCodeOnDevice(pObject, pCore, pTargetCode, targetCodeSize))
    {
        return false;
    }
    return startLocalFlashingCodeOnDevice(pObject, pCore);
}

static bool loadLocalFlashingCodeOnDevice(LocalFlashingObject* pObject, CpuCore* pCore, const uint8_t* pTargetCode, size_t targetCodeSize)
{
    assert ( targetCodeSize >= sizeof(pObject->localFlashingConfig) );
    memcpy(&pObject->localFlashingConfig, pTargetCode, sizeof(pObject->localFlashingConfig));

    uint32_t bytesWritten = pCore->writeMemory(pObject->localFlashingConfig.loadAddress, pTargetCode, targetCodeSize, SWD::TRANSFER_8BIT);
    if (bytesWritten != targetCodeSize)
    {
        logErrorF("Failed to write %lu bytes of local flashing code to target at address 0x%08X.",
            targetCodeSize, pObject->localFlashingConfig.loadAddress);
        return false;
    }
    pObject->indicesAddress = pObject->localFlashingConfig.loadAddress + offsetof(LocalFlashingConfig, queueIndices);
    pObject->writeIndex = pObject->localFlashingConfig.queueIndices.writeIndex;
    pObject->freeIndex = pObject->writeIndex;
    pObject->ramWrite = 0;
    pObject->ramRead = 0;
    return true;
}

static bool startLocalFlashingCodeOnDevice(LocalFlashingObject* pObject, CpuCore* pCore)
{
    // Set the CPU registers required for starting the FLASHing routine on the device.
    CortexM_Registers registers;
    memset(&registers, 0, sizeof(registers));
    registers.SP = pObject->localFlashingConfig.stackAddress;
    registers.LR = pObject->localFlashingConfig.routineAddress;
    registers.PC = pObject->localFlashingConfig.routineAddress;

    if (!startCodeOnDevice(&registers))
    {
        return false;
    }
    return true;
}


bool localFlashingQueueEraseOperation(LocalFlashingObject* pObject, CpuCore* pCore, uint32_t addressStart, uint32_t length)
{
    uint32_t errorCode = freeCompletedQueueEntries(pObject, pCore);
    if (errorCode != 0)
    {
        return false;
    }

    // Add FLASH erase operation to device's queue.
    LocalFlashingQueueEntry entry =
    {
        .op = LOCAL_FLASHING_OP_ERASE,
        .errorCode = 0,
        .flashAddress = addressStart,
        .ramAddress = LOCAL_FLASHING_RAM_UNUSED,
        .size = length,
        .alloc = 0
    };
    if (!addEntryToQueue(pObject, pCore, &entry))
    {
        return false;
    }

    return true;
}

static uint32_t freeCompletedQueueEntries(LocalFlashingObject* pObject, CpuCore* pCore)
{
    uint32_t errorCode = 0;
    while (pObject->freeIndex != pObject->writeIndex)
    {
        LocalFlashingQueueEntry entry;
        if (!readQueueEntry(pObject, pCore, pObject->freeIndex, &entry))
        {
            logError("Failed call to readQueueEntry().");
            return 0xFFFFFFFF;
        }
        if (entry.op != LOCAL_FLASHING_OP_COMPLETE)
        {
            // We have encountered the first entry that hasn't been completed yet so return.
            break;
        }

        // Remember the first error code we encounter.
        if (errorCode == 0 && entry.errorCode != 0)
        {
            errorCode = entry.errorCode;
        }

        // Free any RAM associated with this operation as it is no longer in use.
        freeRamFromEntry(pObject, &entry);

        // Mark the completed entry as free now that any associated RAM has been freed.
        entry.op = LOCAL_FLASHING_OP_FREE;
        if (!writeQueueEntry(pObject, pCore, pObject->freeIndex, &entry))
        {
            logError("Failed call to writeQueueEntry().");
            return 0xFFFFFFFF;
        }
        pObject->freeIndex = (pObject->freeIndex + 1) % pObject->localFlashingConfig.queueLength;
    }
    return errorCode;
}

static bool readQueueEntry(LocalFlashingObject* pObject, CpuCore* pCore, uint32_t index, LocalFlashingQueueEntry* pOut)
{
    uint32_t entryAddress = pObject->localFlashingConfig.queueAddress + index * sizeof(*pOut);
    uint32_t bytesRead = pCore->readMemory(entryAddress, pOut, sizeof(*pOut), SWD::TRANSFER_32BIT);
    if (bytesRead != sizeof(*pOut))
    {
        logErrorF("Failed to read LocalFlashingQueueEntry element %lu from target.", index);
        return false;
    }
    return true;
}

static bool writeQueueEntry(LocalFlashingObject* pObject, CpuCore* pCore, uint32_t index, const LocalFlashingQueueEntry* pIn)
{
    uint32_t entryAddress = pObject->localFlashingConfig.queueAddress + index * sizeof(*pIn);
    uint32_t bytesWritten = pCore->writeMemory(entryAddress, pIn, sizeof(*pIn), SWD::TRANSFER_32BIT);
    if (bytesWritten != sizeof(*pIn))
    {
        logErrorF("Failed to write LocalFlashingQueueEntry element %lu to target.", index);
        return false;
    }
    return true;
}

static void freeRamFromEntry(LocalFlashingObject* pObject, LocalFlashingQueueEntry* pEntry)
{
    if (pEntry->alloc == 0)
    {
        return;
    }

    uint32_t endOffset = pEntry->ramAddress + pEntry->alloc - pObject->localFlashingConfig.ramAddress;
    if (endOffset >= pObject->localFlashingConfig.ramLength)
    {
        endOffset = 0;
    }
    pObject->ramRead = endOffset;
    pEntry->alloc = 0;
}

static bool addEntryToQueue(LocalFlashingObject* pObject, CpuCore* pCore, const LocalFlashingQueueEntry* pEntry)
{
    if (!waitForFreeQueueEntry(pObject, pCore))
    {
        return false;
    }
    if (!writeQueueEntry(pObject, pCore, pObject->writeIndex, pEntry))
    {
        return false;
    }
    pObject->writeIndex = (pObject->writeIndex + 1) % pObject->localFlashingConfig.queueLength;
    if (!writeWriteIndex(pObject, pCore, pObject->writeIndex))
    {
        return false;
    }
    return true;
}

static bool waitForFreeQueueEntry(LocalFlashingObject* pObject, CpuCore* pCore)
{
    absolute_time_t endTime = make_timeout_time_ms(LOCAL_FLASHING_OP_TIMEOUT);
    do
    {
        LocalFlashingQueueIndices indices;
        if (!readQueueIndices(pObject, pCore, &indices))
        {
            return false;
        }
        if (indices.writeIndex != pObject->writeIndex)
        {
            logErrorF("Expected writeIndex=%lu. Actual=%lu", pObject->writeIndex, indices.writeIndex);
            return false;
        }
        uint32_t nextIndex = (pObject->writeIndex + 1) % pObject->localFlashingConfig.queueLength;
        if (nextIndex != indices.readIndex)
        {
            // Queue isn't full so we have at least one free entry.
            return true;
        }
    } while (absolute_time_diff_us(get_absolute_time(), endTime) > 0);

    return false;
}

static bool readQueueIndices(LocalFlashingObject* pObject, CpuCore* pCore, LocalFlashingQueueIndices* pOut)
{
    uint32_t bytesRead = pCore->readMemory(pObject->indicesAddress, pOut, sizeof(*pOut), SWD::TRANSFER_32BIT);
    if (bytesRead != sizeof(*pOut))
    {
        logError("Failed to read LocalFlashingQueueIndices from target.");
        return false;
    }
    return true;
}

static bool writeWriteIndex(LocalFlashingObject* pObject, CpuCore* pCore, uint32_t writeIndex)
{
    uint32_t address = pObject->indicesAddress + offsetof(LocalFlashingQueueIndices, writeIndex);
    uint32_t bytesWritten = pCore->writeMemory(address, &writeIndex, sizeof(writeIndex), SWD::TRANSFER_32BIT);
    if (bytesWritten != sizeof(writeIndex))
    {
        logErrorF("Failed to write %lu to LocalFlashingQueueIndices.writeIndex to target.", writeIndex);
        return false;
    }
    return true;
}


bool localFlashingQueueProgramOperation(LocalFlashingObject* pObject, CpuCore* pCore, uint32_t addressStart, const void* pBuffer, size_t bufferSize, size_t alignedSize)
{
    uint32_t errorCode = freeCompletedQueueEntriesToFreeNeededRam(pObject, pCore, bufferSize);
    if (errorCode != 0)
    {
        logErrorF("Failed call to freeCompletedQueueEntriesToFreeNeededRam(pCore, %lu) with error %lu", bufferSize, errorCode);
        return false;
    }
    uint32_t targetRamAddress = allocateTargetRam(pObject, bufferSize);

    // Copy buffer from debugger to target RAM.
    uint32_t bytesCopied = pCore->writeMemory(targetRamAddress, pBuffer, alignedSize, SWD::TRANSFER_32BIT);
    if (bytesCopied != alignedSize)
    {
        logErrorF("Failed to copy %lu bytes to target RAM @ 0x%08lX.", alignedSize, targetRamAddress);
        return false;
    }

    // Add FLASH write operation to device's queue.
    LocalFlashingQueueEntry entry =
    {
        .op = LOCAL_FLASHING_OP_WRITE,
        .errorCode = 0,
        .flashAddress = addressStart,
        .ramAddress = targetRamAddress,
        .size = bufferSize,
        .alloc = alignedSize
    };
    if (!addEntryToQueue(pObject, pCore, &entry))
    {
        return false;
    }

    return true;
}

static uint32_t freeCompletedQueueEntriesToFreeNeededRam(LocalFlashingObject* pObject, CpuCore* pCore, uint32_t byteCount)
{
    if (byteCount > pObject->localFlashingConfig.ramLength)
    {
        return 0xFFFFFFFF;
    }
    while (!canAllocateTargetRam(pObject, byteCount))
    {
        uint32_t result = freeCompletedQueueEntries(pObject, pCore);
        if (result != 0)
        {
            return result;
        }
    }

    return 0;
}

static bool canAllocateTargetRam(LocalFlashingObject* pObject, uint32_t bytesNeeded)
{
    // This check is a little more complicated than a usual circular buffer since the allocation needs to be contiguous,
    // so it can't split bytes across the end of the buffer.
    if (pObject->ramRead <= pObject->ramWrite)
    {
        uint32_t endSpace = pObject->localFlashingConfig.ramLength - pObject->ramWrite;
        uint32_t startSpace = pObject->ramRead;
        return (bytesNeeded <= endSpace || bytesNeeded < startSpace);
    }
    else
    {
        uint32_t freeSpace = pObject->ramRead - pObject->ramWrite;
        return (bytesNeeded < freeSpace);
    }
}

static uint32_t allocateTargetRam(LocalFlashingObject* pObject, uint32_t byteCount)
{
    if (pObject->ramRead <= pObject->ramWrite)
    {
        // Read pointer is still lagging behind write pointer.
        uint32_t endSpace = pObject->localFlashingConfig.ramLength - pObject->ramWrite;
        if (byteCount <= endSpace)
        {
            // Fits in free space at end of target RAM.
            uint32_t alloc = pObject->ramWrite;
            pObject->ramWrite = pObject->ramWrite + byteCount;
            if (pObject->ramWrite >= pObject->localFlashingConfig.ramLength)
            {
                pObject->ramWrite = 0;
            }
            return pObject->localFlashingConfig.ramAddress + alloc;
        }
        else
        {
            // Fits in free space at the very beginning of target RAM.
            uint32_t alloc = 0;
            pObject->ramWrite = byteCount;
            assert ( pObject->ramWrite < pObject->ramRead );
            return pObject->localFlashingConfig.ramAddress + alloc;
        }
    }
    else
    {
        // Write pointer has wrapped around and is now behind read pointer.
        uint32_t alloc = pObject->ramWrite;
        pObject->ramWrite = pObject->ramWrite + byteCount;
        assert ( pObject->ramWrite < pObject->ramRead );
        return pObject->localFlashingConfig.ramAddress + alloc;
    }
}


bool localFlashingUninit(LocalFlashingObject* pObject, CpuCore* pCore)
{
    uint32_t errorCode = freeCompletedQueueEntries(pObject, pCore);
    if (errorCode != 0)
    {
        logErrorF("Failed call to freeCompletedQueueEntries() with error %lu", errorCode);
        return false;
    }

    // Ask target to trigger breakpoint when it is done processing the queue.
    LocalFlashingQueueEntry entry =
    {
        .op = LOCAL_FLASHING_OP_BREAKPOINT,
        .errorCode = 0,
        .flashAddress = 0,
        .ramAddress = LOCAL_FLASHING_RAM_UNUSED,
        .size = 0,
        .alloc = 0
    };
    if (!addEntryToQueue(pObject, pCore, &entry))
    {
        return false;
    }

    // Wait for breakpoint to trigger.
    CortexM_Registers registers;
    if (!waitForCodeToHalt(&registers, LOCAL_FLASHING_OP_TIMEOUT))
    {
        logError("Failed waiting for target to finish FLASHing.");
        return false;
    }

    // UNDONE: Just doing for assert.
    freeCompletedQueueEntries(pObject, pCore);
    assert ( pObject->ramRead == pObject->ramWrite && pObject->freeIndex == pObject->writeIndex );

    return true;
}
