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
// Interface used to add support for Raspberry Pi RP2350 microcontrollers.

// **********************************************************************************************************
// **** DEVICE_MODULE must be set to reflect the filename of this source file before including logging.h ****
// **********************************************************************************************************
#define DEVICE_MODULE "devices/rp2350.cpp"
#include "logging.h"
#include <stdlib.h>
#include <string.h>
#include "rp2350.h"
#include "mri_platform.h"
#include "cpu_core.h"
#include "LocalFlashing.h"
#include <rp2350-LocalFlashing.h>


// RP2350 FLASH memory location.
static const uint32_t FLASH_START_ADDRESS = 0x10000000;
static const uint32_t FLASH_MAX_SIZE = 32 * 1024 * 1024;
static const uint32_t FLASH_END_ADDRESS = FLASH_START_ADDRESS + FLASH_MAX_SIZE;
static const uint32_t FLASH_PAGE_SIZE = 4096;

// RP2350 SRAM memory location.
static const uint32_t RAM_START_ADDRESS = 0x20000000;
static const uint32_t RAM_SIZE = 520*1024;
static const uint32_t RAM_END_ADDRESS = RAM_START_ADDRESS + RAM_SIZE;

// RP2350 object caches pointers to the ROM based FLASH related routines in the object context.
struct RP2350Object
{
    LocalFlashingConfig localFlashingConfig;
    uint32_t            indicesAddress;
    uint32_t            freeIndex;
    uint32_t            writeIndex;
    uint32_t            ramWrite;
    uint32_t            ramRead;
};


// Forward function declarations.
static void disableDmaChannels(RP2350Object* pObject, CpuCore* pCore);
static uint32_t getDmaChannelCount(RP2350Object* pObject, CpuCore* pCore);
static void disableDmaChannel(RP2350Object* pObject, CpuCore* pCore, uint32_t i);
static void resetSecureAccessToSRAM(RP2350Object* pObject, CpuCore* pCore);
static bool checkDebuggerAccessToAccessCtrl(RP2350Object* pObject, CpuCore* pCore);
static void resetAccessControlPermissions(RP2350Object* pObject, CpuCore* pCore);
static void switchTargetToSecureState(RP2350Object* pObject, CpuCore* pCore);
static bool loadLocalFlashingCodeOnDevice(RP2350Object* pObject, CpuCore* pCore);
static bool startLocalFlashingCodeOnDevice(RP2350Object* pObject, CpuCore* pCore);
static bool isAddressAndLength4kAligned(uint32_t address, uint32_t length);
static bool is4kAligned(uint32_t value);
static bool isAttemptingToFlashInvalidAddress(uint32_t address, uint32_t length);
static uint32_t freeCompletedQueueEntries(RP2350Object* pObject, CpuCore* pCore);
static bool readQueueEntry(RP2350Object* pObject, CpuCore* pCore, uint32_t index, LocalFlashingQueueEntry* pOut);
static bool writeQueueEntry(RP2350Object* pObject, CpuCore* pCore, uint32_t index, const LocalFlashingQueueEntry* pIn);
static void freeRamFromEntry(RP2350Object* pObject, LocalFlashingQueueEntry* pEntry);
static bool addEntryToQueue(RP2350Object* pObject, CpuCore* pCore, const LocalFlashingQueueEntry* pEntry);
static bool waitForFreeQueueEntry(RP2350Object* pObject, CpuCore* pCore);
static bool readQueueIndices(RP2350Object* pObject, CpuCore* pCore, LocalFlashingQueueIndices* pOut);
static bool writeWriteIndex(RP2350Object* pObject, CpuCore* pCore, uint32_t writeIndex);
static uint32_t freeCompletedQueueEntriesToFreeNeededRam(RP2350Object* pObject, CpuCore* pCore, uint32_t byteCount);
static bool canAllocateTargetRam(RP2350Object* pObject, uint32_t bytesNeeded);
static uint32_t allocateTargetRam(RP2350Object* pObject, uint32_t byteCount);


// mri-swd will call this function on each of the devices listed in g_supportedDevices until a non-NULL response
// is encountered, indicating that the device attached to the debugger is supported by that element.
//
// pCore is a pointer to the CpuCore object used for interfacing to the core.
//
// The returned pointer can point to anything that the device specific code wants to access on subsequent calls to
// other routines in this function table. It can even be something malloc()ed. Calling the free() function from this
// table can then be used to free it. Should return NULL if the connected target isn't recognized as this type of
// device.
static DeviceObject* rp2350Detect(CpuCore* pCore)
{
    if (pCore->getTarget()->getTarget() != SWD::RP2350)
    {
        return NULL;
    }

    RP2350Object* pObject = (RP2350Object*)malloc(sizeof(*pObject));
    if (!pObject)
    {
        return NULL;
    }

    memset(pObject, 0, sizeof(*pObject));
    return pObject;
}


// Can be used by the device specific code to free the object returned from detect().
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
static void rp2350Free(DeviceObject* pvObject, CpuCore* pCore)
{
    free(pvObject);
}


// Gets friendly name for this device.
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
//
// Returns a \0 terminated string indicating this device's name.
static const char* rp2350GetName(DeviceObject* pvObject, CpuCore* pCore)
{
    return "RP2350";
}

// The SWD interface will be set to this frequency from the possibly lower rate used during SWD::init().
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
//
// Returns the maximum SWCLK frequency supported by this device.
static uint32_t rp2350MaximumSWDClockFrequency(DeviceObject* pvObject, CpuCore* pCore)
{
    // UNDONE: Should support up to 50MHz according the RP2350 datasheet.
    // The RP2350 SWD DP can support clock rates up to 20MHz.
    return 20000000;
}


// Called at the beginning of the FLASH programming process. The device specific code can perform whatever
// initialization it needs here before the flashErase() and flashProgram() routines are called.
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
//
// Returns true if successful and false otherwise.
static bool rp2350FlashBegin(DeviceObject* pvObject, CpuCore* pCore)
{
    assert ( pvObject );
    RP2350Object* pObject = (RP2350Object*)pvObject;
    disableDmaChannels(pObject, pCore);
    resetSecureAccessToSRAM(pObject, pCore);
    switchTargetToSecureState(pObject, pCore);
    if (!loadLocalFlashingCodeOnDevice(pObject, pCore))
    {
        return false;
    }
    return startLocalFlashingCodeOnDevice(pObject, pCore);
}

static void disableDmaChannels(RP2350Object* pObject, CpuCore* pCore)
{
    uint32_t channelCount = getDmaChannelCount(pObject, pCore);
    for (uint32_t i = 0 ; i < channelCount ; i++)
    {
        disableDmaChannel(pObject, pCore, i);
    }
}

static uint32_t getDmaChannelCount(RP2350Object* pObject, CpuCore* pCore)
{
    const uint32_t DMA_N_CHANNELS_Address = 0x50000468;
    uint32_t channelCount = 0;
    uint32_t bytesRead = pCore->readMemory(DMA_N_CHANNELS_Address, &channelCount, sizeof(channelCount), SWD::TRANSFER_32BIT);
    if (bytesRead != sizeof(channelCount) || channelCount == 0)
    {
        logError("Failed to read RP2350 DMA N_CHANNELS register.");
        return 0;
    }
    return channelCount;
}

static void disableDmaChannel(RP2350Object* pObject, CpuCore* pCore, uint32_t i)
{
    const uint32_t DMA_BASE_Address = 0x50000000;
    // The registers (including all of the aliases) for a channel take up this many bytes.
    const uint32_t DMA_CHANNEL_Size = 0x40;
    const uint32_t DMA_CHANNEL_CTRL_TRIG_Offset = 0xC;
    uint32_t channelControlRegisterAddress = DMA_BASE_Address + i * DMA_CHANNEL_Size + DMA_CHANNEL_CTRL_TRIG_Offset;
    const uint32_t channelControlRegisterValue = 0x00000000;

    // Just clear all of the bits in the channel's control register including the enable bit as none of the DMA
    // channel settings are valid anymore after loading in new firmware.
    uint32_t bytesWritten = pCore->writeMemory(channelControlRegisterAddress, &channelControlRegisterValue, sizeof(channelControlRegisterValue), SWD::TRANSFER_32BIT);
    if (bytesWritten != sizeof(channelControlRegisterValue))
    {
        logErrorF("Failed to disable DMA channel %lu.\n", i);
    }
}

static void resetSecureAccessToSRAM(RP2350Object* pObject, CpuCore* pCore)
{
    bool isAccessible = checkDebuggerAccessToAccessCtrl(pObject, pCore);
    if (!isAccessible)
    {
        logError("Debugger can't reset ACCESSCTRL permissions. FLASH programming might fail.");
        return;
    }
    resetAccessControlPermissions(pObject, pCore);
}

static bool checkDebuggerAccessToAccessCtrl(RP2350Object* pObject, CpuCore* pCore)
{
    const uint32_t ACCESSCTRL_LOCK_Address = 0x40060000;
    uint32_t ACCESSCTRL_LOCK_Value = 0x00000000;
    uint32_t bytesRead = pCore->readMemory(ACCESSCTRL_LOCK_Address, &ACCESSCTRL_LOCK_Value, sizeof(ACCESSCTRL_LOCK_Value), SWD::TRANSFER_32BIT);
    if (bytesRead != sizeof(ACCESSCTRL_LOCK_Value))
    {
        logError("Failed read from RP2350 ACCESSCTRL_LOCK register.");
        return false;
    }

    const uint32_t ACCESSCTRL_LOCK_DEBUG_Bit = 1 << 3;
    return (ACCESSCTRL_LOCK_Value & ACCESSCTRL_LOCK_DEBUG_Bit) == 0;
}

static void resetAccessControlPermissions(RP2350Object* pObject, CpuCore* pCore)
{
    const uint32_t ACCESSCTRL_CFGRESET_Address = 0x40060008;
    const uint32_t ACCESSCTRL_PASSWORD = 0xacce0000;
    const uint32_t ACCESSCTRL_CFGRESET_RESET = 1;
    const uint32_t ACCESSCTRL_CFGRESET_Value = ACCESSCTRL_PASSWORD | ACCESSCTRL_CFGRESET_RESET;
    uint32_t bytesWritten = pCore->writeMemory(ACCESSCTRL_CFGRESET_Address, &ACCESSCTRL_CFGRESET_Value, sizeof(ACCESSCTRL_CFGRESET_Value), SWD::TRANSFER_32BIT);
    if (bytesWritten != sizeof(ACCESSCTRL_CFGRESET_Value))
    {
        logError("Failed write to RP2350 ACCESSCTRL_CFGRESET register.");
    }
}

static void switchTargetToSecureState(RP2350Object* pObject, CpuCore* pCore)
{
    const uint32_t DSCSR_Address = 0xE000EE08;
    const uint32_t DSCSR_CDSKEY_Bit = 1 << 17;
    const uint32_t DSCSR_CDS_Bit = 1 << 16;
    uint32_t DSCSR_Value = 0;
    uint32_t bytesRead = pCore->readMemory(DSCSR_Address, &DSCSR_Value, sizeof(DSCSR_Value), SWD::TRANSFER_32BIT);
    if (bytesRead != sizeof(DSCSR_Value))
    {
        logError("Failed read from RP2350 DSCSR register.");
    }

    if (DSCSR_Value & DSCSR_CDS_Bit)
    {
        // Return early if the target is already running in the secure state.
        return;
    }

    DSCSR_Value = (DSCSR_Value & ~DSCSR_CDSKEY_Bit) | DSCSR_CDS_Bit;
    uint32_t bytesWritten = pCore->writeMemory(DSCSR_Address, &DSCSR_Value, sizeof(DSCSR_Value), SWD::TRANSFER_32BIT);
    if (bytesWritten != sizeof(DSCSR_Value))
    {
        logError("Failed write to RP2350 DSCSR register.");
    }

    // Let the user know if it didn't get set properly.
    DSCSR_Value = 0;
    bytesRead = pCore->readMemory(DSCSR_Address, &DSCSR_Value, sizeof(DSCSR_Value), SWD::TRANSFER_32BIT);
    if (bytesRead != sizeof(DSCSR_Value))
    {
        logError("Failed read from RP2350 DSCSR register for verification.");
    }
    if ((DSCSR_Value & DSCSR_CDS_Bit) == 0)
    {
        logErrorF("Failed to set target to secure state. FLASH programming might fail. DSCSR=0x%08X", DSCSR_Value);
    }
}

static bool loadLocalFlashingCodeOnDevice(RP2350Object* pObject, CpuCore* pCore)
{
    assert ( sizeof(g_rp2350_LocalFlashing) >= sizeof(pObject->localFlashingConfig) );
    memcpy(&pObject->localFlashingConfig, g_rp2350_LocalFlashing, sizeof(pObject->localFlashingConfig));

    uint32_t bytesWritten = pCore->writeMemory(pObject->localFlashingConfig.loadAddress, g_rp2350_LocalFlashing, sizeof(g_rp2350_LocalFlashing), SWD::TRANSFER_8BIT);
    if (bytesWritten != sizeof(g_rp2350_LocalFlashing))
    {
        logErrorF("Failed to write %lu bytes of local flashing code to target at address 0x%08X.",
            sizeof(g_rp2350_LocalFlashing), pObject->localFlashingConfig.loadAddress);
        return false;
    }
    pObject->indicesAddress = pObject->localFlashingConfig.loadAddress + offsetof(LocalFlashingConfig, queueIndices);
    pObject->writeIndex = pObject->localFlashingConfig.queueIndices.writeIndex;
    pObject->freeIndex = pObject->writeIndex;
    pObject->ramWrite = 0;
    pObject->ramRead = 0;
    return true;
}

static bool startLocalFlashingCodeOnDevice(RP2350Object* pObject, CpuCore* pCore)
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


// GDB would like the FLASH at the specified address range to be erased in preparation for programming.
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
// addressStart is the address at the beginning of the range to be erased. (inclusive)
// length is the number of bytes starting at address to be erased.
//
// Returns true if successful and false otherwise.
static bool rp2350FlashErase(DeviceObject* pvObject, CpuCore* pCore, uint32_t addressStart, uint32_t length)
{
    RP2350Object* pObject = (RP2350Object*)pvObject;
    assert ( pObject && pCore );

    if (!isAddressAndLength4kAligned(addressStart, length))
    {
        logErrorF("Address (0x%08lX) and length (0x%lX) are not both 4k aligned.", addressStart, length);
        return false;
    }
    if (isAttemptingToFlashInvalidAddress(addressStart, length))
    {
        logErrorF("Erase starting (0x%08lX) and ending (0x%08lX) addresses don't both point to FLASH.", addressStart, addressStart+length);
        return false;
    }

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

static bool isAddressAndLength4kAligned(uint32_t address, uint32_t length)
{
    return (is4kAligned(address) && is4kAligned(length));
}

static bool is4kAligned(uint32_t value)
{
    return (value & (4096-1)) == 0;
}

static bool isAttemptingToFlashInvalidAddress(uint32_t address, uint32_t length)
{
    uint32_t endAddress = address + length;
    if (address < FLASH_START_ADDRESS || endAddress > FLASH_END_ADDRESS)
    {
        return true;
    }
    return false;
}

static uint32_t freeCompletedQueueEntries(RP2350Object* pObject, CpuCore* pCore)
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

static bool readQueueEntry(RP2350Object* pObject, CpuCore* pCore, uint32_t index, LocalFlashingQueueEntry* pOut)
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

static bool writeQueueEntry(RP2350Object* pObject, CpuCore* pCore, uint32_t index, const LocalFlashingQueueEntry* pIn)
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

static void freeRamFromEntry(RP2350Object* pObject, LocalFlashingQueueEntry* pEntry)
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

static bool addEntryToQueue(RP2350Object* pObject, CpuCore* pCore, const LocalFlashingQueueEntry* pEntry)
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

static bool waitForFreeQueueEntry(RP2350Object* pObject, CpuCore* pCore)
{
    absolute_time_t endTime = make_timeout_time_ms(EXEC_RP2040_BOOT_ROM_FUNC_TIMEOUT_MS);
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

static bool readQueueIndices(RP2350Object* pObject, CpuCore* pCore, LocalFlashingQueueIndices* pOut)
{
    uint32_t bytesRead = pCore->readMemory(pObject->indicesAddress, pOut, sizeof(*pOut), SWD::TRANSFER_32BIT);
    if (bytesRead != sizeof(*pOut))
    {
        logError("Failed to read LocalFlashingQueueIndices from target.");
        return false;
    }
    return true;
}

static bool writeWriteIndex(RP2350Object* pObject, CpuCore* pCore, uint32_t writeIndex)
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


// GDB would like the FLASH at the specified address to be programmed with the supplied data.
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
// addressStart is the address at the beginning of the range to be programmed.
// pBuffer is the address of the data to be programmed into the specified FLASH location.
// bufferSize is the number of bytes in pBuffer to be programmed into FLASH at the specified location.
//
// Returns true if successful and false otherwise.
static bool rp2350FlashProgram(DeviceObject* pvObject, CpuCore* pCore, uint32_t addressStart, const void* pBuffer, size_t bufferSize)
{
    RP2350Object* pObject = (RP2350Object*)pvObject;
    assert ( pObject && pCore );

    if (isAttemptingToFlashInvalidAddress(addressStart, bufferSize))
    {
        logErrorF("Write starting (0x%08lX) and ending (0x%08lX) addresses don't both point to FLASH.", addressStart, addressStart+bufferSize);
        return false;
    }

    uint32_t errorCode = freeCompletedQueueEntriesToFreeNeededRam(pObject, pCore, bufferSize);
    if (errorCode != 0)
    {
        logErrorF("Failed call to freeCompletedQueueEntriesToFreeNeededRam(pCore, %lu) with error %lu", bufferSize, errorCode);
        return false;
    }
    uint32_t targetRamAddress = allocateTargetRam(pObject, bufferSize);

    // Make sure that the buffer is 32-bit aligned.
    const uint32_t alignment = sizeof(uint32_t);
    uint32_t alignedBuffer[PACKET_SIZE / sizeof(uint32_t)];
    uint32_t alignedSize = (bufferSize + alignment - 1) & ~(alignment - 1);
    assert ( alignedSize <= sizeof(alignedBuffer) );
    memcpy(alignedBuffer, pBuffer, bufferSize);

    // Copy buffer from debugger to target RAM.
    uint32_t bytesCopied = pCore->writeMemory(targetRamAddress, alignedBuffer, alignedSize, SWD::TRANSFER_32BIT);
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

static uint32_t freeCompletedQueueEntriesToFreeNeededRam(RP2350Object* pObject, CpuCore* pCore, uint32_t byteCount)
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

static bool canAllocateTargetRam(RP2350Object* pObject, uint32_t bytesNeeded)
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

static uint32_t allocateTargetRam(RP2350Object* pObject, uint32_t byteCount)
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


// Called at the end of the FLASH programming process. The device specific code can perform whatever
// cleanup it needs here after all the flashErase() and flashProgram() call have been made.
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
//
// Returns true if successful and false otherwise.
static bool rp2350FlashEnd(DeviceObject* pvObject, CpuCore* pCore)
{
    RP2350Object* pObject = (RP2350Object*)pvObject;
    assert ( pObject && pCore );

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
    if (!waitForCodeToHalt(&registers, EXEC_RP2040_BOOT_ROM_FUNC_TIMEOUT_MS))
    {
        logError("Failed waiting for target to finish FLASHing.");
        return false;
    }

    // UNDONE: Just doing for assert.
    freeCompletedQueueEntries(pObject, pCore);
    assert ( pObject->ramRead == pObject->ramWrite && pObject->freeIndex == pObject->writeIndex );

    return true;
}


// The FLASH region is set to the maximum 16MB supported by the RP2350 and might not reflect the actual size of the
// FLASH part attached to the CPU.
static const DeviceMemoryRegion g_memoryRegions[] =
{
    { .address = 0x00000000, .length = 0x4000, .blockSize = 0, .type = DEVICE_MEMORY_ROM },
    { .address = FLASH_START_ADDRESS, .length = FLASH_MAX_SIZE, .blockSize = FLASH_PAGE_SIZE, .type = DEVICE_MEMORY_FLASH },
    { .address = RAM_START_ADDRESS, .length = RAM_SIZE, .blockSize = 0, .type = DEVICE_MEMORY_RAM }
};
static const DeviceMemoryLayout g_memoryLayout =
{
    .pRegions = g_memoryRegions,
    .regionCount = count_of(g_memoryRegions)
};


// Returns the memory layout of the ROM, FLASH, and RAM regions of the device(s). This will be used to provide the
// memory layout XML to GDB.
//
// If it isn't possible to figure out the exact memory layout for a device then set this function pointer to the
// deviceDefaultMemoryLayout() function declared below.
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
//
// Returns a pointer to the memory layout description for this device. Can't be NULL.
const DeviceMemoryLayout* rp2350GetMemoryLayout(DeviceObject* pvObject, CpuCore* pCore)
{
    return &g_memoryLayout;
}

// If the device supports more than 1 core then it can populate the supplied pTargetArray.
//
// This function pointer can be set to the deviceNoAdditionalTarget() function declared below for single core
// devices.
//
// An example implementation where an additional target is added can be found in rp2350.cpp to expose the second
// core on the RP2350.
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
// pCoreArray is a pointer to an array of CpuCore objects to be initialized for additional cores on this device.
// coreArrayLength is the maximum number of elements in pCoreArray that can be set by this function.
// pCoreCount is the number of additional pCoreArray elements that were actually initialized by this function.
//
// Returns true if successful and false otherwise.
bool rp2350GetAdditionalTargets(DeviceObject* pvObject, CpuCore* pCore, CpuCore* pCoreArray, size_t coreArrayLength, size_t* pCoreCount)
{
    // UNDONE: Add multi-core support through multiple APs to SWD.
    *pCoreCount = 0;
    return true;

#ifdef UNDONE
    assert ( coreArrayLength >= 1 );

    SWD* pCoreBus = pCore->getTarget()->getSwdBus();
    if (!pCoreBus->selectSwdTarget(SWD::RP2350_CORE1))
    {
        logError("Failed to find the second core on SWD bus.");
        return false;
    }
    // Note: The index passed into init() is 1 (since this is Core1), not 0 like the array index.
    if (!pCoreArray[0].init(pCore, 0+1))
    {
        logError("Failed to init the second core.");
        return false;
    }

    *pCoreCount = 1;
    return true;
#endif // UNDONE
}


// The function pointer table for this device. A pointer to it can be found in g_supportedDevices (in devices.cpp).
DeviceFunctionTable g_rp2350DeviceSupport =
{
    .detect = rp2350Detect,
    .free = rp2350Free,
    .getName = rp2350GetName,
    .hasFpu = deviceHasFpu,
    .getMaximumSWDClockFrequency = rp2350MaximumSWDClockFrequency,
    .flashBegin = rp2350FlashBegin,
    .flashErase = rp2350FlashErase,
    .flashProgram = rp2350FlashProgram,
    .flashEnd = rp2350FlashEnd,
    .getMemoryLayout = rp2350GetMemoryLayout,
    .getAdditionalTargets = rp2350GetAdditionalTargets,
    .handleMonitorCommand = NULL
};
