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
typedef struct LocalFlashingObject RP2350Object;


// Forward function declarations.
static void disableDmaChannels(RP2350Object* pObject, CpuCore* pCore);
static uint32_t getDmaChannelCount(RP2350Object* pObject, CpuCore* pCore);
static void disableDmaChannel(RP2350Object* pObject, CpuCore* pCore, uint32_t i);
static void resetSecureAccessToSRAM(RP2350Object* pObject, CpuCore* pCore);
static bool checkDebuggerAccessToAccessCtrl(RP2350Object* pObject, CpuCore* pCore);
static void resetAccessControlPermissions(RP2350Object* pObject, CpuCore* pCore);
static void switchTargetToSecureState(RP2350Object* pObject, CpuCore* pCore);
static bool isAddressAndLength4kAligned(uint32_t address, uint32_t length);
static bool is4kAligned(uint32_t value);
static bool isAttemptingToFlashInvalidAddress(uint32_t address, uint32_t length);


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
    return localFlashingInit(pObject, pCore, g_rp2350_LocalFlashing, sizeof(g_rp2350_LocalFlashing));
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

    return localFlashingQueueEraseOperation(pObject, pCore, addressStart, length);
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

    // Make sure that the buffer is 32-bit aligned.
    const uint32_t alignment = sizeof(uint32_t);
    uint32_t alignedBuffer[PACKET_SIZE / sizeof(uint32_t)];
    uint32_t alignedSize = (bufferSize + alignment - 1) & ~(alignment - 1);
    assert ( alignedSize <= sizeof(alignedBuffer) );
    memcpy(alignedBuffer, pBuffer, bufferSize);

    return localFlashingQueueProgramOperation(pObject, pCore, addressStart, alignedBuffer, bufferSize, alignedSize);
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

    return localFlashingUninit(pObject, pCore);
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
    assert ( coreArrayLength >= 1 );

    const uint32_t RP2350_CORE1_AP = 4;
    SWD* pCoreBus = pCore->getTarget()->getSwdBus();
    if (!pCoreBus->checkAP(RP2350_CORE1_AP))
    {
        logError("Failed to find the second core on SWD bus.");
        return false;
    }
    // Note: The index passed into init() is 1 (since this is Core1), not 0 like the array index.
    if (!pCoreArray[0].init(pCore, 0+1, CpuCore::AP_CORE))
    {
        logError("Failed to init the second core.");
        return false;
    }

    *pCoreCount = 1;
    return true;
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
