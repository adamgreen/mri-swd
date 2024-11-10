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
// Interface used to add support for Raspberry Pi RP2040 microcontrollers.

// **********************************************************************************************************
// **** DEVICE_MODULE must be set to reflect the filename of this source file before including logging.h ****
// **********************************************************************************************************
#define DEVICE_MODULE "devices/rp2040.cpp"
#include "logging.h"
#include <stdlib.h>
#include <string.h>
#include "rp2040.h"
#include "mri_platform.h"
#include "cpu_core.h"
#include "LocalFlashing.h"
#include <rp2040-LocalFlashing.h>


// RP2040 FLASH memory location.
static const uint32_t FLASH_START_ADDRESS = 0x10000000;
static const uint32_t FLASH_MAX_SIZE = 16 * 1024 * 1024;
static const uint32_t FLASH_END_ADDRESS = FLASH_START_ADDRESS + FLASH_MAX_SIZE;
static const uint32_t FLASH_PAGE_SIZE = 4096;

// RP2040 SRAM memory location.
static const uint32_t RAM_START_ADDRESS = 0x20000000;
static const uint32_t RAM_SIZE = 264*1024;
static const uint32_t RAM_END_ADDRESS = RAM_START_ADDRESS + RAM_SIZE;

// RP2040 object caches pointers to the ROM based FLASH related routines in the object context.
typedef struct LocalFlashingObject RP2040Object;


// Forward function declarations.
static void disableDmaChannels(RP2040Object* pObject, CpuCore* pCore);
static uint32_t getDmaChannelCount(RP2040Object* pObject, CpuCore* pCore);
static void disableDmaChannel(RP2040Object* pObject, CpuCore* pCore, uint32_t i);
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
static DeviceObject* rp2040Detect(CpuCore* pCore)
{
    if (pCore->getTarget()->getTarget() != SWD::RP2040_CORE0)
    {
        return NULL;
    }

    RP2040Object* pObject = (RP2040Object*)malloc(sizeof(*pObject));
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
static void rp2040Free(DeviceObject* pvObject, CpuCore* pCore)
{
    free(pvObject);
}


// Gets friendly name for this device.
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
//
// Returns a \0 terminated string indicating this device's name.
static const char* rp2040GetName(DeviceObject* pvObject, CpuCore* pCore)
{
    return "RP2040";
}

// The SWD interface will be set to this frequency from the possibly lower rate used during SWD::init().
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
//
// Returns the maximum SWCLK frequency supported by this device.
static uint32_t rp2040MaximumSWDClockFrequency(DeviceObject* pvObject, CpuCore* pCore)
{
    // The RP2040 SWD DP can support clock rates up to 24MHz.
    return 24000000;
}


// Called at the beginning of the FLASH programming process. The device specific code can perform whatever
// initialization it needs here before the flashErase() and flashProgram() routines are called.
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
//
// Returns true if successful and false otherwise.
static bool rp2040FlashBegin(DeviceObject* pvObject, CpuCore* pCore)
{
    assert ( pvObject );
    RP2040Object* pObject = (RP2040Object*)pvObject;
    disableDmaChannels(pObject, pCore);
    return localFlashingInit(pObject, pCore, g_rp2040_LocalFlashing, sizeof(g_rp2040_LocalFlashing));
}

static void disableDmaChannels(RP2040Object* pObject, CpuCore* pCore)
{
    uint32_t channelCount = getDmaChannelCount(pObject, pCore);
    for (uint32_t i = 0 ; i < channelCount ; i++)
    {
        disableDmaChannel(pObject, pCore, i);
    }
}

static uint32_t getDmaChannelCount(RP2040Object* pObject, CpuCore* pCore)
{
    const uint32_t DMA_N_CHANNELS_Address = 0x50000448;
    uint32_t channelCount = 0;
    uint32_t bytesRead = pCore->readMemory(DMA_N_CHANNELS_Address, &channelCount, sizeof(channelCount), SWD::TRANSFER_32BIT);
    if (bytesRead != sizeof(channelCount) || channelCount == 0)
    {
        logError("Failed to read RP2040 DMA N_CHANNELS register.");
        return 0;
    }
    return channelCount;
}

static void disableDmaChannel(RP2040Object* pObject, CpuCore* pCore, uint32_t i)
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


// GDB would like the FLASH at the specified address range to be erased in preparation for programming.
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
// addressStart is the address at the beginning of the range to be erased. (inclusive)
// length is the number of bytes starting at address to be erased.
//
// Returns true if successful and false otherwise.
static bool rp2040FlashErase(DeviceObject* pvObject, CpuCore* pCore, uint32_t addressStart, uint32_t length)
{
    RP2040Object* pObject = (RP2040Object*)pvObject;
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
static bool rp2040FlashProgram(DeviceObject* pvObject, CpuCore* pCore, uint32_t addressStart, const void* pBuffer, size_t bufferSize)
{
    RP2040Object* pObject = (RP2040Object*)pvObject;
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
static bool rp2040FlashEnd(DeviceObject* pvObject, CpuCore* pCore)
{
    RP2040Object* pObject = (RP2040Object*)pvObject;
    assert ( pObject && pCore );

    return localFlashingUninit(pObject, pCore);
}


// The FLASH region is set to the maximum 16MB supported by the RP2040 and might not reflect the actual size of the
// FLASH part attached to the CPU.
static const DeviceMemoryRegion g_memoryRegions[] =
{
    { .address = 0x00000000, .length = 0x4000, .blockSize = 0, .type = DEVICE_MEMORY_ROM },
    { .address = 0x10000000, .length = 0x1000000, .blockSize = FLASH_PAGE_SIZE, .type = DEVICE_MEMORY_FLASH },
    { .address = 0x20000000, .length = 0x42000, .blockSize = 0, .type = DEVICE_MEMORY_RAM }
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
const DeviceMemoryLayout* rp2040GetMemoryLayout(DeviceObject* pvObject, CpuCore* pCore)
{
    return &g_memoryLayout;
}

// If the device supports more than 1 core then it can populate the supplied pTargetArray.
//
// This function pointer can be set to the deviceNoAdditionalTarget() function declared below for single core
// devices.
//
// An example implementation where an additional target is added can be found in rp2040.cpp to expose the second
// core on the RP2040.
//
// pvObject is a pointer to the object allocated by detect().
// pCore is a pointer to the CpuCore object used for interfacing to the core.
// pCoreArray is a pointer to an array of CpuCore objects to be initialized for additional cores on this device.
// coreArrayLength is the maximum number of elements in pCoreArray that can be set by this function.
// pCoreCount is the number of additional pCoreArray elements that were actually initialized by this function.
//
// Returns true if successful and false otherwise.
bool rp2040GetAdditionalTargets(DeviceObject* pvObject, CpuCore* pCore, CpuCore* pCoreArray, size_t coreArrayLength, size_t* pCoreCount)
{
    assert ( coreArrayLength >= 1 );

    SWD* pCoreBus = pCore->getTarget()->getSwdBus();
    if (!pCoreBus->selectSwdTarget(SWD::RP2040_CORE1))
    {
        logError("Failed to find the second core on SWD bus.");
        return false;
    }
    // Note: The index passed into init() is 1 (since this is Core1), not 0 like the array index.
    if (!pCoreArray[0].init(pCore, 0+1, CpuCore::DP_CORE))
    {
        logError("Failed to init the second core.");
        return false;
    }

    *pCoreCount = 1;
    return true;
}


// The function pointer table for this device. A pointer to it can be found in g_supportedDevices (in devices.cpp).
DeviceFunctionTable g_rp2040DeviceSupport =
{
    .detect = rp2040Detect,
    .free = rp2040Free,
    .getName = rp2040GetName,
    .hasFpu = deviceHasNoFpu,
    .getMaximumSWDClockFrequency = rp2040MaximumSWDClockFrequency,
    .flashBegin = rp2040FlashBegin,
    .flashErase = rp2040FlashErase,
    .flashProgram = rp2040FlashProgram,
    .flashEnd = rp2040FlashEnd,
    .getMemoryLayout = rp2040GetMemoryLayout,
    .getAdditionalTargets = rp2040GetAdditionalTargets,
    .handleMonitorCommand = NULL
};
