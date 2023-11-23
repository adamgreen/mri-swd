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
// Interface used to add support for Nordic nRF52xxx microcontrollers.

// **********************************************************************************************************
// **** DEVICE_MODULE must be set to reflect the filename of this source file before including logging.h ****
// **********************************************************************************************************
#define DEVICE_MODULE "devices/nrf52.cpp"
#include "logging.h"
#include <stdlib.h>
#include <string.h>
#include "nrf52.h"
#include "mri_platform.h"

// MRI C headers
extern "C"
{
    #include <core/core.h>
    #include <core/gdb_console.h>
}


// nRF52 FLASH memory location.
// The actual size for the attached target is determined by reading the device's FICR.
static const uint32_t FLASH_START_ADDRESS = 0x00000000;
static const uint32_t FLASH_PAGE_SIZE = 4096;

// A FLASH page should take under 100ms to be erased (85ms according to data sheet.)
static const uint32_t FLASH_PAGE_ERASE_TIMEOUT_MS = 100;

// nRF52 SRAM memory location.
// The actual size for the attached target is determined by reading the device's FICR.
static const uint32_t RAM_START_ADDRESS = 0x20000000;

// Maximum SWDCLK frequency for the nRF52.
static const uint32_t MAX_SWD_FREQUENCY = 8000000;
// Slower SWDKCLK frequency to use when writing to FLASH to keep word writes >= 41µs.
static const uint32_t FLASH_WRITE_SWD_FREQUENCY = 800000;


// nRF52 FICR.INFO structure.
struct FICR_Info
{
    // The nRF52xxx part number in hex, ie 0x52840 for the nRF52840.
    uint32_t part;
    // The variant of the nRF52 IC.
    uint32_t variant;
    // The package of the nRF52 IC.
    uint32_t package;
    // The size of the SRAM in 1k units.
    uint32_t ram;
    // The size of the FLASH in 1k units.
    uint32_t flash;
};

// nRF52 class caches things like the FICR.INFO and memory layout here.
struct nRF52Object
{
    FICR_Info ficrInfo;
    DeviceMemoryRegion memoryRegions[2];
    DeviceMemoryLayout memoryLayout;
};


// Forward function declarations.
static bool isAddressAndLength4kAligned(uint32_t address, uint32_t length);
static bool is4kAligned(uint32_t value);
static bool isAttemptingToFlashInvalidAddress(const DeviceMemoryRegion* pFlashRegion, uint32_t address, uint32_t length);
static bool eraseEnable(SwdTarget* pSWD);
static bool erasePage(SwdTarget* pSWD, uint32_t pageAddress);
static bool waitForEraseToComplete(SwdTarget* pSWD);
static bool eraseDisable(SwdTarget* pSWD);
static bool isAddressAndLengthWordAligned(uint32_t address, uint32_t length);
static bool isWordAligned(uint32_t value);
static bool writeEnable(SwdTarget* pSWD);
static bool writeDisable(SwdTarget* pSWD);
static bool eraseFlashAndUICR(SwdTarget* pSWD);


// mri-swd will call this function on each of the devices listed in g_supportedDevices until a non-NULL response
// is encountered, indicating that the device attached to the debugger is supported by that element.
//
// pSWD is a pointer to the SWD object used for interfacing to the device.
//
// The returned pointer can point to anything that the device specific code wants to access on subsequent calls to
// other routines in this function table. It can even be something malloc()ed. Calling the free() function from this
// table can then be used to free it. Should return NULL if the connected target isn't recognized as this type of
// device.
static DeviceObject* nrf52Detect(SwdTarget* pSWD)
{
    // nRF52xxx devices all contain a Cortex-M4 core that should be recognized by the SWD class.
    if (pSWD->getCpuType() != SWD::CPU_CORTEX_M4)
    {
        return NULL;
    }

    // nRF52xxx parts can be recognized by the FICR.INFO.PART field in memory.
    // If the read of this address fails then this isn't a nRF52xxx device.
    const uint32_t FICR_INFO_Address = 0x10000100;
    nRF52Object object;
    if (!pSWD->readMemory(FICR_INFO_Address, &object.ficrInfo, sizeof(object.ficrInfo), SWD::TRANSFER_32BIT))
    {
        return NULL;
    }

    // nRF52xxx devices will have FICR.INFO.PART matching 0x52XXX.
    if ((object.ficrInfo.part & 0xFF000) != 0x52000)
    {
        return NULL;
    }

    // Populate the memory layout from the FICR.INFO.RAM/FLASH data.
    object.memoryRegions[0] = { .address = FLASH_START_ADDRESS,
                                .length = object.ficrInfo.flash * 1024,
                                .blockSize = FLASH_PAGE_SIZE,
                                .type = DEVICE_MEMORY_FLASH };
    object.memoryRegions[1] = { .address = RAM_START_ADDRESS,
                                .length = object.ficrInfo.ram * 1024,
                                .blockSize = 0,
                                .type = DEVICE_MEMORY_RAM };

    // Allocate a nRF52Object and copy the FICR.INFO and memory layout information into it.
    nRF52Object* pObject = (nRF52Object*)malloc(sizeof(*pObject));
    if (!pObject)
    {
        return NULL;
    }
    *pObject = object;
    pObject->memoryLayout = { .pRegions = &pObject->memoryRegions[0], .regionCount = count_of(pObject->memoryRegions) };

    return pObject;
}


// Can be used by the device specific code to free the object returned from detect().
//
// pvObject is a pointer to the object allocated by detect().
// pSWD is a pointer to the SWD object used for interfacing to the device.
static void nrf52Free(DeviceObject* pvObject, SwdTarget* pSWD)
{
    free(pvObject);
}


// Gets friendly name for this device.
//
// pvObject is a pointer to the object allocated by detect().
// pSWD is a pointer to the SWD object used for interfacing to the device.
//
// Returns a \0 terminated string indicating this device's name.
static const char* nrf52GetName(DeviceObject* pvObject, SwdTarget* pSWD)
{
    return "nRF52xxx";
}

// The SWD interface will be set to this frequency from the possibly lower rate used during SWD::init().
//
// pvObject is a pointer to the object allocated by detect().
// pSWD is a pointer to the SWD object used for interfacing to the device.
//
// Returns the maximum SWCLK frequency supported by this device.
static uint32_t nrf52MaximumSWDClockFrequency(DeviceObject* pvObject, SwdTarget* pSWD)
{
    // The nRF52xxx SWD DP can support clock rates up to 8MHz.
    return MAX_SWD_FREQUENCY;
}


// Called at the beginning of the FLASH programming process. The device specific code can perform whatever
// initialization it needs here before the flashErase() and flashProgram() routines are called.
//
// pvObject is a pointer to the object allocated by detect().
// pSWD is a pointer to the SWD object used for interfacing to the device.
//
// Returns true if successful and false otherwise.
static bool nrf52FlashBegin(DeviceObject* pvObject, SwdTarget* pSWD)
{
    assert ( pvObject );

    // Drop the SWD clock rate to 800kHz while programming so that the word writes are slow enough for updating the
    // FLASH on the nRF52 (41µs per 32-bit write).
    if (!pSWD->setFrequency(FLASH_WRITE_SWD_FREQUENCY))
    {
        logError("Failed to slow down the SWCLK for FLASH programming.");
        return false;
    }

    return true;
}


// GDB would like the FLASH at the specified address range to be erased in preparation for programming.
//
// pvObject is a pointer to the object allocated by detect().
// pSWD is a pointer to the SWD object used for interfacing to the device.
// addressStart is the address at the beginning of the range to be erased. (inclusive)
// length is the number of bytes starting at address to be erased.
//
// Returns true if successful and false otherwise.
static bool nrf52FlashErase(DeviceObject* pvObject, SwdTarget* pSWD, uint32_t addressStart, uint32_t length)
{
    nRF52Object* pObject = (nRF52Object*)pvObject;
    assert ( pObject && pSWD );

    if (!isAddressAndLength4kAligned(addressStart, length))
    {
        logErrorF("Address (0x%08lX) and length (0x%lX) are not both 4k aligned.", addressStart, length);
        return false;
    }
    if (isAttemptingToFlashInvalidAddress(&pObject->memoryRegions[0], addressStart, length))
    {
        logErrorF("Erase starting (0x%08lX) and ending (0x%08lX) addresses don't both point to FLASH.", addressStart, addressStart+length);
        return false;
    }

    if (!eraseEnable(pSWD))
    {
        return false;
    }

    uint32_t endAddress = addressStart + length;
    uint32_t address = addressStart;
    bool result = true;
    while (address < endAddress)
    {
        if (!erasePage(pSWD, address))
        {
            logErrorF("Failed to erase page starting at address 0x%08X.", address);
            result = false;
            break;
        }

        address += FLASH_PAGE_SIZE;
    }

    if (!eraseDisable(pSWD))
    {
        return false;
    }

    return result;
}

static bool isAddressAndLength4kAligned(uint32_t address, uint32_t length)
{
    return (is4kAligned(address) && is4kAligned(length));
}

static bool is4kAligned(uint32_t value)
{
    return (value & (4096-1)) == 0;
}

static bool isAttemptingToFlashInvalidAddress(const DeviceMemoryRegion* pFlashRegion, uint32_t address, uint32_t length)
{
    assert ( pFlashRegion->type == DEVICE_MEMORY_FLASH );

    uint32_t endAddress = address + length;
    if (endAddress > pFlashRegion->address + pFlashRegion->length)
    {
        return true;
    }
    return false;
}

static const uint32_t NVMC_CONFIG_Address = 0x4001E504;

static bool eraseEnable(SwdTarget* pSWD)
{
    const uint32_t eraseEnable = 2;
    if (!pSWD->writeMemory(NVMC_CONFIG_Address, &eraseEnable, sizeof(eraseEnable), SWD::TRANSFER_32BIT))
    {
        logError("Failed to write to NVMC.CONFIG to enable FLASH erase.");
        return false;
    }
    return true;
}

static bool erasePage(SwdTarget* pSWD, uint32_t pageAddress)
{
    const uint32_t NVMC_ERASEPAGE_Address = 0x4001E508;
    if (!pSWD->writeMemory(NVMC_ERASEPAGE_Address, &pageAddress, sizeof(pageAddress), SWD::TRANSFER_32BIT))
    {
        logErrorF("Failed to write address 0x%08lX to NVMC.ERASEPAGE.", pageAddress);
        return false;
    }

    if (!waitForEraseToComplete(pSWD))
    {
        logError("Failed to erase all of FLASH and UICR.");
        return false;
    }
    return true;
}

static bool waitForEraseToComplete(SwdTarget* pSWD)
{
    absolute_time_t endTime = make_timeout_time_ms(FLASH_PAGE_ERASE_TIMEOUT_MS);
    uint32_t nvmcReady = 0;
    do
    {
        const uint32_t NVMC_READY_Address = 0x4001E400;
        if (!pSWD->readMemory(NVMC_READY_Address, &nvmcReady, sizeof(nvmcReady), SWD::TRANSFER_32BIT))
        {
            logError("Failed to read NVMC.READY.");
            return false;
        }
    }
    while (nvmcReady == 0 && absolute_time_diff_us(get_absolute_time(), endTime) > 0);

    if (nvmcReady == 0)
    {
        logError("Timed out waiting for NVMC.READY.");
        return false;
    }

    return true;
}

static bool eraseDisable(SwdTarget* pSWD)
{
    const uint32_t readOnly = 0;
    if (!pSWD->writeMemory(NVMC_CONFIG_Address, &readOnly, sizeof(readOnly), SWD::TRANSFER_32BIT))
    {
        logError("Failed to write to NVMC.CONFIG to make FLASH read-only.");
        return false;
    }
    return true;
}


// GDB would like the FLASH at the specified address to be programmed with the supplied data.
//
// pvObject is a pointer to the object allocated by detect().
// pSWD is a pointer to the SWD object used for interfacing to the device.
// addressStart is the address at the beginning of the range to be programmed.
// pBuffer is the address of the data to be programmed into the specified FLASH location.
// bufferSize is the number of bytes in pBuffer to be programmed into FLASH at the specified location.
//
// Returns true if successful and false otherwise.
static bool nrf52FlashProgram(DeviceObject* pvObject, SwdTarget* pSWD, uint32_t addressStart, const void* pBuffer, size_t bufferSize)
{
    nRF52Object* pObject = (nRF52Object*)pvObject;
    assert ( pObject && pSWD );

    if (!isAddressAndLengthWordAligned(addressStart, bufferSize))
    {
        logErrorF("Address (0x%08lX) and length (0x%lX) are not both word aligned.", addressStart, bufferSize);
        return false;
    }
    if (isAttemptingToFlashInvalidAddress(&pObject->memoryRegions[0], addressStart, bufferSize))
    {
        logErrorF("Write starting (0x%08lX) and ending (0x%08lX) addresses don't both point to FLASH.", addressStart, addressStart+bufferSize);
        return false;
    }

    if (!writeEnable(pSWD))
    {
        return false;
    }

    // Make sure that the source data is 4-byte aligned as well.
    uint32_t alignedBuffer[PACKET_SIZE/sizeof(uint32_t)];
    assert ( sizeof(alignedBuffer) >= bufferSize );
    memcpy(alignedBuffer, pBuffer, bufferSize);

    bool result = true;
    if (!pSWD->writeMemory(addressStart, alignedBuffer, bufferSize, SWD::TRANSFER_32BIT))
    {
        logErrorF("Failed to write %lu bytes to FLASH at address 0x%08lX", bufferSize, addressStart);
        result = false;
    }

    if (!writeDisable(pSWD))
    {
        return false;
    }

    return result;
}

static bool isAddressAndLengthWordAligned(uint32_t address, uint32_t length)
{
    return (isWordAligned(address) && isWordAligned(length));
}

static bool isWordAligned(uint32_t value)
{
    return (value & (sizeof(uint32_t)-1)) == 0;
}

static bool writeEnable(SwdTarget* pSWD)
{
    const uint32_t writeEnable = 1;
    if (!pSWD->writeMemory(NVMC_CONFIG_Address, &writeEnable, sizeof(writeEnable), SWD::TRANSFER_32BIT))
    {
        logError("Failed to write to NVMC.CONFIG to enable FLASH write.");
        return false;
    }
    return true;
}

static bool writeDisable(SwdTarget* pSWD)
{
    // The same thing is written to NVMC.CONFIG to disable erasing and writing
    return eraseDisable(pSWD);
}


// Called at the end of the FLASH programming process. The device specific code can perform whatever
// cleanup it needs here after all the flashErase() and flashProgram() call have been made.
//
// pvObject is a pointer to the object allocated by detect().
// pSWD is a pointer to the SWD object used for interfacing to the device.
//
// Returns true if successful and false otherwise.
static bool nrf52FlashEnd(DeviceObject* pvObject, SwdTarget* pSWD)
{
    assert ( pvObject && pSWD );

    // Restore the SWD clock rate back to the maximum now that we are done with FLASH programming.
    if (!pSWD->setFrequency(MAX_SWD_FREQUENCY))
    {
        logError("Failed to restore SWCLK to 8MHz after FLASH programming.");
        return false;
    }

    return true;
}


// Returns the memory layout of the ROM, FLASH, and RAM regions of the device(s). This will be used to provide the
// memory layout XML to GDB.
//
// If it isn't possible to figure out the exact memory layout for a device then set this function pointer to the
// deviceDefaultMemoryLayout() function declared below.
//
// pvObject is a pointer to the object allocated by detect().
// pSWD is a pointer to the SWD object used for interfacing to the device.
//
// Returns a pointer to the memory layout description for this device. Can't be NULL.
const DeviceMemoryLayout* nrf52GetMemoryLayout(DeviceObject* pvObject, SwdTarget* pSWD)
{
    nRF52Object* pObject = (nRF52Object*)pvObject;
    assert ( pObject && pSWD );

    // Return the memory layout for the specific device attached. This layout was generated in rp52Detect().
    return &pObject->memoryLayout;
}


// User has issued a "monitor" command in GDB. This function is called to give the device specific code an
// opportunity to handle the command before letting the mri-swd core code handle it.
//
// pvObject is a pointer to the object allocated by detect().
// pSWD is a pointer to the SWD object used for interfacing to the device.
//
// Returns true if this command has been handled by this device specific code.
// Returns false if the more generic mri-swd monitor command handler should be used instead.
static bool nrf52HandleMonitorCommand(DeviceObject* pvObject, SwdTarget* pSWD, const char** ppArgs, size_t argCount)
{
    // Check for "help" command to list nRF52 specific commands.
    if (argCount >= 1 && strcasecmp(ppArgs[0], "help") == 0)
    {
        WriteStringToGdbConsole("Supported nRF52 monitor commands:\r\n");
        WriteStringToGdbConsole("nrf52 erase_flash_uicr\r\n");
        PrepareStringResponse("OK");
        return true;
    }

    // All other supported commands must be prefixed with "nrf52".
    if (argCount < 2 || strcasecmp(ppArgs[0], "nrf52") != 0)
    {
        return false;
    }

    // Parse and handle nRF52 specific monitor commands.
    if (strcasecmp(ppArgs[1], "erase_flash_uicr") == 0)
    {
        eraseFlashAndUICR(pSWD);
        return true;
    }

    return false;
}

static bool eraseFlashAndUICR(SwdTarget* pSWD)
{
    if (!eraseEnable(pSWD))
    {
        return false;
    }

    const uint32_t NVMC_ERASEALL_Address = 0x4001E50C;
    const uint32_t eraseVal = 1;
    if (!pSWD->writeMemory(NVMC_ERASEALL_Address, &eraseVal, sizeof(eraseVal), SWD::TRANSFER_32BIT))
    {
        logError("Failed to write to NVMC.ERASEALL.");
        return false;
    }

    if (!waitForEraseToComplete(pSWD))
    {
        logError("Failed to erase all of FLASH and UICR.");
        return false;
    }

    if (!eraseDisable(pSWD))
    {
        return false;
    }
    return true;
}

// The function pointer table for this device. A pointer to it can be found in g_supportedDevices (in devices.cpp).
DeviceFunctionTable g_nrf52DeviceSupport =
{
    .detect = nrf52Detect,
    .free = nrf52Free,
    .getName = nrf52GetName,
    .hasFpu = deviceHasFpu,
    .getMaximumSWDClockFrequency = nrf52MaximumSWDClockFrequency,
    .flashBegin = nrf52FlashBegin,
    .flashErase = nrf52FlashErase,
    .flashProgram = nrf52FlashProgram,
    .flashEnd = nrf52FlashEnd,
    .getMemoryLayout = nrf52GetMemoryLayout,
    .getAdditionalTargets = deviceNoAdditionalTargets,
    .handleMonitorCommand = nrf52HandleMonitorCommand
};
