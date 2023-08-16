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
struct RP2040Object
{
    uint32_t _connect_internal_flash;
    uint32_t _flash_exit_xip;
    uint32_t _flash_range_erase;
    uint32_t _flash_range_program;
    uint32_t _flash_flush_cache;
    uint32_t _flash_enter_cmd_xip;
    uint32_t _debug_trampoline;
    uint32_t _debug_trampoline_end;
    bool     isInitialized;
};

// Number of function pointers to be filled out in the RP2040Object structure.
static const size_t ROM_FUNCTION_COUNT = 8;


// Forward function declarations.
static bool findRequiredBootRomRoutines(RP2040Object* pObject, SWD* pSWD);
static bool checkRP2040BootRomMagicValue(SWD* pSWD);
static bool isAddressAndLength4kAligned(uint32_t address, uint32_t length);
static bool is4kAligned(uint32_t value);
static bool isAttemptingToFlashInvalidAddress(uint32_t address, uint32_t length);
static bool disableFlashXIP(RP2040Object* pObject, SWD* pSWD);
static bool reenableFlashXIP(RP2040Object* pObject, SWD* pSWD);
static bool callBootRomRoutine(RP2040Object* pObject, SWD* pSWD, uint32_t functionOffset, uint32_t param1, uint32_t param2, uint32_t param3, uint32_t param4);
static int32_t alignStartOfWriteByCopyingExistingFlashData(SWD* pSWD, uint8_t* pDest, uint32_t alignedStart, uint32_t unalignedStart);
static uint32_t copyBytes(uint8_t* pDest, size_t destSize, const uint8_t* pSrc, uint32_t srcSize);


// mri-swd will call this function on each of the devices listed in g_supportedDevices until a non-NULL response
// is encountered, indicating that the device attached to the debugger is supported by that element.
//
// pSWD is a pointer to the SWD object used for interfacing to the device.
//
// The returned pointer can point to anything that the device specific code wants to access on subsequent calls to
// other routines in this function table. It can even be something malloc()ed. Calling the free() function from this
// table can then be used to free it. Should return NULL if the connected target isn't recognized as this type of
// device.
static DeviceObject* rp2040Detect(SWD* pSWD)
{
    if (pSWD->getTarget() != SWD::RP2040_CORE0)
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
// pSWD is a pointer to the SWD object used for interfacing to the device.
static void rp2040Free(DeviceObject* pvObject, SWD* pSWD)
{
    free(pvObject);
}


// Gets friendly name for this device.
//
// pvObject is a pointer to the object allocated by detect().
// pSWD is a pointer to the SWD object used for interfacing to the device.
//
// Returns a \0 terminated string indicating this device's name.
static const char* rp2040GetName(DeviceObject* pvObject, SWD* pSWD)
{
    return "RP2040";
}

// The SWD interface will be set to this frequency from the possibly lower rate used during SWD::init().
//
// pvObject is a pointer to the object allocated by detect().
// pSWD is a pointer to the SWD object used for interfacing to the device.
//
// Returns the maximum SWCLK frequency supported by this device.
static uint32_t rp2040MaximumSWDClockFrequency(DeviceObject* pvObject, SWD* pSWD)
{
    // The RP2040 SWD DP can support clock rates up to 24MHz.
    return 24000000;
}


// Called at the beginning of the FLASH programming process. The device specific code can perform whatever
// initialization it needs here before the flashErase() and flashProgram() routines are called.
//
// pvObject is a pointer to the object allocated by detect().
// pSWD is a pointer to the SWD object used for interfacing to the device.
//
// Returns true if successful and false otherwise.
static bool rp2040FlashBegin(DeviceObject* pvObject, SWD* pSWD)
{
    assert ( pvObject );
    return findRequiredBootRomRoutines((RP2040Object*)pvObject, pSWD);
}

static bool findRequiredBootRomRoutines(RP2040Object* pObject, SWD* pSWD)
{
    if (pObject->isInitialized)
    {
        // Function table has already been initialized so just return.
        return true;
    }

    if (!checkRP2040BootRomMagicValue(pSWD))
    {
        logError("Failed verification of the RP2040 Boot ROM magic value.");
        return false;
    }

    // Read the 16-bit Boot ROM function table pointer from address 0x14.
    const uint32_t romFuctionTableAddress = 0x14;
    uint16_t address = 0;
    uint32_t bytesRead = pSWD->readTargetMemory(romFuctionTableAddress, &address, sizeof(address), SWD::TRANSFER_16BIT);
    if (bytesRead != sizeof(address) || address == 0)
    {
        logError("Failed to read RP2040 Boot ROM function table pointer.");
        return false;
    }

    // The codes in the RP2040 Boot ROM function table entries are 16-bit values formed from 2 characters.
    #define ROM_CODE(X, Y) ((X) | (Y<<8))

    // Each function table entry is composed of a 16-bit ROM_CODE and a 16-bit pointer offset into the 64k Boot ROM.
    struct FunctionTableEntry
    {
        uint16_t code;
        uint16_t offset;
    };
    FunctionTableEntry entries[32];

    // Search the function table for the routines we need to program the FLASH.
    uint32_t entriesLeftToFind = ROM_FUNCTION_COUNT;
    while (entriesLeftToFind > 0)
    {
        bytesRead = pSWD->readTargetMemory(address, entries, sizeof(entries), SWD::TRANSFER_16BIT);
        if (bytesRead != sizeof(entries))
        {
            logErrorF("Failed to read RP2040 Boot ROM function table at 0x08X. Bytes read: %lu.", address, bytesRead);
            return false;
        }

        for (size_t i = 0 ; i < count_of(entries) && entriesLeftToFind > 0 ; i++)
        {
            uint32_t offset = entries[i].offset;
            uint32_t code = entries[i].code;
            switch (code)
            {
                case 0x0000:
                    // Have encountered the end of the function table before finding all of the needed routines.
                    assert ( entriesLeftToFind > 0 );
                    logErrorF("Failed to find all of the required functions in the RP2040 Boot ROM. "
                                "Still required %lu functions.", entriesLeftToFind);
                    return false;
                case ROM_CODE('I', 'F'):
                    pObject->_connect_internal_flash = offset;
                    entriesLeftToFind--;
                    break;
                case ROM_CODE('E', 'X'):
                    pObject->_flash_exit_xip = offset;
                    entriesLeftToFind--;
                    break;
                case ROM_CODE('R', 'E'):
                    pObject->_flash_range_erase = offset;
                    entriesLeftToFind--;
                    break;
                case ROM_CODE('R', 'P'):
                    pObject->_flash_range_program = offset;
                    entriesLeftToFind--;
                    break;
                case ROM_CODE('F', 'C'):
                    pObject->_flash_flush_cache = offset;
                    entriesLeftToFind--;
                    break;
                case ROM_CODE('C', 'X'):
                    pObject->_flash_enter_cmd_xip = offset;
                    entriesLeftToFind--;
                    break;
                case ROM_CODE('D', 'T'):
                    pObject->_debug_trampoline = offset;
                    entriesLeftToFind--;
                    break;
                case ROM_CODE('D', 'E'):
                    pObject->_debug_trampoline_end = offset;
                    entriesLeftToFind--;
                    break;
                default:
                    break;
            }
        }
        address += bytesRead;
    }

    pObject->isInitialized = true;
    return true;
}

static bool checkRP2040BootRomMagicValue(SWD* pSWD)
{
    // The RP2040 should contain this magic value at address 0x10.
    const uint32_t bootRomMagicAddress = 0x10;
    const uint8_t  bootRomMagicValue[] = { 'M', 'u', 0x01 };

    uint8_t  value[3] = { 0, 0, 0 };
    uint32_t bytesRead = pSWD->readTargetMemory(bootRomMagicAddress, &value, sizeof(value), SWD::TRANSFER_8BIT);
    if (bytesRead != sizeof(value))
    {
        logError("Failed to read RP2040 Boot ROM magic value.");
        return false;
    }
    if (memcmp(&value, bootRomMagicValue, sizeof(bootRomMagicValue)) != 0)
    {
        logErrorF("RP2040 ROM magic value is invalid (0x%02X 0x%02X 0x%02X).", value[0], value[1], value[2]);
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
static bool rp2040FlashErase(DeviceObject* pvObject, SWD* pSWD, uint32_t addressStart, uint32_t length)
{
    RP2040Object* pObject = (RP2040Object*)pvObject;
    assert ( pObject && pSWD );

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

    if (!disableFlashXIP(pObject, pSWD))
    {
        logError("Failed to disable FLASH XIP.");
        return false;
    }

    // UNDONE: These are probably specific to the FLASH used on the Pico and compatible devices.
    const uint32_t FLASH_BLOCK_SIZE = 1u << 16;
    const uint32_t FLASH_BLOCK_ERASE_CMD = 0xd8;
    if (!callBootRomRoutine(pObject, pSWD, pObject->_flash_range_erase, addressStart - FLASH_START_ADDRESS, length, FLASH_BLOCK_SIZE, FLASH_BLOCK_ERASE_CMD))
    {
        logErrorF("Failed calling _flash_range_erase(0x%08lX, %lu, %lu, 0x%02X) in RP2040 Boot ROM.",
                addressStart - FLASH_START_ADDRESS, length, FLASH_BLOCK_SIZE, FLASH_BLOCK_ERASE_CMD);
        reenableFlashXIP(pObject, pSWD);
        return false;
    }

    if (!reenableFlashXIP(pObject, pSWD))
    {
        logError("Failed to reenable FLASH XIP.");
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

static bool disableFlashXIP(RP2040Object* pObject, SWD* pSWD)
{
    if (!callBootRomRoutine(pObject, pSWD, pObject->_connect_internal_flash, 0, 0, 0, 0))
    {
        logError("Failed calling _connect_internal_flash() in RP2040 Boot ROM.");
        return false;
    }
    if (!callBootRomRoutine(pObject, pSWD, pObject->_flash_exit_xip, 0, 0, 0, 0))
    {
        logError("Failed calling _flash_exit_xip() in RP2040 Boot ROM.");
        return false;
    }

    return true;
}

static bool reenableFlashXIP(RP2040Object* pObject, SWD* pSWD)
{
    if (!callBootRomRoutine(pObject, pSWD, pObject->_flash_flush_cache, 0, 0, 0, 0))
    {
        logError("Failed calling _flash_flush_cache() in RP2040 Boot ROM.");
        return false;
    }
    if (!callBootRomRoutine(pObject, pSWD, pObject->_flash_enter_cmd_xip, 0, 0, 0, 0))
    {
        logError("Failed calling _flash_enter_cmd_xip() in RP2040 Boot ROM.");
        return false;
    }

    return true;
}

static bool callBootRomRoutine(RP2040Object* pObject, SWD* pSWD, uint32_t functionOffset, uint32_t param1, uint32_t param2, uint32_t param3, uint32_t param4)
{
    // Set the CPU registers required for calling the Boot ROM function.
    CortexM_Registers registers;
    memset(&registers, 0, sizeof(registers));
    registers.R0 = param1;
    registers.R1 = param2;
    registers.R2 = param3;
    registers.R3 = param4;
    registers.R7 = functionOffset;
    registers.SP = RAM_END_ADDRESS;
    registers.LR = pObject->_debug_trampoline_end;
    registers.PC = pObject->_debug_trampoline;

    bool result = runCodeOnDevice(&registers, EXEC_RP2040_BOOT_ROM_FUNC_TIMEOUT_MS);
    if (!result)
    {
        return false;
    }

    // Should be stopped at the _debug_trampoline_end symbol.
    // * Set thumb bit in PC to match function pointer address.
    uint32_t pc = registers.PC | 1;
    if (pc != pObject->_debug_trampoline_end)
    {
        logErrorF("RP2040 Boot ROM routine didn't stop as expected. Expected: 0x%08lX Actual: 0x%08lX.",
                  pObject->_debug_trampoline_end, pc);
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
static bool rp2040FlashProgram(DeviceObject* pvObject, SWD* pSWD, uint32_t addressStart, const void* pBuffer, size_t bufferSize)
{
    RP2040Object* pObject = (RP2040Object*)pvObject;
    assert ( pObject && pSWD );

    if (isAttemptingToFlashInvalidAddress(addressStart, bufferSize))
    {
        logErrorF("Write starting (0x%08lX) and ending (0x%08lX) addresses don't both point to FLASH.", addressStart, addressStart+bufferSize);
        return false;
    }

    // Need to align both the address and length to 256-bytes.
    const uint32_t alignment = 256;
    uint32_t alignedStartAddress = addressStart & ~(alignment-1);

    // Copy bytes into 256-byte aligned buffer on the stack.
    const uint32_t maxPacketBytes = PACKET_SIZE-4;
    uint8_t buffer[maxPacketBytes + 2*alignment];

    // Align start of FLASH write to word boundary by padding with existing bytes from beginning of first word in FLASH.
    uint32_t bytesLeft = sizeof(buffer);
    uint8_t* pDest = buffer;
    int32_t size = alignStartOfWriteByCopyingExistingFlashData(pSWD, pDest, alignedStartAddress, addressStart);
    if (size < 0)
    {
        return false;
    }
    bytesLeft -= size;
    pDest += size;

    // Copy the bytes provided by GDB into aligned buffer.
    size = copyBytes(pDest, bytesLeft, (const uint8_t*)pBuffer, bufferSize);
    bytesLeft -= size;
    pDest += size;

    // Pad last few bytes with 0xFF to make the length of the write aligned as well.
    uint32_t endAddress = addressStart + size;
    uint32_t alignedEndAddress = (endAddress+alignment-1) & ~(alignment-1);
    memset(pDest, 0xFF, alignedEndAddress-endAddress);
    uint32_t byteCount = alignedEndAddress-alignedStartAddress;

    // Copy buffer from debugger to beginning of target RAM.
    uint32_t bytesCopied = pSWD->writeTargetMemory(RAM_START_ADDRESS, buffer, byteCount, SWD::TRANSFER_32BIT);
    if (bytesCopied != byteCount)
    {
        logErrorF("Failed to copy %lu bytes to target RAM.", byteCount);
        return false;
    }

    // Disable FLASH XIP before writing to it.
    if (!disableFlashXIP(pObject, pSWD))
    {
        logError("Failed to disable FLASH XIP.");
        return false;
    }

    // Copy bytes from device RAM to FLASH.
    bool result = callBootRomRoutine(pObject, pSWD, pObject->_flash_range_program,
                                     alignedStartAddress-FLASH_START_ADDRESS, RAM_START_ADDRESS, byteCount, 0);
    if (!result)
    {
        logError("Failed calling _flash_range_program() in RP2040 Boot ROM.");
    }

    // Need to re-enable for future alignment padding reads.
    if (!reenableFlashXIP(pObject, pSWD))
    {
        logError("Failed to re-enable FLASH XIP.");
        return false;
    }

    return result;
}

static int32_t alignStartOfWriteByCopyingExistingFlashData(SWD* pSWD, uint8_t* pDest, uint32_t alignedStart, uint32_t unalignedStart)
{
    uint32_t bytesToCopy = unalignedStart - alignedStart;
    uint32_t bytesRead = pSWD->readTargetMemory(alignedStart, pDest, bytesToCopy, SWD::TRANSFER_8BIT);
    if (bytesRead != bytesToCopy)
    {
        logErrorF("Failed to read unaligned %lu bytes at 0x%08lX.", bytesToCopy, alignedStart);
        return -1;
    }

    return bytesToCopy;
}

static uint32_t copyBytes(uint8_t* pDest, size_t destSize, const uint8_t* pSrc, uint32_t srcSize)
{
    uint8_t* pStart = pDest;
    while (destSize-- > 0 && srcSize-- > 0)
    {
        *pDest++ = *pSrc++;
    }
    return pDest - pStart;
}


// Called at the end of the FLASH programming process. The device specific code can perform whatever
// cleanup it needs here after all the flashErase() and flashProgram() call have been made.
//
// pvObject is a pointer to the object allocated by detect().
// pSWD is a pointer to the SWD object used for interfacing to the device.
//
// Returns true if successful and false otherwise.
static bool rp2040FlashEnd(DeviceObject* pvObject, SWD* pSWD)
{
    assert ( pvObject && pSWD );

    return true;
}


// The FLASH region is set to the maximum 16MB supported by the RP2040 and might not reflect the actual size of the the
// FLASH part attached to the CPU.
static const DeviceMemoryRegion g_memoryRegions[] =
{
    { .address = 0x00000000, .length = 0x4000, .blockSize = 0, .type = DEVICE_MEMORY_ROM },
    { .address = 0x10000000, .length = 0x1000000, .blockSize = FLASH_PAGE_SIZE, .type = DEVICE_MEMORY_FLASH },
    { .address = 0x20000000, .length = 0x42000, .blockSize = 0, .type = DEVICE_MEMORY_ROM }
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
// pSWD is a pointer to the SWD object used for interfacing to the device.
//
// Returns a pointer to the memory layout description for this device. Can't be NULL.
const DeviceMemoryLayout* rp2040GetMemoryLayout(DeviceObject* pvObject, SWD* pSWD)
{
    return &g_memoryLayout;
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
    .handleMonitorCommand = NULL
};
