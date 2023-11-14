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
// RP2040 FLASH programming code that runs locally on the target itself.
// This allows the FLASH erasing and programming to occur in parallel with GDB data transfer.
#include <stdlib.h>
#include "LocalFlashing.h"
#include "../config.h"


// Number of ROM function pointers that need to be discovered for use while erasing and programming.
static const size_t ROM_FUNCTION_COUNT = 6;

// RP2040 FLASH memory location.
static const uint32_t FLASH_START_ADDRESS = 0x10000000;

// RP2040 SRAM memory location.
static const uint32_t RAM_START_ADDRESS = 0x20000000;


// Function prototypes.
static void flashingRoutine(void);
static void findRequiredBootRomRoutines(void);
static void checkRP2040BootRomMagicValue(void);
static void rp2040FlashErase(uint32_t addressStart, uint32_t length);
static void disableFlashXIP(void);
static void reenableFlashXIP(void);
static void rp2040FlashProgram(uint32_t addressStart, const void* pBuffer, size_t bufferSize);
static uint32_t alignStartOfWriteByCopyingExistingFlashData(uint8_t* pDest, uint32_t alignedStart, uint32_t unalignedStart);
static uint32_t copyBytes(uint8_t* pDest, size_t destSize, const uint8_t* pSrc, uint32_t srcSize);
static void memset(void* b, int c, size_t len);
static void memcpy(void* dst, const void* src, size_t n);


// Data shared between mri-swd debugger and this code running on the target when loading new code into FLASH.
LocalFlashingQueueEntry g_queue[32];
uint32_t                g_ram[232*1024 / sizeof(uint32_t)];
LocalFlashingConfig     g_config =
{
    .queueIndices = { .writeIndex = 0, .readIndex = 0 },
    .queueAddress = (uint32_t)g_queue,
    .queueLength = (uint32_t)(sizeof(g_queue)/sizeof(g_queue[0])),
    .ramAddress = (uint32_t)g_ram,
    .ramLength = (uint32_t)sizeof(g_ram),
    .stackAddress = 0x20042000,
    .routineAddress = (uint32_t)flashingRoutine,
    .loadAddress = RAM_START_ADDRESS
};

// Pointers to Boot ROM functions.
typedef void (*rom_connect_internal_flash_fn)(void);
typedef void (*rom_flash_exit_xip_fn)(void);
typedef void (*rom_flash_range_erase_fn)(uint32_t, size_t, uint32_t, uint8_t);
typedef void (*rom_flash_range_program_fn)(uint32_t, const uint8_t*, size_t);
typedef void (*rom_flash_flush_cache_fn)(void);
typedef void (*rom_flash_enter_cmd_xip_fn)(void);

rom_connect_internal_flash_fn   g_connect_internal_flash;
rom_flash_exit_xip_fn           g_flash_exit_xip;
rom_flash_range_erase_fn        g_flash_range_erase;
rom_flash_range_program_fn      g_flash_range_program;
rom_flash_flush_cache_fn        g_flash_flush_cache;
rom_flash_enter_cmd_xip_fn      g_flash_enter_cmd_xip;


static void flashingRoutine(void)
{
    findRequiredBootRomRoutines();

    while (1)
    {
        while (g_config.queueIndices.readIndex == g_config.queueIndices.writeIndex)
        {
            // Wait for the queue to not be empty.
        }

        uint32_t readIndex = g_config.queueIndices.readIndex;
        LocalFlashingQueueEntry* pEntry = &g_queue[readIndex];
        readIndex++;
        if (readIndex >= g_config.queueLength)
        {
            readIndex = 0;
        }
        pEntry->errorCode = 0;
        switch (pEntry->op)
        {
            case LOCAL_FLASHING_OP_ERASE:
                rp2040FlashErase(pEntry->flashAddress, pEntry->size);
                break;
            case LOCAL_FLASHING_OP_WRITE:
                rp2040FlashProgram(pEntry->flashAddress, (const void*)pEntry->ramAddress, pEntry->size);
                break;
            case LOCAL_FLASHING_OP_FREE:
            case LOCAL_FLASHING_OP_COMPLETE:
                pEntry->errorCode = 0xBAADF00D;
                // FALL THROUGH!!
                // Shouldn't get FREE or COMPLETED opcodes so they are treated the same as BREAKPOINT so that mri-swd
                // detects that the target has halted and times out.
            case LOCAL_FLASHING_OP_BREAKPOINT:
                pEntry->op = LOCAL_FLASHING_OP_COMPLETE;
                g_config.queueIndices.readIndex = readIndex;
                __asm volatile ("bkpt #0");
                break;
        }

        pEntry->op = LOCAL_FLASHING_OP_COMPLETE;
        g_config.queueIndices.readIndex = readIndex;
    }
}

static void findRequiredBootRomRoutines(void)
{
    checkRP2040BootRomMagicValue();

    // Read the 16-bit Boot ROM function table pointer from address 0x14.
    const uint16_t* pRomFunctionTable = (const uint16_t*)0x14;
    uint16_t address = *pRomFunctionTable;

    // The codes in the RP2040 Boot ROM function table entries are 16-bit values formed from 2 characters.
    #define ROM_CODE(X, Y) ((X) | (Y<<8))

    // Each function table entry is composed of a 16-bit ROM_CODE and a 16-bit pointer offset into the 64k Boot ROM.
    struct FunctionTableEntry
    {
        uint16_t code;
        uint16_t offset;
    };
    const struct FunctionTableEntry* pEntries = (const struct FunctionTableEntry*)(size_t)address;

    // Search the function table for the routines we need to program the FLASH.
    uint32_t entriesLeftToFind = ROM_FUNCTION_COUNT;
    while (entriesLeftToFind > 0)
    {
        uint32_t offset = pEntries->offset;
        uint32_t code = pEntries->code;
        switch (code)
        {
            case 0x0000:
                // Failed to find all of the required functions in the RP2040 Boot ROM before hitting end of table.
                __asm volatile ("bkpt #0");
                break;
            case ROM_CODE('I', 'F'):
                g_connect_internal_flash = (rom_connect_internal_flash_fn)offset;
                entriesLeftToFind--;
                break;
            case ROM_CODE('E', 'X'):
                g_flash_exit_xip = (rom_flash_exit_xip_fn)offset;
                entriesLeftToFind--;
                break;
            case ROM_CODE('R', 'E'):
                g_flash_range_erase = (rom_flash_range_erase_fn)offset;
                entriesLeftToFind--;
                break;
            case ROM_CODE('R', 'P'):
                g_flash_range_program = (rom_flash_range_program_fn)offset;
                entriesLeftToFind--;
                break;
            case ROM_CODE('F', 'C'):
                g_flash_flush_cache = (rom_flash_flush_cache_fn)offset;
                entriesLeftToFind--;
                break;
            case ROM_CODE('C', 'X'):
                g_flash_enter_cmd_xip = (rom_flash_enter_cmd_xip_fn)offset;
                entriesLeftToFind--;
                break;
            default:
                break;
        }
        pEntries++;
    }
}

static void checkRP2040BootRomMagicValue(void)
{
    // The RP2040 should contain the expected magic values ('M', 'u', 0x01) at address 0x10.
    const uint8_t* pRomMagic = (const uint8_t*)0x10;
    if (pRomMagic[0] != 'M' || pRomMagic[1] != 'u' || pRomMagic[2] != 0x01)
    {
        // RP2040 ROM magic value is invalid.
        __asm volatile ("bkpt #0");
    }
}

static void rp2040FlashErase(uint32_t addressStart, uint32_t length)
{
    const uint32_t FLASH_BLOCK_SIZE = 1u << 16;
    const uint32_t FLASH_BLOCK_ERASE_CMD = 0xd8;
    disableFlashXIP();
    g_flash_range_erase(addressStart - FLASH_START_ADDRESS, length, FLASH_BLOCK_SIZE, FLASH_BLOCK_ERASE_CMD);
    reenableFlashXIP();
}

static void disableFlashXIP(void)
{
    g_connect_internal_flash();
    g_flash_exit_xip();
}

static void reenableFlashXIP(void)
{
    g_flash_flush_cache();
    g_flash_enter_cmd_xip();
}

static void rp2040FlashProgram(uint32_t addressStart, const void* pBuffer, size_t bufferSize)
{
    // Need to align both the address and length to 256-bytes.
    const uint32_t alignment = 256;
    uint32_t alignedStartAddress = addressStart & ~(alignment-1);

    // Copy bytes into 256-byte aligned buffer.
    static uint8_t buffer[PACKET_SIZE - 4 + 2 * 256];

    // Align start of FLASH write to page boundary by padding with existing bytes from beginning of first word in FLASH.
    uint32_t bytesLeft = sizeof(buffer);
    uint8_t* pDest = buffer;
    uint32_t size = alignStartOfWriteByCopyingExistingFlashData(pDest, alignedStartAddress, addressStart);
    bytesLeft -= size;
    pDest += size;

    // Copy the bytes provided by GDB into aligned buffer.
    size = copyBytes(pDest, bytesLeft, (const uint8_t*)pBuffer, bufferSize);
    bytesLeft -= size;
    pDest += size;

    // Pad last few bytes with 0xFF to make the length of the write page aligned as well.
    uint32_t endAddress = addressStart + size;
    uint32_t alignedEndAddress = (endAddress+alignment-1) & ~(alignment-1);
    memset(pDest, 0xFF, alignedEndAddress-endAddress);
    uint32_t byteCount = alignedEndAddress-alignedStartAddress;

    // Copy bytes from device RAM to FLASH.
    disableFlashXIP();
    g_flash_range_program(alignedStartAddress-FLASH_START_ADDRESS, buffer, byteCount);
    reenableFlashXIP();
}

static uint32_t alignStartOfWriteByCopyingExistingFlashData(uint8_t* pDest, uint32_t alignedStart, uint32_t unalignedStart)
{
    uint32_t bytesToCopy = unalignedStart - alignedStart;
    memcpy(pDest, (void*)alignedStart, bytesToCopy);
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

static void memset(void* b, int c, size_t len)
{
    uint8_t* p = b;
    while (len-- > 0)
    {
        *p++ = c;
    }
}

static void memcpy(void* dst, const void* src, size_t n)
{
    uint8_t* pDest = dst;
    const uint8_t* pSrc = src;
    while (n-- > 0)
    {
        *pDest++ = *pSrc++;
    }
}
