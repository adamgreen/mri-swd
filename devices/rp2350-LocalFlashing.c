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
// RP2350 FLASH programming code that runs locally on the target itself.
// This allows the FLASH erasing and programming to occur in parallel with GDB data transfer.
#include <stdlib.h>
#include "LocalFlashing.h"
#include "../config.h"


// RP2350 FLASH memory location.
#define FLASH_START_ADDRESS     0x10000000

// RP2350 SRAM memory location.
#define RAM_START_ADDRESS       0x20000000
#define RAM_SIZE                (520*1024)
#define RAM_END_ADDRESS         (RAM_START_ADDRESS + RAM_SIZE)
#define RAM_STACK_SIZE          (32*1024)

// Function prototypes.
static void flashingRoutine(void);
static void initCache(void);
static void enableRCP(void);
static void findRequiredBootRomRoutines(void);
static void checkRP2350BootRomMagicValue(void);
static void rp2350FlashErase(uint32_t addressStart, uint32_t length);
static void disableFlashXIP(void);
static void reenableFlashXIP(void);
static void rp2350FlashProgram(uint32_t addressStart, const void* pBuffer, size_t bufferSize);
static uint32_t alignStartOfWriteByCopyingExistingFlashData(uint8_t* pDest, uint32_t alignedStart, uint32_t unalignedStart);
static int isAddressInCache(uint32_t address);
static uint32_t copyBytes(uint8_t* pDest, size_t destSize, const uint8_t* pSrc, uint32_t srcSize);
static void memset(void* b, int c, size_t len);
static void memcpy(void* dst, const void* src, size_t n);
static void cacheBytes(uint32_t startAddress, const uint8_t* pBuffer, uint32_t size);


// Data shared between mri-swd debugger and this code running on the target when loading new code into FLASH.
static LocalFlashingQueueEntry g_queue[32];
static uint32_t                g_ram[(RAM_SIZE - RAM_STACK_SIZE) / sizeof(uint32_t)];
LocalFlashingConfig            g_config =
{
    .queueIndices = { .writeIndex = 0, .readIndex = 0 },
    .queueAddress = (uint32_t)g_queue,
    .queueLength = (uint32_t)(sizeof(g_queue)/sizeof(g_queue[0])),
    .ramAddress = (uint32_t)g_ram,
    .ramLength = (uint32_t)sizeof(g_ram),
    .stackAddress = RAM_END_ADDRESS,
    .routineAddress = (uint32_t)flashingRoutine,
    .loadAddress = RAM_START_ADDRESS
};

// Cache last 256 bytes written to FLASH to be used for alignment on next write if needed.
static uint8_t  g_cache[256];
static uint32_t g_cacheStartAddress;
static uint32_t g_cacheEndAddress;
static int      g_isFlashEnabled;


// Pointers to Boot ROM functions.
typedef void (*rom_connect_internal_flash_fn)(void);
typedef void (*rom_flash_exit_xip_fn)(void);
typedef void (*rom_flash_range_erase_fn)(uint32_t, size_t, uint32_t, uint8_t);
typedef void (*rom_flash_range_program_fn)(uint32_t, const uint8_t*, size_t);
typedef void (*rom_flash_flush_cache_fn)(void);
typedef void (*rom_flash_enter_cmd_xip_fn)(void);
typedef void (*rom_flash_reset_address_trans_fn)(void);

rom_connect_internal_flash_fn    g_connect_internal_flash;
rom_flash_exit_xip_fn            g_flash_exit_xip;
rom_flash_range_erase_fn         g_flash_range_erase;
rom_flash_range_program_fn       g_flash_range_program;
rom_flash_flush_cache_fn         g_flash_flush_cache;
rom_flash_enter_cmd_xip_fn       g_flash_enter_cmd_xip;
rom_flash_reset_address_trans_fn g_flash_reset_address_trans;


// UNDONE: Can the debugger just do this for ARMv-8M targets when running code on the target?
// Adding a flashingRoutine() shim which disables MSP/PSP limits before running any code that might touch the
// stack pointers.
static __attribute__ ((noinline)) void flashingRoutineActual(void);
static __attribute__ ((naked))void flashingRoutine(void)
{
    // Set the MSP/PSP limits to 0.
    uint32_t zeroLimit = 0;
    void (*pFlashingRoutineActual)(void) = flashingRoutineActual;
    __asm__ volatile ("msr msplim, %0" : : "r" (zeroLimit));
    __asm__ volatile ("msr psplim, %0" : : "r" (zeroLimit));
    __asm__ volatile ("bx %0" : : "r" (pFlashingRoutineActual));
}

static __attribute__ ((noinline)) void flashingRoutineActual(void)
{
    initCache();
    enableRCP();
    findRequiredBootRomRoutines();
    g_isFlashEnabled = 1;

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
                rp2350FlashErase(pEntry->flashAddress, pEntry->size);
                break;
            case LOCAL_FLASHING_OP_WRITE:
                rp2350FlashProgram(pEntry->flashAddress, (const void*)pEntry->ramAddress, pEntry->size);
                break;
            case LOCAL_FLASHING_OP_FREE:
            case LOCAL_FLASHING_OP_COMPLETE:
                pEntry->errorCode = 0xBAADF00D;
                // FALL THROUGH!!
                // Shouldn't get FREE or COMPLETED opcodes so they are treated the same as BREAKPOINT so that mri-swd
                // detects that the target has halted and times out.
            case LOCAL_FLASHING_OP_BREAKPOINT:
                reenableFlashXIP();
                pEntry->op = LOCAL_FLASHING_OP_COMPLETE;
                g_config.queueIndices.readIndex = readIndex;
                __asm volatile ("bkpt #0");
                break;
        }

        pEntry->op = LOCAL_FLASHING_OP_COMPLETE;
        g_config.queueIndices.readIndex = readIndex;
    }
}

static void initCache(void)
{
    g_cacheStartAddress = 0;
    g_cacheEndAddress = 0;
}

static void enableRCP(void)
{
    // Make sure that the RCP co-processor is enabled. It is ok to enable it if it was already enabled.
    volatile uint32_t* CPACR = (volatile uint32_t*)0xE000ED88;
    const uint32_t CPACR_CP7_ENABLE_Bitmask = 0x3 << (7 * 2);
    const uint32_t CPACR_CP7_ENABLE = 0x1 << (7 * 2);
    *CPACR = (*CPACR & ~CPACR_CP7_ENABLE_Bitmask) | CPACR_CP7_ENABLE;

    // Check the status of the RCP canary and only seed it if not already done, otherwise the processor will fault.
    const uint32_t RCP_CANARY_VALID_PATTERN = 0xa500a500;
    uint32_t rcpCanaryStatus;
    __asm__ volatile ("mrc p7, #1, %0, c0, c0, #0" : "=r" (rcpCanaryStatus));
    if (rcpCanaryStatus == RCP_CANARY_VALID_PATTERN)
    {
        return;
    }

    // Salt the core0 an core1 canary values with whatever values are in r0.
    __asm__ volatile ("mcrr p7, #8, r0, r0, c0");
    __asm__ volatile ("mcrr p7, #8, r0, r0, c1");
}

static void findRequiredBootRomRoutines(void)
{
    checkRP2350BootRomMagicValue();

    // Read the 16-bit Boot ROM pointer for the ROM function table lookup function.
    typedef void* (*rom_table_lookup_fn)(uint32_t code, uint32_t mask);
    rom_table_lookup_fn romTableLookup = (rom_table_lookup_fn)(uintptr_t) *(uint16_t*)(0x16);

    // The codes in the RP2350 Boot ROM function table entries are 16-bit values formed from 2 characters.
    #define ROM_CODE(X, Y) ((X) | (Y<<8))
    // We always call the ROM fuctions from secure mode.
    const uint32_t RT_FLAG_FUNC_ARM_SEC = 0x4;

    // Lookup each of the ROM routines that we will call later.
    g_connect_internal_flash = romTableLookup(ROM_CODE('I', 'F'), RT_FLAG_FUNC_ARM_SEC);
    g_flash_exit_xip = romTableLookup(ROM_CODE('E', 'X'), RT_FLAG_FUNC_ARM_SEC);
    g_flash_range_erase = romTableLookup(ROM_CODE('R', 'E'), RT_FLAG_FUNC_ARM_SEC);
    g_flash_range_program = romTableLookup(ROM_CODE('R', 'P'), RT_FLAG_FUNC_ARM_SEC);
    g_flash_flush_cache = romTableLookup(ROM_CODE('F', 'C'), RT_FLAG_FUNC_ARM_SEC);
    g_flash_enter_cmd_xip = romTableLookup(ROM_CODE('C', 'X'), RT_FLAG_FUNC_ARM_SEC);
    g_flash_reset_address_trans = romTableLookup(ROM_CODE('R', 'A'), RT_FLAG_FUNC_ARM_SEC);
    if (!g_connect_internal_flash || !g_flash_exit_xip  || !g_flash_range_erase ||
        !g_flash_range_program || !g_flash_flush_cache || !g_flash_enter_cmd_xip || !g_flash_reset_address_trans)
    {
        // Failed to find all of the required functions in the RP2350 Boot ROM before hitting end of table.
        __asm volatile ("bkpt #0");
    }
}

static void checkRP2350BootRomMagicValue(void)
{
    // The RP2350 should contain the expected magic values ('M', 'u', 0x02) at address 0x10.
    const uint8_t* pRomMagic = (const uint8_t*)0x10;
    if (pRomMagic[0] != 'M' || pRomMagic[1] != 'u' || pRomMagic[2] != 0x02)
    {
        // RP2350 ROM magic value is invalid.
        __asm volatile ("bkpt #0");
    }
}

static void rp2350FlashErase(uint32_t addressStart, uint32_t length)
{
    const uint32_t FLASH_BLOCK_SIZE = 1u << 16;
    const uint32_t FLASH_BLOCK_ERASE_CMD = 0xd8;
    disableFlashXIP();
    g_flash_range_erase(addressStart - FLASH_START_ADDRESS, length, FLASH_BLOCK_SIZE, FLASH_BLOCK_ERASE_CMD);
}

static void disableFlashXIP(void)
{
    if (!g_isFlashEnabled)
    {
        return;
    }
    g_connect_internal_flash();
    g_flash_exit_xip();
    g_isFlashEnabled = 0;
}

static void reenableFlashXIP(void)
{
    if (g_isFlashEnabled)
    {
        return;
    }
    g_flash_flush_cache();
    g_flash_enter_cmd_xip();
    g_flash_reset_address_trans();
    g_isFlashEnabled = 1;
}

static void rp2350FlashProgram(uint32_t addressStart, const void* pBuffer, size_t bufferSize)
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
    cacheBytes(alignedStartAddress, buffer, byteCount);
    disableFlashXIP();
    g_flash_range_program(alignedStartAddress-FLASH_START_ADDRESS, buffer, byteCount);
}

static uint32_t alignStartOfWriteByCopyingExistingFlashData(uint8_t* pDest, uint32_t alignedStart, uint32_t unalignedStart)
{
    uint32_t bytesToCopy = unalignedStart - alignedStart;
    if (bytesToCopy == 0)
    {
        // It is already aligned so just return.
        return 0;
    }
    else if (isAddressInCache(alignedStart) && isAddressInCache(unalignedStart-1))
    {
        // The required data was written in the last write and cached away.
        uint32_t offsetInCache = alignedStart - g_cacheStartAddress;
        memcpy(pDest, &g_cache[offsetInCache], bytesToCopy);
    }
    else
    {
        // Need to re-enable FLASH and read the required data from it.
        reenableFlashXIP();
        memcpy(pDest, (void*)alignedStart, bytesToCopy);
    }
    return bytesToCopy;
}

static int isAddressInCache(uint32_t address)
{
    return address >= g_cacheStartAddress && address < g_cacheEndAddress;

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

static void cacheBytes(uint32_t startAddress, const uint8_t* pBuffer, uint32_t size)
{
    uint32_t endAddress = startAddress + size;
    if (size < sizeof(g_cache))
    {
        memcpy(g_cache, pBuffer, size);
        g_cacheStartAddress = startAddress;
        g_cacheEndAddress = endAddress;
    }
    else
    {
        startAddress = endAddress - sizeof(g_cache);
        memcpy(g_cache, pBuffer + size - sizeof(g_cache), sizeof(g_cache));
        g_cacheStartAddress = startAddress;
        g_cacheEndAddress = endAddress;
    }
}
