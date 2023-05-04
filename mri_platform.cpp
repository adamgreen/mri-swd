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
// Routines to expose the Cortex-M debug functionality via SWD to the mri debug core.
#include <string.h>
#include <stdio.h>
#include "gdb_socket.h"
#include "swd.h"
#include "mri_platform.h"

// MRI C headers
extern "C"
{
    #include <core/core.h>
    #include <core/mri.h>
    #include <core/platforms.h>
    #include <core/semihost.h>
}


// UNDONE: Maybe move these pin connections and TCP/IP port numbers out into their own config.h
// UNDONE: Move other configurations parameters such as GDB packet size, etc.
// SWD pin connections.
#define SWCLK_PIN 0
#define SWDIO_PIN 1


// Macros used to enable/disable logging of SWD error conditions.
#define logFailure(X) errorf("%s:%u %s() - " X "\n", __FILE__, __LINE__, __FUNCTION__)

static int (*errorf)(const char* format, ...) = printf;


static SWD       g_swd;
static GDBSocket g_gdbSocket;
static bool      g_wasStopFromGDB = false;
static bool      g_wasMemoryExceptionEncountered = false;


// The number of special registers (msp, psp, primask, basepri, faultmask, and control) is 6.
static const uint32_t specialRegisterCount = 6;
// The number of integer registers (R0-R12,SP,LR,PC,XPSR) is 17.
static const uint32_t integerRegisterCount = 17;
// The number of float registers (S0-S32, FPSCR) is 33.
static const uint32_t floatRegisterCount = 33;
// The number of registers in the CPU content depends on whether device has a FPU or not.
static const uint32_t registerCountNoFPU = integerRegisterCount + specialRegisterCount;
static const uint32_t registerCountFPU = registerCountNoFPU + floatRegisterCount;

// Give friendly names to the indices of important registers in the context scatter gather list.
const uint32_t R0 = 0;
const uint32_t R1 = 1;
const uint32_t R2 = 2;
const uint32_t R3 = 3;
const uint32_t R7 = 7;
const uint32_t SP = 13;
const uint32_t LR = 14;
const uint32_t PC = 15;
const uint32_t CPSR = 16;
const uint32_t MSP = 17;
const uint32_t PSP = 18;
const uint32_t PRIMASK = 19;
const uint32_t BASEPRI = 20;
const uint32_t FAULTMASK = 21;
const uint32_t CONTROL = 22;

// CPU register context information is stored here.
static uint32_t       g_contextRegisters[registerCountFPU];
static ContextSection g_contextEntriesNoFPU = { .pValues = &g_contextRegisters[0], .count = registerCountNoFPU };
static ContextSection g_contextEntriesFPU = { .pValues = &g_contextRegisters[0], .count = registerCountFPU };
static MriContext     g_context;


// Forward Function Declarations.
static bool initSWD();
static void requestCpuToHalt();
static bool readDHCSR(uint32_t* pValue);
static bool writeDHCSR(uint32_t DHCSR_Value);
static bool hasCpuHalted(uint32_t DHCSR_Val);
static void saveContext();
static bool readCpuRegister(uint32_t registerIndex, uint32_t* pValue);
static void waitForRegisterTransferToComplete();
static bool hasRegisterTransferCompleted(uint32_t DHCSR_Value);
static void restoreContext();
static bool writeCpuRegister(uint32_t registerIndex, uint32_t value);
static void requestCpuToResume();


void mainDebuggerLoop()
{
    // UNDONE: Need to handle clock rates other than 24MHz.
    bool wasSwdInitSuccessful = initSWD();
    if (!wasSwdInitSuccessful)
    {
        printf("Failed to initialize SWD connection to debuggee.\n");
        __breakpoint();
    }

    bool wasSocketInitSuccessful = g_gdbSocket.init();
    if (!wasSocketInitSuccessful)
    {
        printf("Failed to initialize GDB socket.\n");
        __breakpoint();
    }

    // Initialize the MRI core.
    __try
    {
        mriInit("");
    }
    __catch
    {
        printf("Failed to initialize the MRI core.");
        __breakpoint();
        return;
    }

    while (true)
    {
        bool haveGdbStopRequest = !g_gdbSocket.m_tcpToMriQueue.isEmpty();
        if (haveGdbStopRequest)
        {
            g_wasStopFromGDB = true;
            requestCpuToHalt();
        }

        // Read the Debug Halting Control and Status Register to query current state of CPU.
        uint32_t DHCSR_Val = 0;
        if (!readDHCSR(&DHCSR_Val))
        {
            continue;
        }

        if (hasCpuHalted(DHCSR_Val))
        {
            saveContext();
            mriDebugException(&g_context);
            restoreContext();
            requestCpuToResume();
        }
    }
}

static bool initSWD()
{
    if (!g_swd.init(24000000, SWCLK_PIN, SWDIO_PIN))
    {
        printf("Failed to initialize the SWD port.\n");
        return false;
    }

    bool notDormant = false;
    notDormant = g_swd.sendJtagToSwdSequence();
    bool maybeDormant = !notDormant;
    if (notDormant)
    {
        printf("Target already selected with DPIDR=0x%08lX\n", g_swd.getDPIDR());
    }
    else if (maybeDormant)
    {
        bool foundTarget = g_swd.searchForKnownSwdTarget();
        if (foundTarget)
        {
            printf("Found target=0x%08X with DPIDR=0x%08lX\n", g_swd.getTarget(), g_swd.getDPIDR());
        }
        else
        {
            printf("SWD failed to switch SWD port out of dormant mode.\n");
            return false;
        }
    }

    printf("Initializing target's debug components...\n");
    bool result = g_swd.initTargetForDebugging();
    if (!result)
    {
        printf("Failed to initialize target's debug components.\n");
        return false;
    }
    printf("Initialization complete!\n");
    return true;
}

static const uint32_t DHCSR_C_HALT_Bit = 1 << 1;

static void requestCpuToHalt()
{
    uint32_t DHCSR_Val = 0;

    readDHCSR(&DHCSR_Val);

    DHCSR_Val |= DHCSR_C_HALT_Bit;

    if (!writeDHCSR(DHCSR_Val))
    {
        logFailure("Failed to set C_HALT bit in DHCSR.");
    }
}

static const uint32_t DHCSR_Address = 0xE000EDF0;

static bool readDHCSR(uint32_t* pValue)
{
    if (!g_swd.readTargetMemory(DHCSR_Address, pValue, sizeof(*pValue), SWD::TRANSFER_32BIT))
    {
        logFailure("Failed to read DHCSR register.");
        return false;
    }
    return true;
}

static bool writeDHCSR(uint32_t DHCSR_Value)
{
    // Upper 16-bits must contain DBGKEY for CPU to accept this write.
    const uint32_t DHCSR_DBGKEY_Shift = 16;
    const uint32_t DHCSR_DBGKEY_Mask = 0xFFFF << DHCSR_DBGKEY_Shift;
    const uint32_t DHCSR_DBGKEY = 0xA05F << DHCSR_DBGKEY_Shift;
    DHCSR_Value = (DHCSR_Value & ~DHCSR_DBGKEY_Mask) | DHCSR_DBGKEY;

    return g_swd.writeTargetMemory(DHCSR_Address, &DHCSR_Value, sizeof(DHCSR_Value), SWD::TRANSFER_32BIT);
}

static bool hasCpuHalted(uint32_t DHCSR_Val)
{
    const uint32_t DHCSR_S_HALT_Bit = 1 << 17;
    return (DHCSR_Val & DHCSR_S_HALT_Bit) == DHCSR_S_HALT_Bit;
}

// The special registers: CONTROL, FAULTMASK, BASEPRI, and PRIMASK are accessed at this single DCRSR index.
const uint32_t specialRegisterIndex = 0x14;

static void saveContext()
{
    bool encounteredError = false;

    // UNDONE: Support chips with FPU as well.
    (void)g_contextEntriesFPU;
    Context_Init(&g_context, &g_contextEntriesNoFPU, 1);

    // Transfer R0 - PSP first.
    for (uint32_t i = R0 ; i <= PSP ; i++)
    {
        uint32_t regValue = 0;
        if (!readCpuRegister(i, &regValue))
        {
            encounteredError = true;
        }
        Context_Set(&g_context, i, regValue);
    }

    // Transfer CONTROL, FAULTMASK, BASEPRI, PRIMASK next. They are all accessed in the CPU via a single 32-bit entry.
    uint32_t specialRegs = 0;
    if (!readCpuRegister(specialRegisterIndex, &specialRegs))
    {
        encounteredError = true;
    }
    for (uint32_t i = 0 ; i < 4 ; i++)
    {
        Context_Set(&g_context, PRIMASK+i, (specialRegs >> (8 * i)) & 0xFF);
    }

    // UNDONE: Should transfer FPU registers.

    if (encounteredError)
    {
        logFailure("Failed to read CPU register(s).");
    }
}

// Debug Core Register Selector Register.
static uint32_t DCRSR_Address = 0xE000EDF4;
// Specifies the access type for the transfer: 0 for read, 1 for write.
static uint32_t DCRSR_REGWnR_Bit = 1 << 16;
// Specifies the ARM core register, special-purpose register, or Floating-point Extension register, to transfer.
static uint32_t DCRSR_REGSEL_Mask = 0x7F;

// Debug Core Register Data Register.
static uint32_t DCRDR_Address = 0xE000EDF8;

static bool readCpuRegister(uint32_t registerIndex, uint32_t* pValue)
{
    uint32_t DCRSR_Value = registerIndex & DCRSR_REGSEL_Mask;
    if (!g_swd.writeTargetMemory(DCRSR_Address, &DCRSR_Value, sizeof(DCRSR_Value), SWD::TRANSFER_32BIT))
    {
        return false;
    }

    waitForRegisterTransferToComplete();

    return g_swd.readTargetMemory(DCRDR_Address, pValue, sizeof(*pValue), SWD::TRANSFER_32BIT);
}

static void waitForRegisterTransferToComplete()
{
    uint32_t DHCSR_Value = 0;
    do
    {
        if (!readDHCSR(&DHCSR_Value))
        {
            return;
        }
    } while (!hasRegisterTransferCompleted(DHCSR_Value));
}

static bool hasRegisterTransferCompleted(uint32_t DHCSR_Value)
{
    const uint32_t DHCSR_S_REGRDY_Bit = 1 << 16;
    return (DHCSR_Value & DHCSR_S_REGRDY_Bit) == DHCSR_S_REGRDY_Bit;
}

static void restoreContext()
{
    bool encounteredError = false;

    // UNDONE: Support chips with FPU as well.

    // Transfer R0 - PSP first.
    for (uint32_t i = R0 ; i <= PSP ; i++)
    {
        if (!writeCpuRegister(i, Context_Get(&g_context, i)))
        {
            encounteredError = true;
        }
    }

    // Transfer CONTROL, FAULTMASK, BASEPRI, PRIMASK next. They are all accessed in the CPU via a single 32-bit entry.
    uint32_t specialRegs = 0;
    for (uint32_t i = 0 ; i < 4 ; i++)
    {
        specialRegs |= (Context_Get(&g_context, PRIMASK+i) & 0xFF) << (8 * i);
    }
    if (!writeCpuRegister(specialRegisterIndex, specialRegs))
    {
        encounteredError = true;
    }

    // UNDONE: Should transfer FPU registers.

    if (encounteredError)
    {
        logFailure("Failed to write CPU register(s).");
    }
}

static bool writeCpuRegister(uint32_t registerIndex, uint32_t value)
{
    if (!g_swd.writeTargetMemory(DCRDR_Address, &value, sizeof(value), SWD::TRANSFER_32BIT))
    {
        return false;
    }

    uint32_t DCRSR_Value = DCRSR_REGWnR_Bit | (registerIndex & DCRSR_REGSEL_Mask);
    if (!g_swd.writeTargetMemory(DCRSR_Address, &DCRSR_Value, sizeof(DCRSR_Value), SWD::TRANSFER_32BIT))
    {
        return false;
    }

    waitForRegisterTransferToComplete();

    return true;
}

static void requestCpuToResume()
{
    uint32_t DHCSR_Val = 0;

    readDHCSR(&DHCSR_Val);

    DHCSR_Val &= ~DHCSR_C_HALT_Bit;

    if (!writeDHCSR(DHCSR_Val))
    {
        logFailure("Failed to clear C_HALT bit in DHCSR.");
    }
}




// *********************************************************************************************************************
// MRI PLATFORM LAYER
//
// Routines needed by the MRI core for platform specific operations. Declarations are in MRI's platforms.h
// *********************************************************************************************************************
// *********************************************************************************************************************
// Implementation of the Platform_Comm* functions.
// Routines used by the MRI core to communicate with GDB. The implementations below use TCP/IP sockets to communicate
// with GDB wirelessly.
// *********************************************************************************************************************
static void waitToReceiveData(void);


uint32_t Platform_CommHasReceiveData(void)
{
    return !g_gdbSocket.m_tcpToMriQueue.isEmpty();
}

uint32_t  Platform_CommHasTransmitCompleted(void)
{
    return (g_gdbSocket.bytesInFlight() == 0);
}

int Platform_CommReceiveChar(void)
{
    waitToReceiveData();

    uint8_t byte = 0;
    uint32_t bytesRead = g_gdbSocket.m_tcpToMriQueue.read(&byte, sizeof(byte));
    (void)bytesRead;
    assert ( bytesRead == sizeof(byte) );

    return (int)byte;
}

static void waitToReceiveData(void)
{
    while (!Platform_CommHasReceiveData())
    {
    }
}

void Platform_CommSendBuffer(Buffer* pBuffer)
{
    g_gdbSocket.send(Buffer_GetArray(pBuffer), Buffer_GetLength(pBuffer));
}

void Platform_CommSendChar(int character)
{
    uint8_t byte = (uint8_t)character;
    g_gdbSocket.send(&byte, sizeof(byte));
}




// *********************************************************************************************************************
// Routine called by the MRI core at init time (as part of handling the mriInit() call in mainDebuggerLoop() above).
// It is used perform any platform specific initialization. In our case we want to initialize the DWT (Data Watchpoint)
// and FPB (Flash Patch and Breakpoint) units on the Cortex-M CPU.
// *********************************************************************************************************************
// Element of the Cortex-M DWT Comparator register array.
struct DWT_COMP_Type
{
    uint32_t comp;
    uint32_t mask;
    uint32_t function;
    uint32_t padding;
};

// Forward declarations for functions used by Platform_Init() to initialize the data watchpoint (DWT) and code
// breakpoint units (called BP on ARMv6M and FPB on ARMv7M) on Cortex-M devices.
static void configureDWTandFPB();
static void enableDWTandITM();
static void initDWT();
static void clearDWTComparators();
static uint32_t getDWTComparatorCount();
static void clearDWTComparator(uint32_t comparatorAddress);
static void initFPB();
static void clearFPBComparators();
static uint32_t getFPBCodeComparatorCount();
static uint32_t readFPControlRegister();
static uint32_t getFPBLiteralComparatorCount();
static void clearFPBComparator(uint32_t comparatorAddress);
static void enableFPB();
static void writeFPControlRegister(uint32_t FP_CTRL_Value);
static void enableHaltingDebug();


void Platform_Init(Token* pParameterTokens)
{
    // UNDONE: I probably need to perform this for each core in a dual core system.
    // UNDONE: Need to enable HALT debugging on the Cortex-M queue.
    configureDWTandFPB();
    enableHaltingDebug();
}

static void configureDWTandFPB()
{
    enableDWTandITM();
    initDWT();
    initFPB();
}

static void enableDWTandITM()
{
    // DEMCR_DWTENA is the name on ARMv6M and it is called TRCENA on ARMv7M.
    const uint32_t DEMCR_DWTENA_Bit = 1 << 24;
    const uint32_t DEMCR_Address = 0xE000EDFC;
    uint32_t DEMCR_Value = 0;

    if (!g_swd.readTargetMemory(DEMCR_Address, &DEMCR_Value, sizeof(DEMCR_Value), SWD::TRANSFER_32BIT))
    {
        logFailure("Failed to read DEMCR register.");
        return;
    }

    DEMCR_Value |= DEMCR_DWTENA_Bit;

    if (!g_swd.writeTargetMemory(DEMCR_Address, &DEMCR_Value, sizeof(DEMCR_Value), SWD::TRANSFER_32BIT))
    {
        logFailure("Failed to set DWTENA/TRCENA bit in DEMCR register.");
        return;
    }
}

static void initDWT()
{
    clearDWTComparators();
}

static void clearDWTComparators()
{
    uint32_t DWT_COMP_Address = 0xE0001020;
    uint32_t comparatorCount = getDWTComparatorCount();
    for (uint32_t i = 0 ; i < comparatorCount ; i++)
    {
        clearDWTComparator(DWT_COMP_Address);
        DWT_COMP_Address += sizeof(DWT_COMP_Type);
    }
}

static uint32_t getDWTComparatorCount()
{
    uint32_t DWT_CTRL_Address = 0xE0001000;
    uint32_t DWT_CTRL_Value = 0;

    if (!g_swd.readTargetMemory(DWT_CTRL_Address, &DWT_CTRL_Value, sizeof(DWT_CTRL_Value), SWD::TRANSFER_32BIT))
    {
        logFailure("Failed to read DWT_CTRL register.");
        return 0;
    }
    return (DWT_CTRL_Value >> 28) & 0xF;
}

static void clearDWTComparator(uint32_t comparatorAddress)
{
    //  Matched.  Read-only.  Set to 1 to indicate that this comparator has been matched.  Cleared on read.
    const uint32_t DWT_COMP_FUNCTION_DATAVMATCH_Bit = 1 << 8;
    //  Cycle Count Match.  Set to 1 for enabling cycle count match and 0 otherwise.  Only valid on comparator 0.
    const uint32_t DWT_COMP_FUNCTION_CYCMATCH_Bit = 1 << 7;
    //  Enable Data Trace Address offset packets.  0 to disable.
    const uint32_t DWT_COMP_FUNCTION_EMITRANGE_Bit = 1 << 5;
    //  Selects action to be taken on match.
    const uint32_t DWT_COMP_FUNCTION_FUNCTION_Mask = 0xF;
    DWT_COMP_Type dwtComp;

    if (!g_swd.readTargetMemory(comparatorAddress, &dwtComp, 3*sizeof(uint32_t), SWD::TRANSFER_32BIT))
    {
        logFailure("Failed to read DWT_COMP/DWT_MASK/DWT_FUNCTION registers for clearing.");
    }

    dwtComp.comp = 0;
    dwtComp.mask = 0;
    dwtComp.function &= ~(DWT_COMP_FUNCTION_DATAVMATCH_Bit |
                                     DWT_COMP_FUNCTION_CYCMATCH_Bit |
                                     DWT_COMP_FUNCTION_EMITRANGE_Bit |
                                     DWT_COMP_FUNCTION_FUNCTION_Mask);

    if (!g_swd.writeTargetMemory(comparatorAddress, &dwtComp, 3*sizeof(uint32_t), SWD::TRANSFER_32BIT))
    {
        logFailure("Failed to write DWT_FUNCTION register for clearing.");
    }
}

static void initFPB()
{
    clearFPBComparators();
    enableFPB();
}

static void clearFPBComparators()
{
    uint32_t currentComparatorAddress = 0xE0002008;
    uint32_t codeComparatorCount = getFPBCodeComparatorCount();
    uint32_t literalComparatorCount = getFPBLiteralComparatorCount();
    uint32_t totalComparatorCount = codeComparatorCount + literalComparatorCount;
    for (uint32_t i = 0 ; i < totalComparatorCount ; i++)
    {
        clearFPBComparator(currentComparatorAddress);
        currentComparatorAddress += sizeof(uint32_t);
    }
}

static uint32_t getFPBCodeComparatorCount()
{
    // Most significant bits of number of instruction address comparators.  Read-only
    const uint32_t FP_CTRL_NUM_CODE_MSB_Shift = 12;
    const uint32_t FP_CTRL_NUM_CODE_MSB_Mask = 0x7 << FP_CTRL_NUM_CODE_MSB_Shift;
    //  Least significant bits of number of instruction address comparators.  Read-only
    const uint32_t FP_CTRL_NUM_CODE_LSB_Shift = 4;
    const uint32_t FP_CTRL_NUM_CODE_LSB_Mask = 0xF << FP_CTRL_NUM_CODE_LSB_Shift;
    uint32_t FP_CTRL_Value = readFPControlRegister();

    return (((FP_CTRL_Value & FP_CTRL_NUM_CODE_MSB_Mask) >> (FP_CTRL_NUM_CODE_MSB_Shift - 4)) |
            ((FP_CTRL_Value & FP_CTRL_NUM_CODE_LSB_Mask) >> FP_CTRL_NUM_CODE_LSB_Shift));
}

static const uint32_t FP_CTRL_Address = 0xE0002000;

static uint32_t readFPControlRegister()
{
    uint32_t FP_CTRL_Value = 0;
    if (!g_swd.readTargetMemory(FP_CTRL_Address, &FP_CTRL_Value, sizeof(FP_CTRL_Value), SWD::TRANSFER_32BIT))
    {
        logFailure("Failed to read FP_CTRL register.");
        return 0;
    }
    return FP_CTRL_Value;
}

static uint32_t getFPBLiteralComparatorCount()
{
    //  Number of instruction literal address comparators.  Read only
    const uint32_t FP_CTRL_NUM_LIT_Shift = 8;
    const uint32_t FP_CTRL_NUM_LIT_Mask = 0xF << FP_CTRL_NUM_LIT_Shift;
    uint32_t FP_CTRL_Value = readFPControlRegister();

    return ((FP_CTRL_Value & FP_CTRL_NUM_LIT_Mask) >> FP_CTRL_NUM_LIT_Shift);
}

static void clearFPBComparator(uint32_t comparatorAddress)
{
    uint32_t comparatorValue = 0;
    if (!g_swd.writeTargetMemory(comparatorAddress, &comparatorValue, sizeof(comparatorValue), SWD::TRANSFER_32BIT))
    {
        logFailure("Failed to write to FP comparator register.");
    }
}

static void enableFPB()
{
    //  This Key field must be set to 1 when writing or the write will be ignored.
    const uint32_t FP_CTRL_KEY = 1 << 1;
    //  Enable bit for the FPB.  Set to 1 to enable FPB.
    const uint32_t FP_CTRL_ENABLE = 1;

    uint32_t FP_CTRL_Value = readFPControlRegister();
    FP_CTRL_Value |= (FP_CTRL_KEY | FP_CTRL_ENABLE);
    writeFPControlRegister(FP_CTRL_Value);
}

static void writeFPControlRegister(uint32_t FP_CTRL_Value)
{
    if (!g_swd.writeTargetMemory(FP_CTRL_Address, &FP_CTRL_Value, sizeof(FP_CTRL_Value), SWD::TRANSFER_32BIT))
    {
        logFailure("Failed to write FP_CTRL register.");
    }
}

static void enableHaltingDebug()
{
    /*  Enable halt mode debug.  Set to 1 to enable halt mode debugging. */
    const uint32_t DHCSR_C_DEBUGEN_Bit = 1 << 0;
    if (!writeDHCSR(DHCSR_C_DEBUGEN_Bit))
    {
        logFailure("Failed to set C_DEBUGEN bit in DHCSR.");
    }
}




// *********************************************************************************************************************
// Routines called by the MRI core each time the CPU is halted and resumed.
// *********************************************************************************************************************
void Platform_EnteringDebugger(void)
{
    g_wasMemoryExceptionEncountered = false;
#ifdef UNDONE
    mriCortexMState.originalPC = Platform_GetProgramCounter();
    Platform_DisableSingleStep();
#endif // UNDONE
}

void Platform_LeavingDebugger(void)
{
    g_wasStopFromGDB = false;
}



// *********************************************************************************************************************
// Routines called by the MRI core to access the storage used for buffering inbound and outbound packets from/to GDB.
// g_packetBuffer must be large enough to contain a 'G' packet sent from GDB to update all of the CPU registers in
// the context at once. This is:
//      1 (byte for 'G' itself) +
//        [ 2 (text hex digits per byte) *
//          4 (bytes per 32-bit word) *
//          56 (registers store in context for CPU w/ FPU) ] +
//      4 (bytes for packet overhead of '$', '#', and 2 hex digit checksum)
//      = 1 + 2 * 4 * 56 + 4 = 453
// *********************************************************************************************************************
// UNDONE: Should I make this larger to maybe improve FLASHing performance?
static char g_packetBuffer[512];

char* Platform_GetPacketBuffer(void)
{
    return g_packetBuffer;
}

uint32_t Platform_GetPacketBufferSize(void)
{
    return sizeof(g_packetBuffer);
}




// *********************************************************************************************************************
// Routine called by the MRI core to create the T response to be sent back to GDB on debug stops.
// *********************************************************************************************************************
static void sendRegisterForTResponse(Buffer* pBuffer, uint8_t registerOffset, uint32_t registerValue);
static void writeBytesToBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount);


void Platform_WriteTResponseRegistersToBuffer(Buffer* pBuffer)
{
    sendRegisterForTResponse(pBuffer, R7, Context_Get(&g_context, R7));
    sendRegisterForTResponse(pBuffer, SP, Context_Get(&g_context, SP));
    sendRegisterForTResponse(pBuffer, LR, Context_Get(&g_context, LR));
    sendRegisterForTResponse(pBuffer, PC, Context_Get(&g_context, PC));
}

static void sendRegisterForTResponse(Buffer* pBuffer, uint8_t registerOffset, uint32_t registerValue)
{
    Buffer_WriteByteAsHex(pBuffer, registerOffset);
    Buffer_WriteChar(pBuffer, ':');
    writeBytesToBufferAsHex(pBuffer, &registerValue, sizeof(registerValue));
    Buffer_WriteChar(pBuffer, ';');
}

static void writeBytesToBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount)
{
    uint8_t* pByte = (uint8_t*)pBytes;
    size_t   i;

    for (i = 0 ; i < byteCount ; i++)
        Buffer_WriteByteAsHex(pBuffer, *pByte++);
}




// *********************************************************************************************************************
// Routines called by the MRI core to read and write memory on the target device.
// *********************************************************************************************************************
// UNDONE: Need a standard way to read and write multiple words at once.
//         Can probably move core/memory.c into here and re-implement to be more efficient.
uint32_t Platform_MemRead32(const void* pv)
{
    g_wasMemoryExceptionEncountered = false;
    uint32_t data = 0;
    uint32_t bytesRead = g_swd.readTargetMemory((uint32_t)pv, &data, sizeof(data), SWD::TRANSFER_32BIT);
    if (bytesRead != sizeof(data))
    {
        g_wasMemoryExceptionEncountered = true;
        return 0;
    }
    return data;
}

uint16_t Platform_MemRead16(const void* pv)
{
    g_wasMemoryExceptionEncountered = false;
    uint16_t data = 0;
    uint32_t bytesRead = g_swd.readTargetMemory((uint32_t)pv, &data, sizeof(data), SWD::TRANSFER_16BIT);
    if (bytesRead != sizeof(data))
    {
        g_wasMemoryExceptionEncountered = true;
        return 0;
    }
    return data;
}

uint8_t Platform_MemRead8(const void* pv)
{
    g_wasMemoryExceptionEncountered = false;
    uint16_t data = 0;
    uint32_t bytesRead = g_swd.readTargetMemory((uint32_t)pv, &data, sizeof(data), SWD::TRANSFER_8BIT);
    if (bytesRead != sizeof(data))
    {
        g_wasMemoryExceptionEncountered = true;
        return 0;
    }
    return data;
}

void Platform_MemWrite32(void* pv, uint32_t value)
{
    g_wasMemoryExceptionEncountered = false;
    uint32_t bytesWritten = g_swd.writeTargetMemory((uint32_t)pv, &value, sizeof(value), SWD::TRANSFER_32BIT);
    if (bytesWritten != sizeof(value))
    {
        g_wasMemoryExceptionEncountered = true;
    }
}

void Platform_MemWrite16(void* pv, uint16_t value)
{
    g_wasMemoryExceptionEncountered = false;
    uint32_t bytesWritten = g_swd.writeTargetMemory((uint32_t)pv, &value, sizeof(value), SWD::TRANSFER_16BIT);
    if (bytesWritten != sizeof(value))
    {
        g_wasMemoryExceptionEncountered = true;
    }
}

void Platform_MemWrite8(void* pv, uint8_t value)
{
    g_wasMemoryExceptionEncountered = false;
    uint32_t bytesWritten = g_swd.writeTargetMemory((uint32_t)pv, &value, sizeof(value), SWD::TRANSFER_8BIT);
    if (bytesWritten != sizeof(value))
    {
        g_wasMemoryExceptionEncountered = true;
    }
}

int Platform_WasMemoryFaultEncountered()
{
    return g_wasMemoryExceptionEncountered;
}

void Platform_SyncICacheToDCache(void *pv, uint32_t size)
{
}




uint32_t Platform_HandleGDBCommand(Buffer* pBuffer)
{
    // UNDONE: Can remove this if we don't end up using it but I think we will for FLASH loading.
    return 0;
}



uint8_t Platform_DetermineCauseOfException(void)
{
    return 0;
}

PlatformTrapReason Platform_GetTrapReason(void)
{
    PlatformTrapReason reason = { MRI_PLATFORM_TRAP_TYPE_UNKNOWN, 0 };
    return reason;
}

void Platform_DisplayFaultCauseToGdbConsole(void)
{
}



void Platform_EnableSingleStep(void)
{
}

void Platform_DisableSingleStep(void)
{
}

int Platform_IsSingleStepping(void)
{
    return false;
}

uint32_t Platform_GetProgramCounter(void)
{
    return 0;
}

void Platform_SetProgramCounter(uint32_t newPC)
{
}

void Platform_AdvanceProgramCounterToNextInstruction(void)
{
}

int Platform_WasProgramCounterModifiedByUser(void)
{
    return 0;
}



/* NOTE: This is the original version of the following XML which has had things stripped to reduce the amount of
         FLASH consumed by the debug monitor.  This includes the removal of the copyright comment.
<?xml version="1.0"?>
<!-- Copyright (C) 2010, 2011 Free Software Foundation, Inc.

     Copying and distribution of this file, with or without modification,
     are permitted in any medium without royalty provided the copyright
     notice and this notice are preserved.  -->

<!DOCTYPE feature SYSTEM "gdb-target.dtd">
<feature name="org.gnu.gdb.arm.m-profile">
  <reg name="r0" bitsize="32"/>
  <reg name="r1" bitsize="32"/>
  <reg name="r2" bitsize="32"/>
  <reg name="r3" bitsize="32"/>
  <reg name="r4" bitsize="32"/>
  <reg name="r5" bitsize="32"/>
  <reg name="r6" bitsize="32"/>
  <reg name="r7" bitsize="32"/>
  <reg name="r8" bitsize="32"/>
  <reg name="r9" bitsize="32"/>
  <reg name="r10" bitsize="32"/>
  <reg name="r11" bitsize="32"/>
  <reg name="r12" bitsize="32"/>
  <reg name="sp" bitsize="32" type="data_ptr"/>
  <reg name="lr" bitsize="32"/>
  <reg name="pc" bitsize="32" type="code_ptr"/>
  <reg name="xpsr" bitsize="32" regnum="25"/>
</feature>
*/
static const char g_targetXML[] =
    "<?xml version=\"1.0\"?>\n"
    "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
    "<target>\n"
    "<feature name=\"org.gnu.gdb.arm.m-profile\">\n"
    "<reg name=\"r0\" bitsize=\"32\"/>\n"
    "<reg name=\"r1\" bitsize=\"32\"/>\n"
    "<reg name=\"r2\" bitsize=\"32\"/>\n"
    "<reg name=\"r3\" bitsize=\"32\"/>\n"
    "<reg name=\"r4\" bitsize=\"32\"/>\n"
    "<reg name=\"r5\" bitsize=\"32\"/>\n"
    "<reg name=\"r6\" bitsize=\"32\"/>\n"
    "<reg name=\"r7\" bitsize=\"32\"/>\n"
    "<reg name=\"r8\" bitsize=\"32\"/>\n"
    "<reg name=\"r9\" bitsize=\"32\"/>\n"
    "<reg name=\"r10\" bitsize=\"32\"/>\n"
    "<reg name=\"r11\" bitsize=\"32\"/>\n"
    "<reg name=\"r12\" bitsize=\"32\"/>\n"
    "<reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\"/>\n"
    "<reg name=\"lr\" bitsize=\"32\"/>\n"
    "<reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>\n"
    "<reg name=\"xpsr\" bitsize=\"32\" regnum=\"25\"/>\n"
    "</feature>\n"
    "<feature name=\"org.gnu.gdb.arm.m-system\">\n"
    "<reg name=\"msp\" bitsize=\"32\" regnum=\"26\"/>\n"
    "<reg name=\"psp\" bitsize=\"32\" regnum=\"27\"/>\n"
    "<reg name=\"primask\" bitsize=\"32\" regnum=\"28\"/>\n"
    "<reg name=\"basepri\" bitsize=\"32\" regnum=\"29\"/>\n"
    "<reg name=\"faultmask\" bitsize=\"32\" regnum=\"30\"/>\n"
    "<reg name=\"control\" bitsize=\"32\" regnum=\"31\"/>\n"
    "</feature>\n"
    "</target>\n";

static const char g_targetFpuXML[] =
    "<?xml version=\"1.0\"?>\n"
    "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
    "<target>\n"
    "<feature name=\"org.gnu.gdb.arm.m-profile\">\n"
    "<reg name=\"r0\" bitsize=\"32\"/>\n"
    "<reg name=\"r1\" bitsize=\"32\"/>\n"
    "<reg name=\"r2\" bitsize=\"32\"/>\n"
    "<reg name=\"r3\" bitsize=\"32\"/>\n"
    "<reg name=\"r4\" bitsize=\"32\"/>\n"
    "<reg name=\"r5\" bitsize=\"32\"/>\n"
    "<reg name=\"r6\" bitsize=\"32\"/>\n"
    "<reg name=\"r7\" bitsize=\"32\"/>\n"
    "<reg name=\"r8\" bitsize=\"32\"/>\n"
    "<reg name=\"r9\" bitsize=\"32\"/>\n"
    "<reg name=\"r10\" bitsize=\"32\"/>\n"
    "<reg name=\"r11\" bitsize=\"32\"/>\n"
    "<reg name=\"r12\" bitsize=\"32\"/>\n"
    "<reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\"/>\n"
    "<reg name=\"lr\" bitsize=\"32\"/>\n"
    "<reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>\n"
    "<reg name=\"xpsr\" bitsize=\"32\" regnum=\"25\"/>\n"
    "</feature>\n"
    "<feature name=\"org.gnu.gdb.arm.m-system\">\n"
    "<reg name=\"msp\" bitsize=\"32\" regnum=\"26\"/>\n"
    "<reg name=\"psp\" bitsize=\"32\" regnum=\"27\"/>\n"
    "<reg name=\"primask\" bitsize=\"32\" regnum=\"28\"/>\n"
    "<reg name=\"basepri\" bitsize=\"32\" regnum=\"29\"/>\n"
    "<reg name=\"faultmask\" bitsize=\"32\" regnum=\"30\"/>\n"
    "<reg name=\"control\" bitsize=\"32\" regnum=\"31\"/>\n"
    "</feature>\n"
    "<feature name=\"org.gnu.gdb.arm.vfp\">\n"
    "<reg name=\"d0\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d1\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d2\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d3\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d4\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d5\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d6\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d7\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d8\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d9\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d10\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d11\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d12\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d13\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d14\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d15\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"fpscr\" bitsize=\"32\" type=\"int\" group=\"float\"/>\n"
    "</feature>\n"
    "</target>\n";

uint32_t Platform_GetTargetXmlSize(void)
{
    // UNDONE: Need to figure out if the chip has a FPU or not.
    return sizeof(g_targetXML);
}

const char* mriPlatform_GetTargetXml(void)
{
    return g_targetXML;
}



// UNDONE: This is the RP2040 memory layout.
static const char g_memoryMapXml[] = "<?xml version=\"1.0\"?>"
                                     "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                     "<memory-map>"
                                     "<memory type=\"rom\" start=\"0x00000000\" length=\"0x4000\"> </memory>"
                                     "<memory type=\"flash\" start=\"0x10000000\" length=\"0x1000000\"> <property name=\"blocksize\">0x1000</property></memory>"
                                     "<memory type=\"ram\" start=\"0x20000000\" length=\"0x42000\"> </memory>"
                                     "</memory-map>";

uint32_t Platform_GetDeviceMemoryMapXmlSize(void)
{
    // UNDONE: How to handle this in general for SWD debugger.
    return sizeof(g_memoryMapXml);
}

const char* Platform_GetDeviceMemoryMapXml(void)
{
    return g_memoryMapXml;
}



__throws void Platform_SetHardwareBreakpointOfGdbKind(uint32_t address, uint32_t kind)
{
}

__throws void Platform_SetHardwareBreakpoint(uint32_t address)
{
}

__throws void Platform_ClearHardwareBreakpointOfGdbKind(uint32_t address, uint32_t kind)
{
}

__throws void Platform_ClearHardwareBreakpoint(uint32_t address)
{
}

__throws void Platform_SetHardwareWatchpoint(uint32_t address, uint32_t size,  PlatformWatchpointType type)
{
}

__throws void Platform_ClearHardwareWatchpoint(uint32_t address, uint32_t size,  PlatformWatchpointType type)
{
}



PlatformInstructionType Platform_TypeOfCurrentInstruction(void)
{
    return MRI_PLATFORM_INSTRUCTION_OTHER;
}

PlatformSemihostParameters Platform_GetSemihostCallParameters(void)
{
    PlatformSemihostParameters param;

    memset(&param, 0, sizeof(param));
    return param;
}

void Platform_SetSemihostCallReturnAndErrnoValues(int returnValue, int errNo)
{
}



void Platform_ResetDevice(void)
{
}


uint32_t Platform_RtosGetHaltedThreadId(void)
{
    return 0;
}

uint32_t Platform_RtosGetFirstThreadId(void)
{
    return 0;
}

uint32_t Platform_RtosGetNextThreadId(void)
{
    return 0;
}

const char* Platform_RtosGetExtraThreadInfo(uint32_t threadId)
{
    return NULL;
}

MriContext* Platform_RtosGetThreadContext(uint32_t threadId)
{
    return NULL;
}

int Platform_RtosIsThreadActive(uint32_t threadId)
{
    return 0;
}

int Platform_RtosIsSetThreadStateSupported(void)
{
    return 0;
}

void Platform_RtosSetThreadState(uint32_t threadId, PlatformThreadState state)
{
}

void Platform_RtosRestorePrevThreadState(void)
{
}



void Platform_HandleFaultFromHighPriorityCode(void)
{
}



int Semihost_IsDebuggeeMakingSemihostCall(void)
{
    return 0;
}

int Semihost_HandleSemihostRequest(void)
{
    return 0;
}
