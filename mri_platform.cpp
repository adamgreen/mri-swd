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

// CPU register context information is stored here.
static uint32_t       g_contextRegisters[registerCountFPU];
static ContextSection g_contextEntriesNoFPU = { .pValues = &g_contextRegisters[0], .count = registerCountNoFPU };
static ContextSection g_contextEntriesFPU = { .pValues = &g_contextRegisters[0], .count = registerCountFPU };
static MriContext     g_context;


// Forward Function Declarations.
static bool initSWD();


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
        // UNDONE: Read DHCSR to see if the CPU has halted. Can also tell us if it has stalled, reset, locked up,
        if (haveGdbStopRequest)
        {
            g_wasStopFromGDB = true;
            // UNDONE: Request CPU halt and once it does halt, enter MRI.
            // UNDONE: Build up real context.
            // UNDONE: Support chips with FPU as well.
            (void)g_contextEntriesFPU;
            mriContext_Init(&g_context, &g_contextEntriesNoFPU, 1);
            mriDebugException(&g_context);
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


void Platform_Init(Token* pParameterTokens)
{
    // UNDONE: I probably need to perform this for each core in a dual core system.
    // UNDONE: Need to enable HALT debugging on the Cortex-M queue.
    configureDWTandFPB();
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




// *********************************************************************************************************************
// Routines called by the MRI core each time the CPU is halted and resumed.
// *********************************************************************************************************************
void Platform_EnteringDebugger(void)
{
    g_wasMemoryExceptionEncountered = false;
#ifdef UNDONE
    mriCortexMState.originalPC = Platform_GetProgramCounter();
    Platform_DisableSingleStep();
    if (isExternalInterrupt(mriCortexMState.exceptionNumber))
        setControlCFlag();
    setActiveDebugFlag();
#endif // UNDONE
}

void Platform_LeavingDebugger(void)
{
    g_wasStopFromGDB = false;
#ifdef UNDONE
    checkStack();
    clearControlCFlag();
    clearActiveDebugFlag();
    clearPendedFromFaultFlag();
    clearMonitorPending();
#endif // UNDONE
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

void Platform_WriteTResponseRegistersToBuffer(Buffer* pBuffer)
{
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
