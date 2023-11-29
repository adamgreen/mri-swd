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
#define PLATFORM_MODULE "mri_platform.cpp"
#include "logging.h"
#include <string.h>
#include <stdio.h>
#include <pico/cyw43_arch.h>
#include "gdb_socket.h"
#include "swd.h"
#include "mri_platform.h"
#include "config.h"
#include "version.h"
#include "devices/devices.h"

// MRI C headers
extern "C"
{
    #include <core/core.h>
    #include <core/mri.h>
    #include <core/packet.h>
    #include <core/platforms.h>
    #include <core/semihost.h>
    #include <core/signal.h>
    #include <core/memory.h>
    #include <core/cmd_common.h>
    #include <core/gdb_console.h>
    #include <core/version.h>
    #include <semihost/newlib/newlib_stubs.h>
}



// Give friendly names to the indices of important registers in the context scatter gather list.
static const uint32_t R0 = 0;
static const uint32_t R1 = 1;
static const uint32_t R2 = 2;
static const uint32_t R3 = 3;
static const uint32_t R7 = 7;
static const uint32_t SP = 13;
static const uint32_t LR = 14;
static const uint32_t PC = 15;
static const uint32_t CPSR = 16;
static const uint32_t MSP = 17;
static const uint32_t PSP = 18;
static const uint32_t PRIMASK = 19;
static const uint32_t BASEPRI = 20;
static const uint32_t FAULTMASK = 21;
static const uint32_t CONTROL = 22;
static const uint32_t S0 = 23;
static const uint32_t FPSCR = S0 + 32;



// Expose debug operations on Cortex-M cores. It can handle targets which have more than one core such as the RP2040.
class CpuCores
{
public:
    CpuCores();

    // Call pHandler function if unrecoverable SWD connection errors are detected at runtime. Handler is typically used
    // to force the MRI core code to exit.
    void setConnectionFailureCallback(void (*pHandler)(void))
    {
        m_connectionFailureHandler = pHandler;
    }

    // Initialize the CPU core objects after establishing a new SWD connection.
    bool init(SWD* pSwdBus);

    // Cleanup the CPU core object after disconnecting the SWD bus. Call init() again later once the SWD
    // connection comes back up.
    void uninit();

    // Power down the SWD Debug Access Ports of all the CPU cores. This reduces power usage on the target
    // device. Similar to uninit() but you aren't really expected to call init() again after calling this method since
    // that would mean the DAP would be powered up again.
    void disconnect();


    // UNDONE: I probably need to perform this for each core in a dual core system.
    // Enables the CPU cores for debugging. This enables vector catch along with the Data Watchpoint
    // and Flash Breakpoint functionality.
    void initForDebugging();

    // Attempt to enable halting debug mode on the CPU cores.
    bool enableHaltDebugging(uint32_t timeout_ms);

    // UNDONE: Take the index of a core to skip, the one which is already halted.
    // Request the CPU cores to halt.
    bool requestCpuToHalt();

    // Does the device have a FPU?
    enum FpuDiscoveryStates
    {
        // We know for sure that the device doesn't have a FPU.
        FPU_NONE = 0,
        // Cortex-M4 or higher MIGHT have a FPU. Will need to query CPACR on first halt to know for sure.
        FPU_MAYBE,
        // We now know for sure that the device has a FPU.
        FPU_AVAILABLE,
        // We later determined the device doesn't have a FPU but target XML indicates that it does.
        FPU_NOT_AVAILABLE
    };
    FpuDiscoveryStates hasFPU()
    {
        return m_fpu;
    }

    // UNDONE: Which core?
    //         Maybe take an array, one for each core. The isDeviceResetting/isCpuHalted could then take this array and
    //         return the lowest index found to match the requested state and -1 if none match.
    // Retrieve the state of the CPU core.
    // Call isDeviceResetting() or isCpuHalted() on the *pState value to determine which of these state, if any, are
    // currently being experienced by the CPU.
    typedef uint32_t CpuState;
    bool getCpuState(CpuState* pState, uint32_t timeout_ms)
    {
        return readDHCSRWithRetry(pState, timeout_ms);
    }
    bool isDeviceResetting(CpuState state)
    {
        const uint32_t S_RESET_ST_Bit = 1 << 25;

        return !!(state & S_RESET_ST_Bit);
    }
    bool isCpuHalted(CpuState state)
    {
        const uint32_t DHCSR_S_HALT_Bit = 1 << 17;
        return !!(state & DHCSR_S_HALT_Bit);
    }

    // Retrieve a string pointer and the length for the XML describing of the memory layout of this device.
    const char* getMemoryLayoutXML()
    {
        return m_pMemoryLayoutXML;
    }
    size_t getMemoryLayoutXMLSize()
    {
        return m_memoryLayoutSize;
    }


    // UNDONE: Probably need a method to mark one of the cores as halted unless we want isCpuHalted() to do it for us.
    //         Probably makes the most sense to pass it directly into this method.
    // Enter the MRI core by calling mriDebugException() after grabbing the required register context of the halted
    // core.
    void enterMriCore();

    // Typically called from Platform_EnteringDebugger() to let this object know that the MRI core was just entered.
    // It caches the reason (interrupt from GDB, breakpoint, watchpoint, etc) for the halting core and makes sure that
    // reset vector catch is disabled for all of the core.
    void enteringDebugger();

    // Typically called from Platform_LeavingDebugger() to let this object know that the MRI core is about to exit and
    // allow execution to be resumed. It makes sure that the Debug Fault Status Register (DFSR) is cleared since we no
    // longer need to know about the cause of the last halt.
    void leavingDebugger();

    // Ask execution to resume on all of the cores. It also takes care of restoring any modified register context before
    // resuming.
    void resume();

    // Ask the cores to reset.
    void reset();

    // Ask the cores to halt immediately after coming out of reset. Should be called before reset() if this is the
    // desired behaviour.
    void enableResetVectorCatch();

    // Should be called as soon as the main loop determines that a reset has occurred so that the object can
    // re-initialize the information it has about all of the cores after their reset.
    void resetCompleted()
    {
        checkForMultipleCores();
    }


    // UNDONE: Will want to add a class to wrap the context object to track when each core's context is stale and needs
    //         to be read, and when it is dirty and needs to be saved back to CPU.
    // Read the requested register on the halted core.
    uintmri_t readRegisterOnHaltedCore(size_t index)
    {
        return Context_Get(&m_context, index);
    }

    // Write a new value to the requested register on the halted core.
    void writeRegisterOnHaltedCore(size_t index, uintmri_t newValue)
    {
        Context_Set(&m_context, index, newValue);
    }

    // Read from the requested memory range in FLASH, ROM, or RAM. Not to be used for reading memory mapped registers as
    // those reads need to be directed at a specific core.
    uint32_t readMemory(uint32_t address, void* pvBuffer, uint32_t bufferSize, SWD::TransferSize readSize)
    {
        // Make sure that we don't try to read registers with this method.
        assert ( address < 0xA0000000 );
        return readTargetMemory(address, pvBuffer, bufferSize, readSize);
    }

    // Write to the requested memory range in FLASH, ROM, or RAM. Not to be used for writing memory mapped registers as
    // those writes need to be directed at a specific core.
    uint32_t writeMemory(uint32_t address, const void* pvBuffer, uint32_t bufferSize, SWD::TransferSize writeSize)
    {
        // Make sure that we don't try to write registers with this method.
        assert ( address < 0xA0000000 );
        return writeTargetMemory(address, pvBuffer, bufferSize, writeSize);
    }


    // UNDONE: Might just ignore this for multithread and use thread state instead.
    // Enable single stepping.
    void enableSingleStep()
    {
        configureSingleSteppingBitsInDHCSR(true);
    }

    // Disable single stepping.
    void disableSingleStep()
    {
        configureSingleSteppingBitsInDHCSR(false);
    }

    // Is single stepping enabled?
    bool isSingleStepping()
    {
        uint32_t DHCSR_Val = 0;
        readDHCSR(&DHCSR_Val);
        return !!(DHCSR_Val & DHCSR_C_STEP_Bit);
    }

    // Set a breakpoint at the requested address (16 or 32-bit instruction). Returns the 32-bit address of the
    // comparator register used for this breakpoint, 0 if no free ones were found. If the requested breakpoint is
    // already set then the address of the already set comparator register will be returned.
    uint32_t setBreakpoint(uint32_t breakpointAddress, bool is32BitInstruction);

    // Clear a breakpoint at the requested address (16 or 32-bit instruction). Returns the 32-bit address of the
    // comparator register just cleared for this breakpoint, 0 if no matching breakpoint for the address was found.
    uint32_t clearBreakpoint(uint32_t breakpointAddress, bool is32BitInstruction);

    // Set a watchpoint (read and/or write) at the specified memory range.
    // Can throw invalidArgumentException if the address, size, or type aren't supported for Cortex-M devices.
    // Can throw exceededHardwareResourcesException if all of the watchpoint comparators are already used up.
    // These exceptions will be caught and handled by the MRI core.
    __throws void setWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type);

    // Clear a watchpoint (read and/or write) at the specified memory range.
    // Can throw invalidArgumentException if the address, size, or type aren't supported for Cortex-M devices.
    // These exceptions will be caught and handled by the MRI core.
    __throws void clearWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type);


    // Returns the signal value (SIGINT, SIGTRAP, etc) most appropriate for the reason that the CPU was halted. This is
    // returned to GDB and displayed to the user.
    //  wasStopFromGDB parameter indicates whether the target was interrupted by request of GDB.
    uint8_t determineCauseOfException(bool wasStopFromGDB);

    // Returns whether the CPU was halted because of a breakpoint, watchpoint, or some other reason. This is returned to
    // GDB and used to display which breakpoint or watchpoint, if any, caused the halt to occur. This returned reason
    // was actually determined earlier when enteringDebugger() was called because watchpoint hit state can only be
    // queried once.
    PlatformTrapReason getTrapReason();

    // If an ARMv7M microcontroller hits a fault while running under the debugger, this method can be called to display
    // information about the type of fault encountered to the GDB console before halting.
    void displayFaultCauseToGdbConsole();

    // Called when user enters "monitor ..." commands in GDB. They are passed into the device specific driver, if one
    // exists, to let it have first shot at handling the requested command. Returns true if handled by the device
    // specific driver and false otherwise.
    bool dispatchMonitorCommandToDevice(const char** ppArgs, size_t argCount);


    // Returns true if there is a device specific driver that knows how to program the FLASH on the currently connected
    // target and false otherwise.
    bool supportsFlashProgramming()
    {
        return m_pDevice != NULL;
    }

    // The following flash*() methods are redirected to the device specific driver to handle programming of the FLASH on
    // the currently connected target.
    bool flashBegin();
    bool flashErase(uint32_t addressStart, uint32_t length);
    bool flashProgram(uint32_t addressStart, const void* pBuffer, size_t bufferSize);
    bool flashEnd();


    // Temporarily sets the registers on the default core to the specified values and then resumes execution on that
    // core. This can be used to set the PC, SP, LR (and any other required registers) to execute arbitrary code that
    // already exists in the ROM of the device or was previously loaded into RAM using writeMemory() calls. Can call
    // waitForCodeToHalt() to know when it has completed.
    bool startCodeOnDevice(const CortexM_Registers* pRegistersIn);

    // Waits for code previously started by startCodeOnDevice() to complete. The code should signal its completion by
    // executing a hardcoded BKPT instruction. The values of the registers at the time of the BKPT can be found in the
    // pRegistersOut structure.
    bool waitForCodeToHalt(CortexM_Registers* pRegistersOut, uint32_t timeout_ms);

    // This combines calls to startCodeOnDevice() and waitForCodeToWait() in one call.
    bool runCodeOnDevice(CortexM_Registers* pRegistersInOut, uint32_t timeout_ms);

protected:
    bool initSWD(SWD* pSwdBus);
    void searchSupportedDevicesList();
    void determineFpuAvailability();
    void updateSwdFrequencyForDevice();
    void constructMemoryLayoutXML();
    void checkForMultipleCores();
    void cleanupDeviceObject();
    void checkForFpu();
    uint32_t readTargetMemory(uint32_t address, void* pvBuffer, uint32_t bufferSize, SWD::TransferSize readSize);
    void handleUnrecoverableSwdError();
    uint32_t writeTargetMemory(uint32_t address, const void* pvBuffer, uint32_t bufferSize, SWD::TransferSize writeSize);
    void saveContext();
    bool readCpuRegister(uint32_t registerIndex, uint32_t* pValue);
    void waitForRegisterTransferToComplete();
    void restoreContext();
    bool writeCpuRegister(uint32_t registerIndex, uint32_t value);
    bool readDHCSR(uint32_t* pValue);
    static bool hasRegisterTransferCompleted(uint32_t DHCSR_Value);
    bool readDHCSRWithRetry(uint32_t* pValue, uint32_t timeout_ms);
    bool writeDHCSR(uint32_t DHCSR_Value);
    bool requestCpuToResume();
    void enableDWTandVectorCatches();
    bool setOrClearBitsInDEMCR(uint32_t bitMask, bool set);
    void initDWT();
    uint32_t clearDWTComparators();
    uint32_t getDWTComparatorCount();
    void clearDWTComparator(uint32_t comparatorAddress);
    void initFPB();
    uint32_t clearFPBComparators();
    uint32_t getFPBCodeComparatorCount();
    uint32_t readFPControlRegister();
    uint32_t getFPBLiteralComparatorCount();
    void clearFPBComparator(uint32_t comparatorAddress);
    void enableFPB();
    void writeFPControlRegister(uint32_t FP_CTRL_Value);
    PlatformTrapReason cacheTrapReason();
    bool readDFSR(uint32_t* pDFSR);
    PlatformTrapReason findMatchedWatchpoint(void);
    PlatformTrapReason getReasonFromMatchComparator(uint32_t comparatorAddress, uint32_t function);
    void disableResetVectorCatch();
    void clearDFSR();
    bool writeDFSR(uint32_t dfsr);
    void configureSingleSteppingBitsInDHCSR(bool enableSingleStepping);
    uint32_t findFPBBreakpointComparator(uint32_t breakpointAddress, bool is32BitInstruction);
    uint32_t calculateFPBComparatorValue(uint32_t breakpointAddress, bool is32BitInstruction);
    bool isBreakpointAddressInvalid(uint32_t breakpointAddress);
    uint32_t getFPBRevision();
    static bool isAddressOdd(uint32_t address);
    static bool isAddressAboveLowestHalfGig(uint32_t address);
    static uint32_t calculateFPBComparatorValueRevision1(uint32_t breakpointAddress, bool is32BitInstruction);
    static uint32_t calculateFPBComparatorReplaceValue(uint32_t breakpointAddress, bool is32BitInstruction);
    static bool isAddressInUpperHalfword(uint32_t address);
    static uint32_t calculateFPBComparatorValueRevision2(uint32_t breakpointAddress);
    uint32_t maskOffFPBComparatorReservedBits(uint32_t comparatorValue);
    uint32_t findFreeFPBBreakpointComparator();
    bool isFPBComparatorEnabled(uint32_t comparator);
    static bool isFPBComparatorEnabledRevision1(uint32_t comparator);
    static bool isFPBComparatorEnabledRevision2(uint32_t comparator);
    static uint32_t convertWatchpointTypeToCortexMType(PlatformWatchpointType type);
    static bool isValidDWTComparatorSetting(uint32_t watchpointAddress,
                                            uint32_t watchpointSize,
                                            uint32_t watchpointType);
    static bool isValidDWTComparatorSize(uint32_t watchpointSize);
    static bool isPowerOf2(uint32_t value);
    static bool isValidDWTComparatorAddress(uint32_t watchpointAddress, uint32_t watchpointSize);
    static bool isAddressAlignedToSize(uint32_t address, uint32_t size);
    static bool isValidDWTComparatorType(uint32_t watchpointType);
    uint32_t enableDWTWatchpoint(uint32_t watchpointAddress,
                                 uint32_t watchpointSize,
                                 uint32_t watchpointType);
    uint32_t findDWTComparator(uint32_t watchpointAddress,
                               uint32_t watchpointSize,
                               uint32_t watchpointType);
    bool doesDWTComparatorMatch(uint32_t comparatorAddress,
                                uint32_t address,
                                uint32_t size,
                                uint32_t function);
    bool doesDWTComparatorFunctionMatch(uint32_t comparatorAddress, uint32_t function);
    static uint32_t maskOffDWTFunctionBits(uint32_t functionValue);
    bool doesDWTComparatorAddressMatch(uint32_t comparatorAddress, uint32_t address);
    bool doesDWTComparatorMaskMatch(uint32_t comparatorAddress, uint32_t size);
    static uint32_t calculateLog2(uint32_t value);
    uint32_t findFreeDWTComparator();
    bool isDWTComparatorFree(uint32_t comparatorAddress);
    bool attemptToSetDWTComparator(uint32_t comparatorAddress,
                                   uint32_t watchpointAddress,
                                   uint32_t watchpointSize,
                                   uint32_t watchpointType);
    bool attemptToSetDWTComparatorMask(uint32_t comparatorAddress, uint32_t watchpointSize);
    uint32_t disableDWTWatchpoint(uint32_t watchpointAddress,
                                  uint32_t watchpointSize,
                                  uint32_t watchpointType);
    uint32_t getExceptionNumber();
    void displayHardFaultCauseToGdbConsole();
    void displayMemFaultCauseToGdbConsole();
    void displayBusFaultCauseToGdbConsole();
    void displayUsageFaultCauseToGdbConsole();
    bool disableSingleStepAndInterrupts();
    bool reenableInterrupts();



    // The number of special registers (msp, psp, primask, basepri, faultmask, and control) is 6.
    static const uint32_t specialRegisterCount = 6;
    // The number of integer registers (R0-R12,SP,LR,PC,XPSR) is 17.
    static const uint32_t integerRegisterCount = 17;
    // The number of float registers (S0-S32, FPSCR) is 33.
    static const uint32_t floatRegisterCount = 33;
    // The number of registers in the CPU content depends on whether the device has a FPU or not.
    static const uint32_t registerCountNoFPU = integerRegisterCount + specialRegisterCount;
    static const uint32_t registerCountFPU = registerCountNoFPU + floatRegisterCount;

    // Debug Core Register Selector Register.
    static const uint32_t DCRSR_Address = 0xE000EDF4;
    // Specifies the access type for the transfer: 0 for read, 1 for write.
    static const uint32_t DCRSR_REGWnR_Bit = 1 << 16;
    // Specifies the ARM core register, special-purpose register, or Floating-point Extension register, to transfer.
    static const uint32_t DCRSR_REGSEL_Mask = 0x7F;

    // Important DCRSR register indices.
    //   The special registers: CONTROL, FAULTMASK, BASEPRI, and PRIMASK are accessed at this single DCRSR index.
    static const uint32_t dcrsrSpecialRegisterIndex = 0x14;
    //   The floating point registers are found at this indices in the DCRSR.
    static const uint32_t dcrsrFpscrIndex = 0x21;
    static const uint32_t dcrsrS0Index = 0x40;

    // Debug Core Register Data Register.
    static const uint32_t DCRDR_Address = 0xE000EDF8;

    // Debug Halting Control and Status Register.
    static const uint32_t DHCSR_Address = 0xE000EDF0;
    // Bit in DHCSR used to halt the CPU.
    static const uint32_t DHCSR_C_HALT_Bit = 1 << 1;
    // Bit in DHCSR used to single step the CPU.
    static const uint32_t DHCSR_C_STEP_Bit = 1 << 2;
    // Bit in the DHCSR to mask (disable) interrupts.
    static const uint32_t DHCSR_C_MASKINTS_Bit = 1 << 3;

    // Bits in DWT register which selects action to be taken on match.
    static const uint32_t DWT_COMP_FUNCTION_FUNCTION_Mask = 0xF;

    // Address of Flash Patch Control register.
    static const uint32_t FP_CTRL_Address = 0xE0002000;
    // Flash Patch breakpoint architecture revision. 0 for revision 1 and 1 for revision 2.
    static const uint32_t FP_CTRL_REVISION2 = 0x1;

    // Address of Flash Patch Comparator Array.
    static const uint32_t FPB_COMP_ARRAY_Address = 0xE0002008;
    //  Specified bits 28:2 of the address to be use for match on this comparator.
    static const uint32_t FP_COMP_COMP_Shift = 2;
    static const uint32_t FP_COMP_COMP_Mask = (0x07FFFFFF << FP_COMP_COMP_Shift);
    //  Enables this comparator.  Set to 1 to enable.
    static const uint32_t FP_COMP_ENABLE_Bit = 1 << 0;
    // FlashPatch Comparator Register Bits for revision 2.
    //  Enables this comparator for flash patching when FP_COMP_BE is 0. Set to 1 to enable.
    static const uint32_t FP_COMP_FE_Bit = 1 << 31;
    //  Enables this comparator as a breakpoint.  Set to 1 to enable.
    static const uint32_t FP_COMP_BE_Bit = 1 << 0;

    // Debug Fault Status Register, DFSR
    static const uint32_t DFSR_Address = 0xE000ED30;
    // Indicates an asynchronous debug event generated because of EDBGRQ being asserted.
    static const uint32_t DFSR_EXTERNAL_Bit = 1 << 4;
    // Indicates whether a vector catch debug event was generated.
    static const uint32_t DFSR_VCATCH_Bit = 1 << 3;
    // Indicates a debug event generated by the DWT.
    static const uint32_t DFSR_DWTTRAP_Bit = 1 << 2;
    // Indicates a debug event generated by a BKPT instruction or breakpoint match in the DWT.
    static const uint32_t DFSR_BKPT_Bit = 1 << 1;
    // Indicates a debug event generated by a C_HALT or C_STEP request, triggered by a write to the DHCSR.
    static const uint32_t DFSR_HALTED_Bit = 1 << 0;

    // Base address of the DWT comp register array.
    static const uint32_t DWT_COMP_ARRAY_Address = 0xE0001020;

    //  Matched.  Read-only.  Set to 1 to indicate that this comparator has been matched.  Cleared on read.
    static const uint32_t DWT_COMP_FUNCTION_MATCHED = 1 << 24;

    //  Selects action to be taken on match.
    //      Data Read Watchpoint
    static const uint32_t DWT_COMP_FUNCTION_FUNCTION_DATA_READ = 0x5;
    //      Data Write Watchpoint
    static const uint32_t DWT_COMP_FUNCTION_FUNCTION_DATA_WRITE = 0x6;
    //      Data Read/Write Watchpoint
    static const uint32_t DWT_COMP_FUNCTION_FUNCTION_DATA_READWRITE = 0x7;

    // Enable Reset Vector Catch. This bit in the DEMCR causes a Local reset to halt a running system.
    static const uint32_t DEMCR_VC_CORERESET_Bit = 1 << 0;

    // The bits in the XPSR register used to indicate the IPSR or exception number.
    // Looks like this could be 9 bits on ARMv7-M or 6 bits on ARMv6-M. User the smaller count here.
    static const uint32_t IPSR_Mask = (1 << 6) - 1;

    // Configurable Status Register Address.
    static const uint32_t CFSR_Address = 0xE000ED28;



    // Element of the Cortex-M DWT Comparator register array.
    struct DWT_COMP_Type
    {
        uint32_t comp;
        uint32_t mask;
        uint32_t function;
        uint32_t padding;
    };



    // UNDONE: Need one of these for each core. Also need to track down whether it is valid and maybe whether it is
    //         dirty or not.
    // CPU register context information is stored here.
    uintmri_t      m_contextRegisters[registerCountFPU];
    ContextSection m_contextEntriesNoFPU = { .pValues = &m_contextRegisters[0], .count = registerCountNoFPU };
    ContextSection m_contextEntriesFPU = { .pValues = &m_contextRegisters[0], .count = registerCountFPU };
    MriContext     m_context;

    // UNDONE: Call target count, coreCount;
    SWD*       m_pSwdBus = NULL;
    SwdTarget  m_swdTargets[MAX_DPV2_TARGETS];
    size_t     m_swdTargetCount = 0;

    // UNDONE: Track indices instead for halting core, default core, and current core (not sure I need this last one).
    SwdTarget* m_pSWD;

    bool       m_hasDetectedSwdDisconnect = false;

    // Cache the reason if breakpoint or watchpoint caused core to halt from the enteringDebugger() method. It is cached
    // because the bit read to determine which watchpoint was hit, if any, is cleared on read.
    PlatformTrapReason m_trapReason;

    FpuDiscoveryStates m_fpu = FPU_NONE;

    // When mri-swd has support (including FLASH programming) for the attached device, then these globals will store
    // pointers to its function table and object data.
    DeviceFunctionTable* m_pDevice = NULL;
    DeviceObject*        m_pDeviceObject = NULL;

    // Used as storage for the device memory layout XML.
    char*   m_pMemoryLayoutXML = NULL;
    int32_t m_memoryLayoutAllocSize = 0;
    int32_t m_memoryLayoutSize = 0;

    // Callback to call when SWD connection errors are detected.
    void (*m_connectionFailureHandler)(void);
};

CpuCores::CpuCores()
{
}

bool CpuCores::init(SWD* pSwdBus)
{
    // Don't call init() again without first calling uninit() to cleanup from the previous SWD connection.
    assert ( m_swdTargetCount == 0 );

    if (!initSWD(pSwdBus))
    {
        return false;
    }
    m_pSwdBus = pSwdBus;
    searchSupportedDevicesList();
    determineFpuAvailability();
    updateSwdFrequencyForDevice();
    constructMemoryLayoutXML();
    checkForMultipleCores();
    m_hasDetectedSwdDisconnect = false;
    assert ( m_swdTargetCount > 0 );

    return true;
}

bool CpuCores::initSWD(SWD* pSwdBus)
{
    logInfo("Initializing target's debug components...");
    bool result = pSwdBus->initTargetForDebugging(m_swdTargets[0]);
    if (!result)
    {
        logError("Failed to initialize target's debug components.");
        return false;
    }
    m_pSWD = &m_swdTargets[0];
    m_swdTargetCount = 1;
    return true;
}

void CpuCores::searchSupportedDevicesList()
{
    assert ( m_pDevice == NULL && m_pDeviceObject == NULL );
    for (size_t i = 0; i < g_supportedDevicesLength ; i++)
    {
        DeviceFunctionTable* pDevice = g_supportedDevices[i];
        DeviceObject* pObject = pDevice->detect(m_pSWD);
        if (pObject)
        {
            m_pDevice = pDevice;
            m_pDeviceObject = pObject;
            logInfoF("Found device of type %s.", pDevice->getName(m_pDeviceObject, m_pSWD));
            break;
        }
    }
    if (!m_pDevice)
    {
        logInfo("Unrecognized device type. FLASH programming not supported!");
    }
}

void CpuCores::determineFpuAvailability()
{
    if (m_pDevice)
    {
        // Device driver will tell us explicitly if this device support FPU of not.
        if (m_pDevice->hasFpu(m_pDeviceObject, m_pSWD))
        {
            logInfo("Device has FPU.");
            m_fpu = FPU_AVAILABLE;
        }
        else
        {
            m_fpu = FPU_NONE;
        }
    }
    else
    {
        // Don't know for sure if the device has FPU at init time so will have to determine on the fly.
        if (m_pSWD->getCpuType() >= SWD::CPU_CORTEX_M4)
        {
            // Device might have FPU. Check on first halt.
            m_fpu = FPU_MAYBE;
        }
        else
        {
            // Cortex-M3 and lower never have a FPU.
            m_fpu = FPU_NONE;
        }
    }
}

void CpuCores::updateSwdFrequencyForDevice()
{
    if (!m_pDevice)
    {
        // Use the initial/default SWD speed if the current target not found in the supported devices list.
        return;
    }

    // Use the supported device object to determine the maximum SWD frequency supported by this device.
    uint32_t swdFrequency = m_pDevice->getMaximumSWDClockFrequency(m_pDeviceObject, m_pSWD);
    if (!m_pSWD->setFrequency(swdFrequency))
    {
        logErrorF("Failed to re-initialize the SWD port to a frequency of %lu.", swdFrequency);
        return;
    }
    logInfoF("Configured SWD frequency to %lu.", swdFrequency);
}


void CpuCores::constructMemoryLayoutXML()
{
    const DeviceMemoryLayout* pLayout;
    if (m_pDevice)
    {
        // Devices on the supported list can report their memory layout.
        pLayout = m_pDevice->getMemoryLayout(m_pDeviceObject, m_pSWD);
    }
    else
    {
        // Fall back to the default memory layout used by all Cortex-M devices. It will report regions that are larger
        // than those actually found on the device but it does know which regions are read-only and which are
        // read/write.
        pLayout = deviceDefaultMemoryLayout(NULL, m_pSWD);
    }
    assert ( pLayout );

    const char xmlHeader[] = "<?xml version=\"1.0\"?>"
                             "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                             "<memory-map>";
    const char xmlFooter[] = "</memory-map>";
    const char xmlLongestRegion[] = "<memory type=\"flash\" start=\"0x12345678\" length=\"0x12345678\"> <property name=\"blocksize\">0x12345678</property></memory>";

    // Calculate the maximum number of bytes needed for the full memory layout XML string and make sure that
    // m_pMemoryLayoutXML is at least this large.
    int32_t bytesRequired = sizeof(xmlHeader) + pLayout->regionCount * sizeof(xmlLongestRegion) + sizeof(xmlFooter);
    if (bytesRequired > m_memoryLayoutAllocSize)
    {
        char* pAlloc = (char*)realloc(m_pMemoryLayoutXML, bytesRequired);
        if (!pAlloc)
        {
            logErrorF("Failed to allocate %d bytes for memory layout XML.", bytesRequired);
            return;
        }
        m_pMemoryLayoutXML = pAlloc;
        m_memoryLayoutAllocSize = bytesRequired;
    }

    // Add the XML header.
    int32_t bytesLeft = m_memoryLayoutAllocSize;
    char* pDest = m_pMemoryLayoutXML;
    memcpy(pDest, xmlHeader, sizeof(xmlHeader)-1);
    bytesLeft -= sizeof(xmlHeader) - 1;
    pDest += sizeof(xmlHeader) - 1;

    // Add a line for each region in the device's memory layout.
    for (size_t i = 0 ; i < pLayout->regionCount ; i++)
    {
        int bytesUsed = 0;
        const DeviceMemoryRegion* pRegion = &pLayout->pRegions[i];

        switch (pRegion->type)
        {
            case DEVICE_MEMORY_ROM:
                bytesUsed = snprintf(pDest, bytesLeft,
                                     "<memory type=\"rom\" start=\"0x%lX\" length=\"0x%lX\"></memory>",
                                     pRegion->address, pRegion->length);
                break;
            case DEVICE_MEMORY_FLASH:
                bytesUsed = snprintf(pDest, bytesLeft,
                                     "<memory type=\"flash\" start=\"0x%lX\" length=\"0x%lX\"><property name=\"blocksize\">0x%lX</property></memory>",
                                     pRegion->address, pRegion->length, pRegion->blockSize);
                break;
            case DEVICE_MEMORY_RAM:
                bytesUsed = snprintf(pDest, bytesLeft,
                                     "<memory type=\"ram\" start=\"0x%lX\" length=\"0x%lX\"></memory>",
                                     pRegion->address, pRegion->length);
                break;
            default:
                assert ( false );
                break;
        }

        assert ( bytesUsed > 0 && bytesUsed < bytesLeft );
        bytesLeft -= bytesUsed;
        pDest += bytesUsed;
    }

    // Add the XML footer, including the NULL terminator.
    assert ( bytesLeft >= (int32_t)sizeof(xmlFooter) );
    memcpy(pDest, xmlFooter, sizeof(xmlFooter));
    bytesLeft -= sizeof(xmlFooter);

    // Record the final size of the XML string, including the NULL terminator.
    m_memoryLayoutSize = m_memoryLayoutAllocSize - bytesLeft;
}

void CpuCores::checkForMultipleCores()
{
    if (!m_pDevice)
    {
        return;
    }

    size_t targetCount = 0;
    if (!m_pDevice->getAdditionalTargets(m_pDeviceObject, m_pSWD, &m_swdTargets[1], count_of(m_swdTargets)-1, &targetCount))
    {
        logError("Failed calling m_pDevice->getAdditionalTargets()");
        return;
    }
    assert ( targetCount <= count_of(m_swdTargets)-1 );
    m_swdTargetCount = targetCount + 1;
    logInfoF("Device has %u core(s).", m_swdTargetCount);
}

void CpuCores::uninit()
{
    cleanupDeviceObject();
    if (m_pSwdBus)
    {
        // Send line reset to main SWD object to make sure that it forgets about the currently connected target.
        m_pSwdBus->sendLineReset();
        m_pSwdBus = NULL;
    }
    for (size_t i = 0 ; i < m_swdTargetCount ; i++)
    {
        m_swdTargets[i].uninit();
    }
    m_swdTargetCount = 0;
    m_pSWD = NULL;
}

void CpuCores::cleanupDeviceObject()
{
    if (!m_pDevice)
    {
        return;
    }

    m_pDevice->free(m_pDeviceObject, m_pSWD);
    m_pDeviceObject = NULL;
    m_pDevice = NULL;
}

void CpuCores::disconnect()
{
    cleanupDeviceObject();
    for (size_t i = 0 ; i < m_swdTargetCount ; i++)
    {
        m_swdTargets[i].disconnect();
        m_swdTargets[i].uninit();
    }
    m_swdTargetCount = 0;
    m_pSWD = NULL;
    m_pSwdBus = NULL;
}

void CpuCores::enterMriCore()
{
    checkForFpu();
    saveContext();
    mriDebugException(&m_context);
}

void CpuCores::resume()
{
    restoreContext();
    requestCpuToResume();
}

void CpuCores::checkForFpu()
{
    if (m_fpu != FPU_MAYBE)
    {
        // We already know for sure whether there is a FPU on this device or not so just return.
        return;
    }

    const uint32_t CPACR_Address = 0xE000ED88;
    const uint32_t CPACR_CP10_Shift = 20;
    const uint32_t CPACR_CP10_Mask = 0xF << CPACR_CP10_Shift;
    const uint32_t CPACR_CP11_Shift = 22;
    const uint32_t CPACR_CP11_Mask = 0xF << CPACR_CP11_Shift;

    uint32_t CPACR_OrigValue = 0;
    if (readTargetMemory(CPACR_Address, &CPACR_OrigValue, sizeof(CPACR_OrigValue), SWD::TRANSFER_32BIT) != sizeof(CPACR_OrigValue))
    {
        logError("Failed to read CPACR register.");
        // On error, assume that FPU doesn't exist.
        m_fpu = FPU_NOT_AVAILABLE;
        return;
    }
    if ((CPACR_OrigValue & CPACR_CP10_Mask) && (CPACR_OrigValue & CPACR_CP11_Mask))
    {
        // FPU exists and is enabled.
        logInfo("Device has FPU.");
        m_fpu = FPU_AVAILABLE;
        return;
    }

    // Try enabling the FPU.
    uint32_t CPACR_TestValue = CPACR_CP10_Mask | CPACR_CP11_Mask | CPACR_OrigValue;
    if (writeTargetMemory(CPACR_Address, &CPACR_TestValue, sizeof(CPACR_TestValue), SWD::TRANSFER_32BIT) != sizeof(CPACR_TestValue))
    {
        logError("Failed to test CPACR register.");
        // On error, assume that FPU doesn't exist.
        m_fpu = FPU_NOT_AVAILABLE;
        return;
    }

    // See if the FPU enable took. If so then the FPU exists.
    uint32_t CPACR_UpdatedValue = ~CPACR_TestValue;
    if (readTargetMemory(CPACR_Address, &CPACR_UpdatedValue, sizeof(CPACR_UpdatedValue), SWD::TRANSFER_32BIT) != sizeof(CPACR_UpdatedValue))
    {
        logError("Failed to verify CPACR register.");
        // On error, assume that FPU doesn't exist.
        m_fpu = FPU_NOT_AVAILABLE;
    }
    if (CPACR_UpdatedValue == CPACR_TestValue)
    {
        logInfo("Device has FPU.");
        m_fpu = FPU_AVAILABLE;
    }

    // Restore CPACR Value.
    if (writeTargetMemory(CPACR_Address, &CPACR_OrigValue, sizeof(CPACR_OrigValue), SWD::TRANSFER_32BIT) != sizeof(CPACR_OrigValue))
    {
        logError("Failed to restore CPACR register.");
    }
}

uint32_t CpuCores::readTargetMemory(uint32_t address, void* pvBuffer, uint32_t bufferSize, SWD::TransferSize readSize)
{
    if (m_hasDetectedSwdDisconnect || bufferSize == 0)
    {
        return 0;
    }

    uint32_t bytesRead = m_pSWD->readMemory(address, pvBuffer, bufferSize, readSize);
    if (bytesRead == 0 && m_pSWD->getLastReadWriteError() == SWD::SWD_PROTOCOL)
    {
        handleUnrecoverableSwdError();
    }
    return bytesRead;
}

void CpuCores::handleUnrecoverableSwdError()
{
    logErrorF("Encountered unrecoverable read/write error %d.", m_pSWD->getLastReadWriteError());
    m_hasDetectedSwdDisconnect = true;
    if (m_connectionFailureHandler)
    {
        m_connectionFailureHandler();
    }
}

uint32_t CpuCores::writeTargetMemory(uint32_t address,
                                     const void* pvBuffer,
                                     uint32_t bufferSize,
                                     SWD::TransferSize writeSize)
{
    if (m_hasDetectedSwdDisconnect || bufferSize == 0)
    {
        return 0;
    }

    uint32_t bytesWritten = m_pSWD->writeMemory(address, pvBuffer, bufferSize, writeSize);
    if (bytesWritten == 0 && m_pSWD->getLastReadWriteError() == SWD::SWD_PROTOCOL)
    {
        handleUnrecoverableSwdError();
    }
    return bytesWritten;
}

void CpuCores::saveContext()
{
    bool encounteredError = false;

    bool contextHasFpu = m_fpu >= FPU_MAYBE;
    if (contextHasFpu)
    {
        Context_Init(&m_context, &m_contextEntriesFPU, 1);
    }
    else
    {
        Context_Init(&m_context, &m_contextEntriesNoFPU, 1);
    }

    // Transfer R0 - PSP first.
    for (uint32_t i = R0 ; i <= PSP ; i++)
    {
        uint32_t regValue = 0;
        if (!readCpuRegister(i, &regValue))
        {
            encounteredError = true;
        }
        Context_Set(&m_context, i, regValue);
    }

    // Transfer CONTROL, FAULTMASK, BASEPRI, PRIMASK next. They are all accessed in the CPU via a single 32-bit entry.
    uint32_t specialRegs = 0;
    if (!readCpuRegister(dcrsrSpecialRegisterIndex, &specialRegs))
    {
        encounteredError = true;
    }
    for (uint32_t i = 0 ; i < 4 ; i++)
    {
        Context_Set(&m_context, PRIMASK+i, (specialRegs >> (8 * i)) & 0xFF);
    }

    // Transfer FPU registers if we returned target XML that indicates they might/do exist.
    if (contextHasFpu)
    {
        // Transfer S0-S31 floating point registers.
        for (uint32_t i = 0 ; i < 32 ; i++)
        {
            uint32_t regValue = 0xBAADFEED;
            if (m_fpu == FPU_AVAILABLE)
            {
                if (!readCpuRegister(dcrsrS0Index+i, &regValue))
                {
                    encounteredError = true;
                }
            }
            Context_Set(&m_context, S0+i, regValue);
        }

        // Transfer FPSCR register.
        if (m_fpu == FPU_AVAILABLE)
        {
            uint32_t regValue = 0xBAADFEED;
            if (m_fpu == FPU_AVAILABLE)
            {
                if (!readCpuRegister(dcrsrFpscrIndex, &regValue))
                {
                    encounteredError = true;
                }
            }
            Context_Set(&m_context, FPSCR, regValue);
        }
    }

    if (encounteredError)
    {
        logError("Failed to read CPU register(s).");
    }
}

bool CpuCores::readCpuRegister(uint32_t registerIndex, uint32_t* pValue)
{
    uint32_t DCRSR_Value = registerIndex & DCRSR_REGSEL_Mask;
    if (writeTargetMemory(DCRSR_Address, &DCRSR_Value, sizeof(DCRSR_Value), SWD::TRANSFER_32BIT) != sizeof(DCRSR_Value))
    {
        return false;
    }

    waitForRegisterTransferToComplete();

    return readTargetMemory(DCRDR_Address, pValue, sizeof(*pValue), SWD::TRANSFER_32BIT) == sizeof(*pValue);
}

void CpuCores::waitForRegisterTransferToComplete()
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

bool CpuCores::hasRegisterTransferCompleted(uint32_t DHCSR_Value)
{
    const uint32_t DHCSR_S_REGRDY_Bit = 1 << 16;
    return (DHCSR_Value & DHCSR_S_REGRDY_Bit) == DHCSR_S_REGRDY_Bit;
}

void CpuCores::restoreContext()
{
    bool encounteredError = false;

    // Transfer R0 - PSP first.
    for (uint32_t i = R0 ; i <= PSP ; i++)
    {
        if (!writeCpuRegister(i, Context_Get(&m_context, i)))
        {
            encounteredError = true;
        }
    }

    // Transfer CONTROL, FAULTMASK, BASEPRI, PRIMASK next. They are all accessed in the CPU via a single 32-bit entry.
    uint32_t specialRegs = 0;
    for (uint32_t i = 0 ; i < 4 ; i++)
    {
        specialRegs |= (Context_Get(&m_context, PRIMASK+i) & 0xFF) << (8 * i);
    }
    if (!writeCpuRegister(dcrsrSpecialRegisterIndex, specialRegs))
    {
        encounteredError = true;
    }

    // Transfer FPU registers back to CPU if we have detected that the FPU exists for sure.
    if (m_fpu == FPU_AVAILABLE)
    {
        // Transfer S0-S31 floating point registers.
        for (uint32_t i = 0 ; i < 32 ; i++)
        {
            if (!writeCpuRegister(dcrsrS0Index+i, Context_Get(&m_context, S0+i)))
            {
                encounteredError = true;
            }
        }

        // Transfer FPSCR register.
        if (!writeCpuRegister(dcrsrFpscrIndex, Context_Get(&m_context, FPSCR)))
        {
            encounteredError = true;
        }
    }

    if (encounteredError)
    {
        logError("Failed to write CPU register(s).");
    }
}

bool CpuCores::writeCpuRegister(uint32_t registerIndex, uint32_t value)
{
    if (writeTargetMemory(DCRDR_Address, &value, sizeof(value), SWD::TRANSFER_32BIT) != sizeof(value))
    {
        return false;
    }

    uint32_t DCRSR_Value = DCRSR_REGWnR_Bit | (registerIndex & DCRSR_REGSEL_Mask);
    if (writeTargetMemory(DCRSR_Address, &DCRSR_Value, sizeof(DCRSR_Value), SWD::TRANSFER_32BIT) != sizeof(DCRSR_Value))
    {
        return false;
    }

    waitForRegisterTransferToComplete();

    return true;
}

bool CpuCores::requestCpuToHalt()
{
    uint32_t DHCSR_Val = 0;

    readDHCSR(&DHCSR_Val);
    // Ignore read errors.

    DHCSR_Val |= DHCSR_C_HALT_Bit;

    if (!writeDHCSR(DHCSR_Val))
    {
        logError("Failed to set C_HALT bit in DHCSR.");
        return false;
    }
    return true;
}

bool CpuCores::requestCpuToResume()
{
    uint32_t DHCSR_Val = 0;

    readDHCSR(&DHCSR_Val);
    DHCSR_Val &= ~DHCSR_C_HALT_Bit;
    if (!writeDHCSR(DHCSR_Val))
    {
        logError("Failed to clear C_HALT bit in DHCSR.");
        return false;
    }
    return true;
}

bool CpuCores::enableHaltDebugging(uint32_t timeout_ms)
{
    uint32_t DHCSR_Val = 0;
    if (!readDHCSRWithRetry(&DHCSR_Val, timeout_ms))
    {
        return false;
    }

    // The values of the current DHCSR_C_* bits will be overwritten with just what is needed to enable SWD debugging
    // and maintain the target's current halted state.
    if (isCpuHalted(DHCSR_Val))
    {
        // If the CPU is currently halted then we don't want to set DHCSR with DHCSR_C_HALT_Bit cleared as that will
        // start the target running again.
        logInfo("CPU was already halted at debugger init.");
        DHCSR_Val = DHCSR_C_HALT_Bit;
    }
    else
    {
        DHCSR_Val = 0;
    }

    //  Enable halt mode debug.  Set to 1 to enable halt mode debugging.
    const uint32_t DHCSR_C_DEBUGEN_Bit = 1 << 0;
    DHCSR_Val |= DHCSR_C_DEBUGEN_Bit;
    if (!writeDHCSR(DHCSR_Val))
    {
        logError("Failed to set C_DEBUGEN bit in DHCSR.");
        return false;
    }

    return true;
}

bool CpuCores::readDHCSRWithRetry(uint32_t* pValue, uint32_t timeout_ms)
{
    bool returnVal = false;
    absolute_time_t endTime = make_timeout_time_ms(timeout_ms);
    m_pSwdBus->disableErrorLogging();
    if (timeout_ms > 0)
    {
        logErrorDisable();
    }
    do
    {
        if (readDHCSR(pValue))
        {
            returnVal = true;
            break;
        }
    } while (absolute_time_diff_us(get_absolute_time(), endTime) > 0);
    if (timeout_ms > 0)
    {
        logErrorEnable();
    }
    m_pSwdBus->enableErrorLogging();

    return returnVal;
}

bool CpuCores::readDHCSR(uint32_t* pValue)
{
    if (readTargetMemory(DHCSR_Address, pValue, sizeof(*pValue), SWD::TRANSFER_32BIT) != sizeof(*pValue))
    {
        logError("Failed to read DHCSR register.");
        return false;
    }
    return true;
}

bool CpuCores::writeDHCSR(uint32_t DHCSR_Value)
{
    // Upper 16-bits must contain DBGKEY for CPU to accept this write.
    const uint32_t DHCSR_DBGKEY_Shift = 16;
    const uint32_t DHCSR_DBGKEY_Mask = 0xFFFF << DHCSR_DBGKEY_Shift;
    const uint32_t DHCSR_DBGKEY = 0xA05F << DHCSR_DBGKEY_Shift;
    DHCSR_Value = (DHCSR_Value & ~DHCSR_DBGKEY_Mask) | DHCSR_DBGKEY;

    return writeTargetMemory(DHCSR_Address, &DHCSR_Value, sizeof(DHCSR_Value), SWD::TRANSFER_32BIT) == sizeof(DHCSR_Value);
}

void CpuCores::initForDebugging()
{
    enableDWTandVectorCatches();
    initDWT();
    initFPB();
}

void CpuCores::enableDWTandVectorCatches()
{
    // DEMCR_DWTENA is the name on ARMv6M and it is called TRCENA on ARMv7M.
    const uint32_t DEMCR_DWTENA_Bit = 1 << 24;
    // Enable Halting debug trap on a HardFault exception.
    const uint32_t DEMCR_VC_HARDERR_Bit = 1 << 10;
    // Enable Halting debug trap on a fault occurring during exception entry or exception return.
    const uint32_t DEMCR_VC_INTERR_Bit = 1 << 9;
    // Enable Halting debug trap on a BusFault exception.
    const uint32_t DEMCR_VC_BUSERR_Bit = 1 << 8;
    // Enable Halting debug trap on a UsageFault exception caused by a state information error, for example an
    // Undefined Instruction exception.
    const uint32_t DEMCR_VC_STATERR_Bit = 1 << 7;
    // Enable Halting debug trap on a UsageFault exception caused by a checking error, for example an alignment
    // check error.
    const uint32_t DEMCR_VC_CHKERR_Bit = 1 << 6;
    // Enable Halting debug trap on a UsageFault caused by an access to a coprocessor.
    const uint32_t DEMCR_VC_NOCPERR_Bit = 1 << 5;
    // Enable Halting debug trap on a MemManage exception.
    const uint32_t DEMCR_VC_MMERR_Bit = 1 << 4;
    // Catch all of the vectors.
    const uint32_t allVectorCatchBits = DEMCR_VC_HARDERR_Bit | DEMCR_VC_INTERR_Bit | DEMCR_VC_BUSERR_Bit |
                                        DEMCR_VC_STATERR_Bit | DEMCR_VC_CHKERR_Bit | DEMCR_VC_NOCPERR_Bit |
                                        DEMCR_VC_MMERR_Bit;

    if (!setOrClearBitsInDEMCR(DEMCR_DWTENA_Bit | allVectorCatchBits, true))
    {
        logError("Failed to set DWTENA/TRCENA and vector catch bits in DEMCR register.");
        return;
    }
}

bool CpuCores::setOrClearBitsInDEMCR(uint32_t bitMask, bool set)
{
    // Debug Exception and Monitor Control Register, DEMCR
    const uint32_t DEMCR_Address = 0xE000EDFC;

    uint32_t DEMCR_Value = 0;
    if (readTargetMemory(DEMCR_Address, &DEMCR_Value, sizeof(DEMCR_Value), SWD::TRANSFER_32BIT) != sizeof(DEMCR_Value))
    {
        logError("Failed to read DEMCR register.");
        return false;
    }

    // Set or clear the requested bits.
    if (set)
    {
        DEMCR_Value |= bitMask;
    }
    else
    {
        DEMCR_Value &= ~bitMask;
    }

    if (writeTargetMemory(DEMCR_Address, &DEMCR_Value, sizeof(DEMCR_Value), SWD::TRANSFER_32BIT) != sizeof(DEMCR_Value))
    {
        logError("Failed to write DEMCR register.");
        return false;
    }
    return true;
}

void CpuCores::initDWT()
{
    uint32_t watchpointCount = clearDWTComparators();
    logInfoF("CPU supports %lu hardware watchpoints.", watchpointCount);
}

uint32_t CpuCores::clearDWTComparators()
{
    uint32_t DWT_COMP_Address = 0xE0001020;
    uint32_t comparatorCount = getDWTComparatorCount();
    for (uint32_t i = 0 ; i < comparatorCount ; i++)
    {
        clearDWTComparator(DWT_COMP_Address);
        DWT_COMP_Address += sizeof(DWT_COMP_Type);
    }
    return comparatorCount;
}

uint32_t CpuCores::getDWTComparatorCount()
{
    uint32_t DWT_CTRL_Address = 0xE0001000;
    uint32_t DWT_CTRL_Value = 0;

    if (readTargetMemory(DWT_CTRL_Address, &DWT_CTRL_Value, sizeof(DWT_CTRL_Value), SWD::TRANSFER_32BIT) != sizeof(DWT_CTRL_Value))
    {
        logError("Failed to read DWT_CTRL register.");
        return 0;
    }
    return (DWT_CTRL_Value >> 28) & 0xF;
}

void CpuCores::clearDWTComparator(uint32_t comparatorAddress)
{
    //  Matched.  Read-only.  Set to 1 to indicate that this comparator has been matched.  Cleared on read.
    const uint32_t DWT_COMP_FUNCTION_DATAVMATCH_Bit = 1 << 8;
    //  Cycle Count Match.  Set to 1 for enabling cycle count match and 0 otherwise.  Only valid on comparator 0.
    const uint32_t DWT_COMP_FUNCTION_CYCMATCH_Bit = 1 << 7;
    //  Enable Data Trace Address offset packets.  0 to disable.
    const uint32_t DWT_COMP_FUNCTION_EMITRANGE_Bit = 1 << 5;
    DWT_COMP_Type dwtComp;

    if (readTargetMemory(comparatorAddress, &dwtComp, 3*sizeof(uint32_t), SWD::TRANSFER_32BIT) != 3*sizeof(uint32_t))
    {
        logError("Failed to read DWT_COMP/DWT_MASK/DWT_FUNCTION registers for clearing.");
    }

    dwtComp.comp = 0;
    dwtComp.mask = 0;
    dwtComp.function &= ~(DWT_COMP_FUNCTION_DATAVMATCH_Bit |
                          DWT_COMP_FUNCTION_CYCMATCH_Bit |
                          DWT_COMP_FUNCTION_EMITRANGE_Bit |
                          DWT_COMP_FUNCTION_FUNCTION_Mask);

    if (writeTargetMemory(comparatorAddress, &dwtComp, 3*sizeof(uint32_t), SWD::TRANSFER_32BIT) != 3*sizeof(uint32_t))
    {
        logError("Failed to write DWT_COMP/DWT_MASK/DWT_FUNCTION registers for clearing.");
    }
}

void CpuCores::initFPB()
{
    uint32_t breakpointCount = clearFPBComparators();
    logInfoF("CPU supports %lu hardware breakpoints.", breakpointCount);
    enableFPB();
}

uint32_t CpuCores::clearFPBComparators()
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
    return codeComparatorCount;
}

uint32_t CpuCores::getFPBCodeComparatorCount()
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

uint32_t CpuCores::readFPControlRegister()
{
    uint32_t FP_CTRL_Value = 0;
    if (readTargetMemory(FP_CTRL_Address, &FP_CTRL_Value, sizeof(FP_CTRL_Value), SWD::TRANSFER_32BIT) != sizeof(FP_CTRL_Value))
    {
        logError("Failed to read FP_CTRL register.");
        return 0;
    }
    return FP_CTRL_Value;
}

uint32_t CpuCores::getFPBLiteralComparatorCount()
{
    //  Number of instruction literal address comparators.  Read only
    const uint32_t FP_CTRL_NUM_LIT_Shift = 8;
    const uint32_t FP_CTRL_NUM_LIT_Mask = 0xF << FP_CTRL_NUM_LIT_Shift;
    uint32_t FP_CTRL_Value = readFPControlRegister();

    return ((FP_CTRL_Value & FP_CTRL_NUM_LIT_Mask) >> FP_CTRL_NUM_LIT_Shift);
}

void CpuCores::clearFPBComparator(uint32_t comparatorAddress)
{
    uint32_t comparatorValue = 0;
    if (writeTargetMemory(comparatorAddress, &comparatorValue, sizeof(comparatorValue), SWD::TRANSFER_32BIT) != sizeof(comparatorValue))
    {
        logError("Failed to write to FP comparator register.");
    }
}

void CpuCores::enableFPB()
{
    //  This Key field must be set to 1 when writing or the write will be ignored.
    const uint32_t FP_CTRL_KEY = 1 << 1;
    //  Enable bit for the FPB.  Set to 1 to enable FPB.
    const uint32_t FP_CTRL_ENABLE = 1;

    uint32_t FP_CTRL_Value = readFPControlRegister();
    FP_CTRL_Value |= (FP_CTRL_KEY | FP_CTRL_ENABLE);
    writeFPControlRegister(FP_CTRL_Value);
}

void CpuCores::writeFPControlRegister(uint32_t FP_CTRL_Value)
{
    if (writeTargetMemory(FP_CTRL_Address, &FP_CTRL_Value, sizeof(FP_CTRL_Value), SWD::TRANSFER_32BIT) != sizeof(FP_CTRL_Value))
    {
        logError("Failed to write FP_CTRL register.");
    }
}

void CpuCores::enteringDebugger()
{
    m_trapReason = cacheTrapReason();
    disableResetVectorCatch();
}

PlatformTrapReason CpuCores::cacheTrapReason()
{
    PlatformTrapReason reason = { MRI_PLATFORM_TRAP_TYPE_UNKNOWN, 0x00000000 };
    uint32_t debugFaultStatus = 0;
    if (!readDFSR(&debugFaultStatus))
    {
        return reason;
    }

    if (debugFaultStatus & DFSR_BKPT_Bit)
    {
        // Was caused by hardware or software breakpoint. If PC points to BKPT then report as software breakpoint.
        if (Platform_TypeOfCurrentInstruction() == MRI_PLATFORM_INSTRUCTION_HARDCODED_BREAKPOINT)
        {
            reason.type = MRI_PLATFORM_TRAP_TYPE_SWBREAK;
        }
        else
        {
            reason.type = MRI_PLATFORM_TRAP_TYPE_HWBREAK;
        }
    }
    else if (debugFaultStatus & DFSR_DWTTRAP_Bit)
    {
        reason = findMatchedWatchpoint();
    }
    return reason;
}

bool CpuCores::readDFSR(uint32_t* pDFSR)
{
    if (readTargetMemory(DFSR_Address, pDFSR, sizeof(*pDFSR), SWD::TRANSFER_32BIT) != sizeof(*pDFSR))
    {
        logError("Failed to read DFSR.");
        return false;
    }
    return true;
}

PlatformTrapReason CpuCores::findMatchedWatchpoint(void)
{
    PlatformTrapReason reason = { MRI_PLATFORM_TRAP_TYPE_UNKNOWN, 0x00000000 };
    uint32_t currentComparatorAddress = DWT_COMP_ARRAY_Address;
    uint32_t comparatorCount = getDWTComparatorCount();
    for (uint32_t i = 0 ; i < comparatorCount ; i++)
    {
        uint32_t function = 0;
        if (readTargetMemory(currentComparatorAddress + offsetof(DWT_COMP_Type, function), &function, sizeof(function), SWD::TRANSFER_32BIT) != sizeof(function))
        {
            logError("Failed to read DWT function register.");
            return reason;
        }
        if (function & DWT_COMP_FUNCTION_MATCHED)
        {
            reason = getReasonFromMatchComparator(currentComparatorAddress, function);
        }
        currentComparatorAddress += sizeof(DWT_COMP_Type);
    }
    return reason;
}

PlatformTrapReason CpuCores::getReasonFromMatchComparator(uint32_t comparatorAddress, uint32_t function)
{
    PlatformTrapReason reason = { MRI_PLATFORM_TRAP_TYPE_UNKNOWN, 0x00000000 };
    switch (function & DWT_COMP_FUNCTION_FUNCTION_Mask)
    {
    case DWT_COMP_FUNCTION_FUNCTION_DATA_READ:
        reason.type = MRI_PLATFORM_TRAP_TYPE_RWATCH;
        break;
    case DWT_COMP_FUNCTION_FUNCTION_DATA_WRITE:
        reason.type = MRI_PLATFORM_TRAP_TYPE_WATCH;
        break;
    case DWT_COMP_FUNCTION_FUNCTION_DATA_READWRITE:
        reason.type = MRI_PLATFORM_TRAP_TYPE_AWATCH;
        break;
    default:
        reason.type = MRI_PLATFORM_TRAP_TYPE_UNKNOWN;
        break;
    }

    uint32_t compValue = 0;
    if (readTargetMemory(comparatorAddress + offsetof(DWT_COMP_Type, comp), &compValue, sizeof(compValue), SWD::TRANSFER_32BIT) != sizeof(compValue))
    {
        logError("Failed to read DWT comp register.");
        return reason;
    }
    reason.address = compValue;

    return reason;
}

void CpuCores::disableResetVectorCatch()
{
    if (!setOrClearBitsInDEMCR(DEMCR_VC_CORERESET_Bit, false))
    {
        logError("Failed to clear reset vector catch bits in DEMCR register.");
        return;
    }
}

void CpuCores::leavingDebugger()
{
    clearDFSR();
}

void CpuCores::clearDFSR()
{
    uint32_t dfsr = 0;
    if (!readDFSR(&dfsr))
    {
        logError("Failed to read DFSR to clear it.");
        return;
    }
    if (!writeDFSR(dfsr))
    {
        logError("Failed to write DFSR to clear it.");
        return;
    }
}

bool CpuCores::writeDFSR(uint32_t dfsr)
{
    if (writeTargetMemory(DFSR_Address, &dfsr, sizeof(dfsr), SWD::TRANSFER_32BIT) != sizeof(dfsr))
    {
        logError("Failed to write to DFSR.");
        return false;
    }
    return true;
}

void CpuCores::configureSingleSteppingBitsInDHCSR(bool enableSingleStepping)
{
    const uint32_t bitsToEnableSingleStepWithInterruptsDisabled = DHCSR_C_STEP_Bit | DHCSR_C_MASKINTS_Bit;
    uint32_t DHCSR_Val = 0;

    readDHCSR(&DHCSR_Val);
    if (enableSingleStepping)
    {
        DHCSR_Val |= bitsToEnableSingleStepWithInterruptsDisabled;
    }
    else
    {
        DHCSR_Val &= ~bitsToEnableSingleStepWithInterruptsDisabled;
    }

    if (!writeDHCSR(DHCSR_Val))
    {
        logErrorF("Failed to update C_STEP & C_MASKINTS bits in DHCSR to %s single stepping.",
                     enableSingleStepping ? "enable" : "disable");
    }
}

uint32_t CpuCores::setBreakpoint(uint32_t breakpointAddress, bool is32BitInstruction)
{
    uint32_t existingFPBBreakpoint = findFPBBreakpointComparator(breakpointAddress, is32BitInstruction);
    if (existingFPBBreakpoint != 0)
    {
        // This breakpoint is already set so just return pointer to existing comparator.
        return existingFPBBreakpoint;
    }

    uint32_t freeFPBBreakpointComparator = findFreeFPBBreakpointComparator();
    if (freeFPBBreakpointComparator == 0)
    {
        // All FPB breakpoint comparator slots are used so return NULL as error indicator.
        logInfoF("No free hardware breakpoints for setting breakpoint at address 0x%08lX.", breakpointAddress);
        return 0;
    }

    uint32_t comparatorValue = calculateFPBComparatorValue(breakpointAddress, is32BitInstruction);
    if (writeTargetMemory(freeFPBBreakpointComparator, &comparatorValue, sizeof(comparatorValue), SWD::TRANSFER_32BIT) != sizeof(comparatorValue))
    {
        logErrorF("Failed to set breakpoint at address 0x%08lX.", breakpointAddress);
        return 0;
    }
    return freeFPBBreakpointComparator;
}

uint32_t CpuCores::findFPBBreakpointComparator(uint32_t breakpointAddress, bool is32BitInstruction)
{
    uint32_t comparatorValueForThisBreakpoint = calculateFPBComparatorValue(breakpointAddress, is32BitInstruction);
    uint32_t codeComparatorCount = getFPBCodeComparatorCount();
    uint32_t currentComparatorAddress = FPB_COMP_ARRAY_Address;
    for (uint32_t i = 0 ; i < codeComparatorCount ; i++)
    {
        uint32_t currentComparatorValue = 0;
        if (readTargetMemory(currentComparatorAddress, &currentComparatorValue, sizeof(currentComparatorValue), SWD::TRANSFER_32BIT) != sizeof(currentComparatorValue))
        {
            logErrorF("Failed to read from FPB comparator at address 0x%08lX.", currentComparatorAddress);
            return 0;
        }
        uint32_t maskOffReservedBits = maskOffFPBComparatorReservedBits(currentComparatorValue);
        if (comparatorValueForThisBreakpoint == maskOffReservedBits)
        {
            return currentComparatorAddress;
        }

        currentComparatorAddress += sizeof(uint32_t);
    }

    // Return NULL if no FPB comparator is already enabled for this breakpoint.
    return 0;
}

uint32_t CpuCores::calculateFPBComparatorValue(uint32_t breakpointAddress, bool is32BitInstruction)
{
    if (isBreakpointAddressInvalid(breakpointAddress))
    {
        return (uint32_t)~0U;
    }
    if (getFPBRevision() == FP_CTRL_REVISION2)
    {
        return calculateFPBComparatorValueRevision2(breakpointAddress);
    }
    else
    {
        return calculateFPBComparatorValueRevision1(breakpointAddress, is32BitInstruction);
    }
}

bool CpuCores::isBreakpointAddressInvalid(uint32_t breakpointAddress)
{
    if (getFPBRevision() == FP_CTRL_REVISION2)
    {
        // On revision 2, can set breakpoint at any address in the 4GB range, except for at an odd addresses.
        return isAddressOdd(breakpointAddress);
    }
    else
    {
        // On revision 1, can only set a breakpoint on addresses where the upper 3-bits are all 0 (upper 3.5GB is off
        // limits) and the address is half-word aligned.
        return isAddressAboveLowestHalfGig(breakpointAddress) || isAddressOdd(breakpointAddress);
    }
}

uint32_t CpuCores::getFPBRevision()
{
    // Flash Patch breakpoint architecture revision. 0 for revision 1 and 1 for revision 2.
    uint32_t FP_CTRL_REV_Shift = 28;
    uint32_t FP_CTRL_REV_Mask = (0xF << FP_CTRL_REV_Shift);

    uint32_t controlValue = readFPControlRegister();
    return ((controlValue & FP_CTRL_REV_Mask) >> FP_CTRL_REV_Shift);
}

bool CpuCores::isAddressOdd(uint32_t address)
{
    return !!(address & 0x1);
}

bool CpuCores::isAddressAboveLowestHalfGig(uint32_t address)
{
    return !!(address & 0xE0000000);
}

uint32_t CpuCores::calculateFPBComparatorValueRevision1(uint32_t breakpointAddress, bool is32BitInstruction)
{
    uint32_t    comparatorValue;
    comparatorValue = (breakpointAddress & FP_COMP_COMP_Mask);
    comparatorValue |= FP_COMP_ENABLE_Bit;
    comparatorValue |= calculateFPBComparatorReplaceValue(breakpointAddress, is32BitInstruction);

    return comparatorValue;
}

uint32_t CpuCores::calculateFPBComparatorReplaceValue(uint32_t breakpointAddress, bool is32BitInstruction)
{
    //  Defines the behaviour for code address comparators.
    uint32_t FP_COMP_REPLACE_Shift = 30;
    // Breakpoint on lower halfword.
    uint32_t FP_COMP_REPLACE_BREAK_LOWER = 0x1U << FP_COMP_REPLACE_Shift;
    // Breakpoint on upper halfword.
    uint32_t FP_COMP_REPLACE_BREAK_UPPER = 0x2U << FP_COMP_REPLACE_Shift;
    // Breakpoint on word.
    uint32_t FP_COMP_REPLACE_BREAK = 0x3U << FP_COMP_REPLACE_Shift;

    if (is32BitInstruction)
    {
        return FP_COMP_REPLACE_BREAK;
    }
    else if (isAddressInUpperHalfword(breakpointAddress))
    {
        return FP_COMP_REPLACE_BREAK_UPPER;
    }
    else
    {
        return FP_COMP_REPLACE_BREAK_LOWER;
    }
}

bool CpuCores::isAddressInUpperHalfword(uint32_t address)
{
    return !!(address & 0x2);
}

uint32_t CpuCores::calculateFPBComparatorValueRevision2(uint32_t breakpointAddress)
{
    return breakpointAddress | FP_COMP_BE_Bit;
}

uint32_t CpuCores::maskOffFPBComparatorReservedBits(uint32_t comparatorValue)
{
    //  Defines the behaviour for code address comparators.
    uint32_t FP_COMP_REPLACE_Shift = 30;
    uint32_t FP_COMP_REPLACE_Mask = 0x3U << FP_COMP_REPLACE_Shift;

    if (getFPBRevision() == FP_CTRL_REVISION2)
    {
        return comparatorValue;
    }
    else
    {
        return (comparatorValue & (FP_COMP_REPLACE_Mask | FP_COMP_COMP_Mask | FP_COMP_ENABLE_Bit));
    }
}

uint32_t CpuCores::findFreeFPBBreakpointComparator()
{
    uint32_t currentComparatorAddress = FPB_COMP_ARRAY_Address;
    uint32_t codeComparatorCount = getFPBCodeComparatorCount();
    for (uint32_t i = 0 ; i < codeComparatorCount ; i++)
    {
        uint32_t currentComparatorValue = 0;
        if (readTargetMemory(currentComparatorAddress, &currentComparatorValue, sizeof(currentComparatorValue), SWD::TRANSFER_32BIT) != sizeof(currentComparatorValue))
        {
            logErrorF("Failed to read from FPB comparator at address 0x%08lX.", currentComparatorAddress);
            return 0;
        }
        if (!isFPBComparatorEnabled(currentComparatorValue))
        {
            return currentComparatorAddress;
        }

        currentComparatorAddress += sizeof(uint32_t);
    }

    // Return 0 if no FPB breakpoint comparators are free.
    return 0;
}

bool CpuCores::isFPBComparatorEnabled(uint32_t comparator)
{
    if (getFPBRevision() == FP_CTRL_REVISION2)
    {
        return isFPBComparatorEnabledRevision2(comparator);
    }
    else
    {
        return isFPBComparatorEnabledRevision1(comparator);
    }
}

bool CpuCores::isFPBComparatorEnabledRevision1(uint32_t comparator)
{
    return !!(comparator & FP_COMP_ENABLE_Bit);
}

bool CpuCores::isFPBComparatorEnabledRevision2(uint32_t comparator)
{
    return !!((comparator & FP_COMP_BE_Bit) || (comparator & FP_COMP_FE_Bit));
}

uint32_t CpuCores::clearBreakpoint(uint32_t breakpointAddress, bool is32BitInstruction)
{
    uint32_t existingFPBBreakpoint = findFPBBreakpointComparator(breakpointAddress, is32BitInstruction);
    if (existingFPBBreakpoint != 0)
    {
        clearFPBComparator(existingFPBBreakpoint);
        logInfoF("Hardware breakpoint cleared at address 0x%08lX.", breakpointAddress);
    }

    return existingFPBBreakpoint;
}

void CpuCores::setWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type)
{
    uint32_t nativeType = convertWatchpointTypeToCortexMType(type);
    if (!isValidDWTComparatorSetting(address, size, nativeType))
    {
        __throw(invalidArgumentException);
    }

    uint32_t comparatorAddress = enableDWTWatchpoint(address, size, nativeType);
    if (comparatorAddress == 0)
    {
        __throw(exceededHardwareResourcesException);
    }
    logInfoF("Hardware watchpoint set at address 0x%08X.", address);
}

uint32_t CpuCores::convertWatchpointTypeToCortexMType(PlatformWatchpointType type)
{
    switch (type)
    {
    case MRI_PLATFORM_WRITE_WATCHPOINT:
        return DWT_COMP_FUNCTION_FUNCTION_DATA_WRITE;
    case MRI_PLATFORM_READ_WATCHPOINT:
        return DWT_COMP_FUNCTION_FUNCTION_DATA_READ;
    case MRI_PLATFORM_READWRITE_WATCHPOINT:
        return DWT_COMP_FUNCTION_FUNCTION_DATA_READWRITE;
    default:
        return 0;
    }
}

bool CpuCores::isValidDWTComparatorSetting(uint32_t watchpointAddress,
                                           uint32_t watchpointSize,
                                           uint32_t watchpointType)
{
    return isValidDWTComparatorSize(watchpointSize) &&
           isValidDWTComparatorAddress(watchpointAddress, watchpointSize) &&
           isValidDWTComparatorType(watchpointType);
}

bool CpuCores::isValidDWTComparatorSize(uint32_t watchpointSize)
{
    return isPowerOf2(watchpointSize);
}

bool CpuCores::isPowerOf2(uint32_t value)
{
    return (value & (value - 1)) == 0;
}

bool CpuCores::isValidDWTComparatorAddress(uint32_t watchpointAddress, uint32_t watchpointSize)
{
    return isAddressAlignedToSize(watchpointAddress, watchpointSize);
}

bool CpuCores::isAddressAlignedToSize(uint32_t address, uint32_t size)
{
    uint32_t addressMask = ~(size - 1);
    return address == (address & addressMask);
}

bool CpuCores::isValidDWTComparatorType(uint32_t watchpointType)
{
    return (watchpointType == DWT_COMP_FUNCTION_FUNCTION_DATA_READ) ||
           (watchpointType == DWT_COMP_FUNCTION_FUNCTION_DATA_WRITE) ||
           (watchpointType == DWT_COMP_FUNCTION_FUNCTION_DATA_READWRITE);
}

uint32_t CpuCores::enableDWTWatchpoint(uint32_t watchpointAddress,
                                       uint32_t watchpointSize,
                                       uint32_t watchpointType)
{
    uint32_t comparatorAddress = findDWTComparator(watchpointAddress, watchpointSize, watchpointType);
    if (comparatorAddress != 0)
    {
        // This watchpoint has already been set so return a pointer to it.
        return comparatorAddress;
    }

    comparatorAddress = findFreeDWTComparator();
    if (comparatorAddress == 0)
    {
        // There are no free comparators left.
        logInfoF("No free hardware watchpoints for setting watchpoint at address 0x%08lX.", watchpointAddress);
        return 0;
    }

    if (!attemptToSetDWTComparator(comparatorAddress, watchpointAddress, watchpointSize, watchpointType))
    {
        // Failed set due to the size being larger than supported by CPU.
        logInfoF("Failed to set watchpoint at address 0x%08lX of size %lu bytes.", watchpointAddress, watchpointSize);
        return 0;
    }

    // Successfully configured a free comparator for this watchpoint.
    return comparatorAddress;
}

uint32_t CpuCores::findDWTComparator(uint32_t watchpointAddress,
                                  uint32_t watchpointSize,
                                  uint32_t watchpointType)
{
    uint32_t currentComparatorAddress = DWT_COMP_ARRAY_Address;
    uint32_t comparatorCount = getDWTComparatorCount();
    for (uint32_t i = 0 ; i < comparatorCount ; i++)
    {
        if (doesDWTComparatorMatch(currentComparatorAddress, watchpointAddress, watchpointSize, watchpointType))
        {
            return currentComparatorAddress;
        }

        currentComparatorAddress += sizeof(DWT_COMP_Type);
    }

    // Return NULL if no DWT comparator is already enabled for this watchpoint.
    return 0;
}

bool CpuCores::doesDWTComparatorMatch(uint32_t comparatorAddress,
                                      uint32_t address,
                                      uint32_t size,
                                      uint32_t function)
{
    return doesDWTComparatorFunctionMatch(comparatorAddress, function) &&
           doesDWTComparatorAddressMatch(comparatorAddress, address) &&
           doesDWTComparatorMaskMatch(comparatorAddress, size);
}

bool CpuCores::doesDWTComparatorFunctionMatch(uint32_t comparatorAddress, uint32_t function)
{
    uint32_t functionValue = 0;
    if (readTargetMemory(comparatorAddress + offsetof(DWT_COMP_Type, function), &functionValue, sizeof(functionValue), SWD::TRANSFER_32BIT) != sizeof(functionValue))
    {
        logError("Failed to read DWT function register.");
        return false;
    }
    uint32_t importantFunctionBits = maskOffDWTFunctionBits(functionValue);

    return importantFunctionBits == function;
}

uint32_t CpuCores::maskOffDWTFunctionBits(uint32_t functionValue)
{
    // Data Watchpoint and Trace Comparator Function Bits.
    //  Data Address Linked Index 1.
    const uint32_t DWT_COMP_FUNCTION_DATAVADDR1_Bit = 0xF << 16;
    //  Data Address Linked Index 0.
    const uint32_t DWT_COMP_FUNCTION_DATAVADDR0_Bit = 0xF << 12;
    //  Selects size for data value matches.
    const uint32_t DWT_COMP_FUNCTION_DATAVSIZE_Mask = 3 << 10;
    //  Data Value Match.  Set to 0 for address compare and 1 for data value compare.
    const uint32_t DWT_COMP_FUNCTION_DATAVMATCH_Bit = 1 << 8;
    //  Cycle Count Match.  Set to 1 for enabling cycle count match and 0 otherwise.  Only valid on comparator 0.
    const uint32_t DWT_COMP_FUNCTION_CYCMATCH_Bit = 1 << 7;
    //  Enable Data Trace Address offset packets.  0 to disable.
    const uint32_t DWT_COMP_FUNCTION_EMITRANGE_Bit = 1 << 5;

    return functionValue & (DWT_COMP_FUNCTION_DATAVADDR1_Bit |
                            DWT_COMP_FUNCTION_DATAVADDR0_Bit |
                            DWT_COMP_FUNCTION_DATAVSIZE_Mask |
                            DWT_COMP_FUNCTION_DATAVMATCH_Bit |
                            DWT_COMP_FUNCTION_CYCMATCH_Bit |
                            DWT_COMP_FUNCTION_EMITRANGE_Bit |
                            DWT_COMP_FUNCTION_FUNCTION_Mask);

}

bool CpuCores::doesDWTComparatorAddressMatch(uint32_t comparatorAddress, uint32_t address)
{
    uint32_t compValue = 0;
    if (readTargetMemory(comparatorAddress + offsetof(DWT_COMP_Type, comp), &compValue, sizeof(compValue), SWD::TRANSFER_32BIT) != sizeof(compValue))
    {
        logError("Failed to read DWT comparator register.");
        return false;
    }
    return compValue == address;
}

bool CpuCores::doesDWTComparatorMaskMatch(uint32_t comparatorAddress, uint32_t size)
{
    uint32_t maskValue = 0;
    if (readTargetMemory(comparatorAddress + offsetof(DWT_COMP_Type, mask), &maskValue, sizeof(maskValue), SWD::TRANSFER_32BIT) != sizeof(maskValue))
    {
        logError("Failed to read DWT mask register.");
        return false;
    }
    return maskValue == calculateLog2(size);
}

uint32_t CpuCores::calculateLog2(uint32_t value)
{
    uint32_t log2 = 0;

    while (value > 1)
    {
        value >>= 1;
        log2++;
    }

    return log2;
}

uint32_t CpuCores::findFreeDWTComparator()
{
    uint32_t currentComparatorAddress = DWT_COMP_ARRAY_Address;
    uint32_t comparatorCount = getDWTComparatorCount();
    for (uint32_t i = 0 ; i < comparatorCount ; i++)
    {
        if (isDWTComparatorFree(currentComparatorAddress))
        {
            return currentComparatorAddress;
        }
        currentComparatorAddress += sizeof(DWT_COMP_Type);
    }

    // Return NULL if there are no free DWT comparators.
    return 0;
}

bool CpuCores::isDWTComparatorFree(uint32_t comparatorAddress)
{
    //  Selects action to be taken on match.
    //      Disabled
    const uint32_t DWT_COMP_FUNCTION_FUNCTION_DISABLED = 0x0;

    uint32_t functionValue = 0;
    if (readTargetMemory(comparatorAddress + offsetof(DWT_COMP_Type, function), &functionValue, sizeof(functionValue), SWD::TRANSFER_32BIT) != sizeof(functionValue))
    {
        logError("Failed to read DWT function register.");
        return false;
    }
    return (functionValue & DWT_COMP_FUNCTION_FUNCTION_Mask) == DWT_COMP_FUNCTION_FUNCTION_DISABLED;
}

bool CpuCores::attemptToSetDWTComparator(uint32_t comparatorAddress,
                                         uint32_t watchpointAddress,
                                         uint32_t watchpointSize,
                                         uint32_t watchpointType)
{
    if (!attemptToSetDWTComparatorMask(comparatorAddress, watchpointSize))
    {
        logErrorF("Failed to set DWT mask register to a size of %lu bytes.", watchpointSize);
        return false;
    }
    if (writeTargetMemory(comparatorAddress + offsetof(DWT_COMP_Type, comp), &watchpointAddress, sizeof(watchpointAddress), SWD::TRANSFER_32BIT) != sizeof(watchpointAddress))
    {
        logError("Failed to write DWT comparator register.");
        return false;
    }
    if (writeTargetMemory(comparatorAddress + offsetof(DWT_COMP_Type, function), &watchpointType, sizeof(watchpointType), SWD::TRANSFER_32BIT) != sizeof(watchpointType))
    {
        logError("Failed to write DWT function register.");
        return false;
    }
    return true;
}

bool CpuCores::attemptToSetDWTComparatorMask(uint32_t comparatorAddress, uint32_t watchpointSize)
{
    uint32_t maskBitCount;

    maskBitCount = calculateLog2(watchpointSize);
    if (writeTargetMemory(comparatorAddress + offsetof(DWT_COMP_Type, mask), &maskBitCount, sizeof(maskBitCount), SWD::TRANSFER_32BIT) != sizeof(maskBitCount))
    {
        logError("Failed to write DWT mask register.");
        return false;
    }

    // Processor may limit number of bits to be masked off so check.
    uint32_t maskValue = 0;
    if (readTargetMemory(comparatorAddress + offsetof(DWT_COMP_Type, mask), &maskValue, sizeof(maskValue), SWD::TRANSFER_32BIT) != sizeof(maskValue))
    {
        logError("Failed to read DWT mask register.");
        return false;
    }
    return maskValue == maskBitCount;
}

void CpuCores::clearWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type)
{
    uint32_t nativeType = convertWatchpointTypeToCortexMType(type);

    if (!isValidDWTComparatorSetting(address, size, nativeType))
    {
        __throw(invalidArgumentException);
    }

    disableDWTWatchpoint(address, size, nativeType);
}

uint32_t CpuCores::disableDWTWatchpoint(uint32_t watchpointAddress,
                                        uint32_t watchpointSize,
                                        uint32_t watchpointType)
{
    uint32_t comparatorAddress = findDWTComparator(watchpointAddress, watchpointSize, watchpointType);
    if (comparatorAddress == 0)
    {
        // This watchpoint not set so return NULL.
        return 0;
    }

    logInfoF("Hardware watchpoint cleared at address 0x%08lX.", watchpointAddress);
    clearDWTComparator(comparatorAddress);
    return comparatorAddress;
}

uint8_t CpuCores::determineCauseOfException(bool wasStopFromGDB)
{
    uint32_t DFSR_Value = 0;
    if (readTargetMemory(DFSR_Address, &DFSR_Value, sizeof(DFSR_Value), SWD::TRANSFER_32BIT) != sizeof(DFSR_Value))
    {
        logError("Failed to read DFSR register.");
        // NOTE: Catch all signal will be SIGSTOP.
        return SIGSTOP;
    }

    // If VCATCH bit is set then look at CPSR to determine which exception handler was caught.
    if (DFSR_Value & DFSR_VCATCH_Bit)
    {
        switch(getExceptionNumber())
        {
        case 2:
            // NMI
            logInfo("Exception caught: NMI.");
            return SIGINT;
        case 3:
            // HardFault
            logInfo("Exception caught: Hard Fault.");
            return SIGSEGV;
        case 4:
            // MemManage
            logInfo("Exception caught: Mem Manage.");
            return SIGSEGV;
        case 5:
            // BusFault
            logInfo("Exception caught: Bus Fault.");
            return SIGBUS;
        case 6:
            // UsageFault
            logInfo("Exception caught: Usage Fault.");
            return SIGILL;
        default:
            // NOTE: Catch all signal for a vector/interrupt catch will be SIGINT.
            logInfo("Exception caught: Unknown.");
            return SIGINT;
        }
    }

    // Check the other bits in the DFSR if VCATCH didn't occur.
    if (DFSR_Value & DFSR_EXTERNAL_Bit)
    {
        logInfo("Debug event caught: External.");
        return SIGSTOP;
    }
    else if (DFSR_Value & DFSR_DWTTRAP_Bit)
    {
        logInfo("Debug event caught: Watchpoint.");
        return SIGTRAP;
    }
    else if (DFSR_Value & DFSR_BKPT_Bit)
    {
        logInfo("Debug event caught: Breakpoint.");
        return SIGTRAP;
    }
    else if (DFSR_Value & DFSR_HALTED_Bit)
    {
        uint32_t DHCSR_Value = 0;
        readDHCSR(&DHCSR_Value);
        if ((DHCSR_Value & DHCSR_C_STEP_Bit) && !wasStopFromGDB)
        {
            logInfo("Debug event caught: Single Step.");
            return SIGTRAP;
        }
        logInfo("Debug event caught: Halted.");
        return SIGINT;
    }

    // NOTE: Default catch all signal is SIGSTOP.
    logInfo("Debug event caught: Unknown.");
    return SIGSTOP;
}

uint32_t CpuCores::getExceptionNumber()
{
    return readRegisterOnHaltedCore(CPSR) & IPSR_Mask;
}

PlatformTrapReason CpuCores::getTrapReason()
{
    return m_trapReason;
}

void CpuCores::displayFaultCauseToGdbConsole()
{
    // Nothing to do on ARMv6-M devices since they don't have fault status registers.
    if (m_pSWD->getCpuType() < SWD::CPU_CORTEX_M3)
    {
        return;
    }

    switch (getExceptionNumber())
    {
    case 3:
        /* HardFault */
        displayHardFaultCauseToGdbConsole();
        break;
    case 4:
        /* MemManage */
        displayMemFaultCauseToGdbConsole();
        break;
    case 5:
        /* BusFault */
        displayBusFaultCauseToGdbConsole();
        break;
    case 6:
        /* UsageFault */
        displayUsageFaultCauseToGdbConsole();
        break;
    default:
        return;
    }
    WriteStringToGdbConsole("\n");
}

void CpuCores::displayHardFaultCauseToGdbConsole()
{
    const uint32_t HFSR_Address = 0xE000ED2C;
    const uint32_t debugEventBit = 1 << 31;
    const uint32_t forcedBit = 1 << 30;
    const uint32_t vectorTableReadBit = 1 << 1;
    uint32_t       hardFaultStatusRegister = 0xBAADFEED;

    if (readTargetMemory(HFSR_Address, &hardFaultStatusRegister, sizeof(hardFaultStatusRegister), SWD::TRANSFER_32BIT) != sizeof(hardFaultStatusRegister))
    {
        logError("Failed to read the HFSR register.");
        return;
    }

    WriteStringToGdbConsole("\n**Hard Fault**");
    WriteStringToGdbConsole("\n  Status Register: ");
    WriteHexValueToGdbConsole(hardFaultStatusRegister);

    if (hardFaultStatusRegister & debugEventBit)
    {
        WriteStringToGdbConsole("\n    Debug Event");
    }

    if (hardFaultStatusRegister & vectorTableReadBit)
    {
        WriteStringToGdbConsole("\n    Vector Table Read");
    }

    if (hardFaultStatusRegister & forcedBit)
    {
        WriteStringToGdbConsole("\n    Forced");
        displayMemFaultCauseToGdbConsole();
        displayBusFaultCauseToGdbConsole();
        displayUsageFaultCauseToGdbConsole();
    }
}

void CpuCores::displayMemFaultCauseToGdbConsole()
{
    const uint32_t MMARValidBit = 1 << 7;
    const uint32_t FPLazyStatePreservationBit = 1 << 5;
    const uint32_t stackingErrorBit = 1 << 4;
    const uint32_t unstackingErrorBit = 1 << 3;
    const uint32_t dataAccess = 1 << 1;
    const uint32_t instructionFetch = 1;
    uint32_t       CFSR_Value = 0xBAADFEED;

    if (readTargetMemory(CFSR_Address, &CFSR_Value, sizeof(CFSR_Value), SWD::TRANSFER_32BIT) != sizeof(CFSR_Value))
    {
        logError("Failed to read the CFSR register.");
        return;
    }
    uint8_t memManageFaultStatusRegister = CFSR_Value;

    /* Check to make sure that there is a memory fault to display. */
    if (memManageFaultStatusRegister == 0)
    {
        return;
    }

    WriteStringToGdbConsole("\n**MPU Fault**");
    WriteStringToGdbConsole("\n  Status Register: ");
    WriteHexValueToGdbConsole(memManageFaultStatusRegister);

    if (memManageFaultStatusRegister & MMARValidBit)
    {
        const uint32_t MMFAR_Address = 0xE000ED34;
        uint32_t       MMFAR_Value;
        if (readTargetMemory(MMFAR_Address, &MMFAR_Value, sizeof(MMFAR_Value), SWD::TRANSFER_32BIT) != sizeof(MMFAR_Value))
        {
            logError("Failed to read the MMFAR register.");
            return;
        }

        WriteStringToGdbConsole("\n    Fault Address: ");
        WriteHexValueToGdbConsole(MMFAR_Value);
    }
    if (memManageFaultStatusRegister & FPLazyStatePreservationBit)
        WriteStringToGdbConsole("\n    FP Lazy Preservation");

    if (memManageFaultStatusRegister & stackingErrorBit)
    {
        WriteStringToGdbConsole("\n    Stacking Error w/ SP = ");
        WriteHexValueToGdbConsole(readRegisterOnHaltedCore(SP));
    }
    if (memManageFaultStatusRegister & unstackingErrorBit)
    {
        WriteStringToGdbConsole("\n    Unstacking Error w/ SP = ");
        WriteHexValueToGdbConsole(readRegisterOnHaltedCore(SP));
    }
    if (memManageFaultStatusRegister & dataAccess)
    {
        WriteStringToGdbConsole("\n    Data Access");
    }

    if (memManageFaultStatusRegister & instructionFetch)
    {
        WriteStringToGdbConsole("\n    Instruction Fetch");
    }
}

void CpuCores::displayBusFaultCauseToGdbConsole()
{
    const uint32_t BFARValidBit = 1 << 7;
    const uint32_t FPLazyStatePreservationBit = 1 << 5;
    const uint32_t stackingErrorBit = 1 << 4;
    const uint32_t unstackingErrorBit = 1 << 3;
    const uint32_t impreciseDataAccessBit = 1 << 2;
    const uint32_t preciseDataAccessBit = 1 << 1;
    const uint32_t instructionPrefetch = 1;
    uint32_t       CFSR_Value = 0xBAADFEED;

    if (readTargetMemory(CFSR_Address, &CFSR_Value, sizeof(CFSR_Value), SWD::TRANSFER_32BIT) != sizeof(CFSR_Value))
    {
        logError("Failed to read the CFSR register.");
        return;
    }
    uint8_t busFaultStatusRegister = CFSR_Value >> 8;

    /* Check to make sure that there is a bus fault to display. */
    if (busFaultStatusRegister == 0)
    {
        return;
    }

    WriteStringToGdbConsole("\n**Bus Fault**");
    WriteStringToGdbConsole("\n  Status Register: ");
    WriteHexValueToGdbConsole(busFaultStatusRegister);

    if (busFaultStatusRegister & BFARValidBit)
    {
        const uint32_t BFAR_Address = 0xE000ED38;
        uint32_t       BFAR_Value;
        if (readTargetMemory(BFAR_Address, &BFAR_Value, sizeof(BFAR_Value), SWD::TRANSFER_32BIT) != sizeof(BFAR_Value))
        {
            logError("Failed to read the BFAR register.");
            return;
        }
        WriteStringToGdbConsole("\n    Fault Address: ");
        WriteHexValueToGdbConsole(BFAR_Value);
    }
    if (busFaultStatusRegister & FPLazyStatePreservationBit)
    {
        WriteStringToGdbConsole("\n    FP Lazy Preservation");
    }

    if (busFaultStatusRegister & stackingErrorBit)
    {
        WriteStringToGdbConsole("\n    Stacking Error w/ SP = ");
        WriteHexValueToGdbConsole(readRegisterOnHaltedCore(SP));
    }
    if (busFaultStatusRegister & unstackingErrorBit)
    {
        WriteStringToGdbConsole("\n    Unstacking Error w/ SP = ");
        WriteHexValueToGdbConsole(readRegisterOnHaltedCore(SP));
    }
    if (busFaultStatusRegister & impreciseDataAccessBit)
    {
        WriteStringToGdbConsole("\n    Imprecise Data Access");
    }

    if (busFaultStatusRegister & preciseDataAccessBit)
    {
        WriteStringToGdbConsole("\n    Precise Data Access");
    }

    if (busFaultStatusRegister & instructionPrefetch)
    {
        WriteStringToGdbConsole("\n    Instruction Prefetch");
    }
}

void CpuCores::displayUsageFaultCauseToGdbConsole()
{
    const uint32_t divideByZeroBit = 1 << 9;
    const uint32_t unalignedBit = 1 << 8;
    const uint32_t coProcessorAccessBit = 1 << 3;
    const uint32_t invalidPCBit = 1 << 2;
    const uint32_t invalidStateBit = 1 << 1;
    const uint32_t undefinedInstructionBit = 1;
    uint32_t       CFSR_Value = 0xBAADFEED;

    if (readTargetMemory(CFSR_Address, &CFSR_Value, sizeof(CFSR_Value), SWD::TRANSFER_32BIT) != sizeof(CFSR_Value))
    {
        logError("Failed to read the CFSR register.");
        return;
    }
    uint8_t usageFaultStatusRegister = CFSR_Value >> 16;

    /* Make sure that there is a usage fault to display. */
    if (usageFaultStatusRegister == 0)
    {
        return;
    }

    WriteStringToGdbConsole("\n**Usage Fault**");
    WriteStringToGdbConsole("\n  Status Register: ");
    WriteHexValueToGdbConsole(usageFaultStatusRegister);

    if (usageFaultStatusRegister & divideByZeroBit)
    {
        WriteStringToGdbConsole("\n    Divide by Zero");
    }

    if (usageFaultStatusRegister & unalignedBit)
    {
        WriteStringToGdbConsole("\n    Unaligned Access");
    }

    if (usageFaultStatusRegister & coProcessorAccessBit)
    {
        WriteStringToGdbConsole("\n    Coprocessor Access");
    }

    if (usageFaultStatusRegister & invalidPCBit)
    {
        WriteStringToGdbConsole("\n    Invalid Exception Return State");
    }

    if (usageFaultStatusRegister & invalidStateBit)
    {
        WriteStringToGdbConsole("\n    Invalid State");
    }

    if (usageFaultStatusRegister & undefinedInstructionBit)
    {
        WriteStringToGdbConsole("\n    Undefined Instruction");
    }
}

void CpuCores::reset()
{
    // Power down the DAP on any other target cores during reset.
    // The RP2040 had troubles reseting without this change.
    for (size_t i = 1 ; i < m_swdTargetCount ; i++)
    {
        m_swdTargets[i].disconnect();
    }
    m_swdTargetCount = 1;

    const uint32_t AIRCR_Address = 0xE000ED0C;
    const uint32_t AIRCR_KEY_Shift = 16;
    const uint32_t AIRCR_KEY_Mask = 0xFFFF << AIRCR_KEY_Shift;
    const uint32_t AIRCR_KEY_VALUE = 0x05FA << AIRCR_KEY_Shift;
    const uint32_t AIRCR_SYSRESETREQ_Bit = 1 << 2;
    uint32_t AIRCR_Value = 0;
    if (readTargetMemory(AIRCR_Address, &AIRCR_Value, sizeof(AIRCR_Value), SWD::TRANSFER_32BIT) != sizeof(AIRCR_Value))
    {
        logError("Failed to read AIRCR register for device reset.");
    }

    // Clear out the existing key value and use the special ones to enable writes.
    // Then set the SYSRESETREQ bit to request a device reset.
    AIRCR_Value = (AIRCR_Value & ~AIRCR_KEY_Mask) | AIRCR_KEY_VALUE | AIRCR_SYSRESETREQ_Bit;

    if (writeTargetMemory(AIRCR_Address, &AIRCR_Value, sizeof(AIRCR_Value), SWD::TRANSFER_32BIT) != sizeof(AIRCR_Value))
    {
        logError("Failed to write AIRCR register for device reset.");
    }
}

bool CpuCores::dispatchMonitorCommandToDevice(const char** ppArgs, size_t argCount)
{
    if (m_pDevice == NULL || m_pDevice->handleMonitorCommand == NULL)
    {
        // No device specific command handler so just return false.
        return false;
    }

    return m_pDevice->handleMonitorCommand(m_pDeviceObject, m_pSWD, ppArgs, argCount);
}

void CpuCores::enableResetVectorCatch()
{
    if (!setOrClearBitsInDEMCR(DEMCR_VC_CORERESET_Bit, true))
    {
        logError("Failed to set reset vector catch bit in DEMCR register.");
        return;
    }
}

bool CpuCores::flashBegin()
{
    return m_pDevice->flashBegin(m_pDeviceObject, m_pSWD);
}

bool CpuCores::flashErase(uint32_t addressStart, uint32_t length)
{
    return m_pDevice->flashErase(m_pDeviceObject, m_pSWD, addressStart, length);
}

bool CpuCores::flashProgram(uint32_t addressStart, const void* pBuffer, size_t bufferSize)
{
    return m_pDevice->flashProgram(m_pDeviceObject, m_pSWD, addressStart, pBuffer, bufferSize);
}

bool CpuCores::flashEnd()
{
    return m_pDevice->flashEnd(m_pDeviceObject, m_pSWD);
}

bool CpuCores::startCodeOnDevice(const CortexM_Registers* pRegistersIn)
{
    // UNDONE: Should take a bitmask of which register to write and read as it does all 16 now and this takes time.
    // Set the CPU registers required for executing code on device.
    bool result = true;
    for (uint32_t i = 0 ; i < count_of(pRegistersIn->registers) ; i++)
    {
        result &= writeCpuRegister(i, pRegistersIn->registers[i]);
    }
    if (!result)
    {
        logError("Failed to set registers for executing code on device.");
        return false;
    }

    // Want to keep interrupts masked while running this code and make sure that single stepping is disabled.
    if (!disableSingleStepAndInterrupts())
    {
        logError("Failed to disable single step and interrupts.");
        return false;
    }

    // Start the target CPU executing.
    if (!requestCpuToResume())
    {
        logError("Failed to start executing debugger code on target device.");
        reenableInterrupts();
        return false;
    }

    return true;
}

bool CpuCores::disableSingleStepAndInterrupts()
{
    uint32_t DHCSR_Val = 0;
    if (!readDHCSR(&DHCSR_Val))
    {
        return false;
    }

    DHCSR_Val = (DHCSR_Val & ~DHCSR_C_STEP_Bit) | DHCSR_C_MASKINTS_Bit;
    if (!writeDHCSR(DHCSR_Val))
    {
        return false;
    }
    return true;
}

bool CpuCores::reenableInterrupts()
{
    uint32_t DHCSR_Val = 0;
    if (!readDHCSR(&DHCSR_Val))
    {
        return false;
    }

    DHCSR_Val &= ~DHCSR_C_MASKINTS_Bit;
    if (!writeDHCSR(DHCSR_Val))
    {
        return false;
    }
    return true;
}

bool CpuCores::waitForCodeToHalt(CortexM_Registers* pRegistersOut, uint32_t timeout_ms)
{
    // Wait for executing code to complete.
    absolute_time_t endTime = make_timeout_time_ms(timeout_ms);
    uint32_t DHCSR_Val = 0;
    do
    {
        if (!readDHCSR(&DHCSR_Val))
        {
            break;
        }
    } while (!isCpuHalted(DHCSR_Val) && absolute_time_diff_us(get_absolute_time(), endTime) > 0);
    reenableInterrupts();
    if (!isCpuHalted(DHCSR_Val))
    {
        logError("Timeout out waiting for code execution on device to complete.");
        return false;
    }

    // Retrieve the register contents after the code has completed execution so that the caller can see their contents.
    bool result = true;
    for (uint32_t i = 0 ; i < count_of(pRegistersOut->registers) ; i++)
    {
        result &= readCpuRegister(i, &pRegistersOut->registers[i]);
    }
    if (!result)
    {
        logError("Failed to fetch registers after executing code on device.");
        return false;
    }

    return true;
}

bool CpuCores::runCodeOnDevice(CortexM_Registers* pRegistersInOut, uint32_t timeout_ms)
{
    if (!startCodeOnDevice(pRegistersInOut))
    {
        return false;
    }
    if (!waitForCodeToHalt(pRegistersInOut, timeout_ms))
    {
        return false;
    }
    return true;
}



static SWD        g_swdBus;
static GDBSocket  g_gdbSocket;
static CpuCores   g_cores;

static bool       g_isSwdConnected = false;
static bool       g_isNetworkConnected = false;
static bool       g_haltOnAttach = false;
static bool       g_wasStopFromGDB = false;
static bool       g_isResetting = false;
static bool       g_isInDebugger = false;
static bool       g_isDetaching = false;
static bool       g_doesHaltDebuggingNeedToBeEnabled = false;

static uint32_t   g_originalPC;
static bool       g_wasMemoryExceptionEncountered = false;


// Forward Function Declarations.
static void handleSwdDisconnect();
static void triggerMriCoreToExit();
static bool initSWD();
static bool waitForSwdAttach(uint32_t delayBetweenAttempts_ms);
static bool attemptSwdAttach();
static bool initNetwork();
static void innerDebuggerLoop();
static bool checkForNetworkDown();
static void detachDebugger();


void mainDebuggerLoop()
{
    if (!g_swdBus.init(DEFAULT_SWD_CLOCK_RATE, SWCLK_PIN, SWDIO_PIN))
    {
        logError("Failed to initialize the SWD port.");
        return;
    }

    g_isSwdConnected = false;
    g_isNetworkConnected = false;
    g_isInDebugger = false;
    g_cores.setConnectionFailureCallback(handleSwdDisconnect);
    while (true)
    {
        if (!g_isSwdConnected)
        {
            g_isSwdConnected = initSWD();
            if (!g_isSwdConnected)
            {
                logError("Failed to initialize SWD connection to debug target.");
                return;
            }

            // UNDONE: Should sample a physical switch here to determine if this should be done.
            g_haltOnAttach = HALT_ON_ATTACH;
        }

        if (!g_isNetworkConnected)
        {
            g_isNetworkConnected = initNetwork();
            if (!g_isNetworkConnected)
            {
                logError("Failed to initialize network.");
                return;
            }
        }

        // Initialize the MRI core.
        __try
        {
            mriInit("");
        }
        __catch
        {
            logError("Failed to initialize the MRI core.");
            return;
        }

        innerDebuggerLoop();

        if (!g_isSwdConnected)
        {
            g_cores.uninit();
        }
        if (!g_isNetworkConnected)
        {
            g_gdbSocket.uninit();
            cyw43_arch_deinit();
        }
    }
}

static void handleSwdDisconnect()
{
    g_isSwdConnected = false;
    triggerMriCoreToExit();
}

static void triggerMriCoreToExit()
{
    if (!g_isInDebugger)
    {
        return;
    }

    // Push an empty packet into MRI so that it will return to its command line handling loop and call
    // handleGDBCommand() which will actually request MRI to exit so that the main loop can try reconnecting.
    const uint8_t emptyPacket[] = "+$#00";
    g_gdbSocket.m_tcpToMriQueue.init();
    g_gdbSocket.m_tcpToMriQueue.write(emptyPacket, sizeof(emptyPacket)-1);
}

static bool initSWD()
{
    bool returnCode = false;

    returnCode = attemptSwdAttach();
    if (!returnCode)
    {
        returnCode = waitForSwdAttach(DELAY_BETWEEN_SWD_ATTACH_ATTEMPTS_MS);
    }

    return returnCode;
}

static bool waitForSwdAttach(uint32_t delayBetweenAttempts_ms)
{
    logErrorDisable();
    bool targetFound;
    uint32_t iteration = 0;
    do
    {
        sleep_ms(delayBetweenAttempts_ms);
        if (iteration++ % LOG_EVERY_N_SWD_ATTACH_ATTEMPTS == 0)
        {
            logInfo("Waiting for SWD target to be attached and powered up...");
        }
        targetFound = attemptSwdAttach();
    } while (!targetFound);
    logErrorEnable();

    return true;
}

static bool attemptSwdAttach()
{
    // Search through all known SWD DPv2 targets to see if any are found.
    // If that fails, try detecting SWD targets which don't go dormant, after making to switch SWJ-DP targets into SWD
    // mode.
    if (g_swdBus.searchForKnownSwdTarget())
    {
        // Have found one of the SWD DPv2 targets known by this debugger.
        logInfoF("Found DPv2 SWD Target=0x%08X with DPIDR=0x%08lX", g_swdBus.getTarget(), g_swdBus.getDPIDR());
    }
    else if (g_swdBus.sendJtagToSwdSequence())
    {
        // Have found a non-dormant SWD target.
        logInfoF("Found SWD Target with DPIDR=0x%08lX", g_swdBus.getDPIDR());
    }
    else
    {
        logError("No SWD Targets found.");
        return false;
    }

    bool result = g_cores.init(&g_swdBus);
    logInfo("SWD initialization complete!");
    return result;
}


static bool initNetwork()
{
    logInfo("Initializing network...");
    if (cyw43_arch_init())
    {
        logError("Wi-Fi chip failed to initialize.");
        return false;
    }
    cyw43_arch_enable_sta_mode();

    int connectionResult;
    do
    {
        logInfo("Attempting to connect to Wi-Fi router...");
        connectionResult = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, WIFI_ROUTER_TIMEOUT_MS);
    } while (connectionResult != 0);
    logInfo("Connected to Wi-Fi router.");

    if (!g_gdbSocket.init())
    {
        logError("Failed to initialize GDB socket.");
        return false;
    }

    return true;
}

static void innerDebuggerLoop()
{
    bool hasCpuHalted = false;
    while (g_isSwdConnected && g_isNetworkConnected)
    {
        // Check the WiFi link to see if it has gone down.
        if (checkForNetworkDown())
        {
            continue;
        }

        // Handle setting up halting debug mode if needed.
        if (g_doesHaltDebuggingNeedToBeEnabled)
        {
            if (g_cores.enableHaltDebugging(READ_DHCSR_TIMEOUT_MS))
            {
                g_doesHaltDebuggingNeedToBeEnabled = false;
            }
        }

        // See if the target should be halted because of attach or because GDB has sent a command via TCP/IP.
        if (g_haltOnAttach || (g_gdbSocket.isGdbConnected() && !g_gdbSocket.m_tcpToMriQueue.isEmpty()))
        {
            logInfoF("%s has requested a CPU halt.", g_haltOnAttach ? "User" : "GDB");
            g_wasStopFromGDB = true;
            g_haltOnAttach = false;
            g_cores.requestCpuToHalt();
        }

        // Query current state of CPU.
        CpuCores::CpuState cpuState = 0;
        if (!g_cores.getCpuState(&cpuState, READ_DHCSR_TIMEOUT_MS))
        {
            logInfo("SWD target is no longer responding. Will attempt to reattach.");
            g_isSwdConnected = false;
            continue;
        }
        if (g_cores.isDeviceResetting(cpuState))
        {
            if (g_isResetting)
            {
                g_cores.resetCompleted();
                g_isResetting = false;
                logInfo("Device RESET request completed.");
            }
            else
            {
                logInfo("External device RESET detected.");
            }
            continue;
        }
        if (!hasCpuHalted && g_cores.isCpuHalted(cpuState))
        {
            logInfo("CPU has halted.");
            hasCpuHalted = true;
        }
        // UNDONE: Should check the sleep, lockup, retire bits in the DHCSR as well.

        if (!g_isResetting && g_gdbSocket.isGdbConnected() && hasCpuHalted)
        {
            hasCpuHalted = false;

            g_cores.enterMriCore();
            if (!g_isSwdConnected || !g_isNetworkConnected)
            {
                continue;
            }
            else if (g_isResetting)
            {
                logInfo("GDB has requested device RESET.");
            }
            else
            {
                g_cores.resume();

                if (g_isDetaching)
                {
                    detachDebugger();
                }
                else
                {
                    logInfoF("CPU execution has been resumed. %s", Platform_IsSingleStepping() ? "Single stepping enabled." : "");
                }
            }
        }
    }
}

static bool checkForNetworkDown()
{
    if (!g_isNetworkConnected)
    {
        return true;
    }

    if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP)
    {
        logError("WiFi link is no longer up.");
        g_isNetworkConnected = false;
        triggerMriCoreToExit();
        return true;
    }
    return false;
}

static void detachDebugger()
{
    g_cores.disconnect();
    g_swdBus.uninit();
    g_gdbSocket.closeClient();

    logInfo("CPU execution has been resumed and debugger detached.");
    logInfo("Debug hardware will need to be reset to reconnect.");
    while (true)
    {
        __wfe();
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
static void checkForDeviceReset();
static void waitToReceiveData();

int Platform_CommHasReceiveData(void)
{
    int hasReceiveData = !g_gdbSocket.m_tcpToMriQueue.isEmpty();
    if (!hasReceiveData)
    {
        // Only check for network and SWD disconnect when there is no data from GDB to process.
        checkForNetworkDown();
        checkForDeviceReset();
    }
    return hasReceiveData;
}

static void checkForDeviceReset()
{
    if (!g_isSwdConnected)
    {
        return;
    }

    // See if the Debug Halting Control and Status Register indicates that the CPU has been reset.
    CpuCores::CpuState state = 0;
    bool result = g_cores.getCpuState(&state, 0);
    if (!result)
    {
        return;
    }
    if (g_cores.isDeviceResetting(state))
    {
        logInfo("Device RESET detected while halted in GDB.");
        g_isResetting = false;
        g_isSwdConnected = false;
        triggerMriCoreToExit();
    }
}

int  Platform_CommHasTransmitCompleted(void)
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
// It is used to perform any platform specific initialization. In our case we want to initialize the DWT (Data
// Watchpoint) and FPB (Flash Patch and Breakpoint) units on the Cortex-M CPU.
// *********************************************************************************************************************
void Platform_Init(Token* pParameterTokens)
{
    g_cores.initForDebugging();
    g_doesHaltDebuggingNeedToBeEnabled = true;
}




// *********************************************************************************************************************
// Routines called by the MRI core each time the CPU is halted and resumed.
// *********************************************************************************************************************
void Platform_EnteringDebugger(void)
{
    g_isInDebugger = true;
    g_wasMemoryExceptionEncountered = false;
    g_originalPC = Platform_GetProgramCounter();
    g_cores.enteringDebugger();
    // UNDONE: Might not need it once I hook in the RTOS code which tracks per thread state.
    Platform_DisableSingleStep();
}


void Platform_LeavingDebugger(void)
{
    g_wasStopFromGDB = false;
    g_cores.leavingDebugger();
    g_isInDebugger = false;
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
static char g_packetBuffer[PACKET_SIZE];

char* Platform_GetPacketBuffer(void)
{
    return g_packetBuffer;
}

uintmri_t Platform_GetPacketBufferSize(void)
{
    return sizeof(g_packetBuffer);
}




// *********************************************************************************************************************
// Routine called by the MRI core to create the T response to be sent back to GDB on debug stops.
// *********************************************************************************************************************
static void sendRegisterForTResponse(Buffer* pBuffer, uint8_t registerOffset, uint32_t registerValue);
static size_t writeBytesToBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount);

void Platform_WriteTResponseRegistersToBuffer(Buffer* pBuffer)
{
    // Source level GDB single stepping can execute more quickly if the T packet contains R7 (the frame pointer), SP,
    // LR, and the PC so that GDB doesn't have to waste time by sending a 'g' packet to obtain them.
    sendRegisterForTResponse(pBuffer, R7, g_cores.readRegisterOnHaltedCore(R7));
    sendRegisterForTResponse(pBuffer, SP, g_cores.readRegisterOnHaltedCore(SP));
    sendRegisterForTResponse(pBuffer, LR, g_cores.readRegisterOnHaltedCore(LR));
    sendRegisterForTResponse(pBuffer, PC, g_cores.readRegisterOnHaltedCore(PC));
}

static void sendRegisterForTResponse(Buffer* pBuffer, uint8_t registerOffset, uint32_t registerValue)
{
    Buffer_WriteByteAsHex(pBuffer, registerOffset);
    Buffer_WriteChar(pBuffer, ':');
    writeBytesToBufferAsHex(pBuffer, &registerValue, sizeof(registerValue));
    Buffer_WriteChar(pBuffer, ';');
}

static size_t writeBytesToBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount)
{
    uint8_t* pByte = (uint8_t*)pBytes;
    size_t   i;

    for (i = 0 ; i < byteCount ; i++)
    {
        __try
        {
            Buffer_WriteByteAsHex(pBuffer, *pByte++);
        }
        __catch
        {
            __rethrow_and_return(i);
        }
    }
    return byteCount;
}




// *********************************************************************************************************************
// Routines called by the MRI core to read and write memory on the target device.
// *********************************************************************************************************************
uint32_t Platform_MemRead32(uintmri_t address)
{
    g_wasMemoryExceptionEncountered = false;
    uint32_t data = 0;
    uint32_t bytesRead = g_cores.readMemory(address, &data, sizeof(data), SWD::TRANSFER_32BIT);
    if (bytesRead != sizeof(data))
    {
        g_wasMemoryExceptionEncountered = true;
        return 0;
    }
    return data;
}

uint16_t Platform_MemRead16(uintmri_t address)
{
    g_wasMemoryExceptionEncountered = false;
    uint16_t data = 0;
    uint32_t bytesRead = g_cores.readMemory(address, &data, sizeof(data), SWD::TRANSFER_16BIT);
    if (bytesRead != sizeof(data))
    {
        g_wasMemoryExceptionEncountered = true;
        return 0;
    }
    return data;
}

uint8_t Platform_MemRead8(uintmri_t address)
{
    g_wasMemoryExceptionEncountered = false;
    uint8_t data = 0;
    uint32_t bytesRead = g_cores.readMemory(address, &data, sizeof(data), SWD::TRANSFER_8BIT);
    if (bytesRead != sizeof(data))
    {
        g_wasMemoryExceptionEncountered = true;
        return 0;
    }
    return data;
}

void Platform_MemWrite32(uintmri_t address, uint32_t value)
{
    g_wasMemoryExceptionEncountered = false;
    uint32_t bytesWritten = g_cores.writeMemory(address, &value, sizeof(value), SWD::TRANSFER_32BIT);
    if (bytesWritten != sizeof(value))
    {
        g_wasMemoryExceptionEncountered = true;
    }
}

void Platform_MemWrite16(uintmri_t address, uint16_t value)
{
    g_wasMemoryExceptionEncountered = false;
    uint32_t bytesWritten = g_cores.writeMemory(address, &value, sizeof(value), SWD::TRANSFER_16BIT);
    if (bytesWritten != sizeof(value))
    {
        g_wasMemoryExceptionEncountered = true;
    }
}

void Platform_MemWrite8(uintmri_t address, uint8_t value)
{
    g_wasMemoryExceptionEncountered = false;
    uint32_t bytesWritten = g_cores.writeMemory(address, &value, sizeof(value), SWD::TRANSFER_8BIT);
    if (bytesWritten != sizeof(value))
    {
        g_wasMemoryExceptionEncountered = true;
    }
}

int Platform_WasMemoryFaultEncountered()
{
    return g_wasMemoryExceptionEncountered;
}

void Platform_SyncICacheToDCache(uintmri_t address, uintmri_t size)
{
    // UNDONE: Implement and test on Portenta-H7.
}




// *********************************************************************************************************************
// Replacements for routines from memory.c to make them faster.
// *********************************************************************************************************************
// UNDONE: Be aware that the TAR read in calculateTransferCount() might return 0 once it goes past the end of valid memory.
static uint32_t readMemoryBytesIntoHexBuffer(Buffer* pBuffer, uintmri_t address, uintmri_t readByteCount);
static uint32_t readMemoryHalfWordIntoHexBuffer(Buffer* pBuffer, uintmri_t address);
static int isNotHalfWordAligned(uintmri_t address);
static uint32_t readMemoryWordIntoHexBuffer(Buffer* pBuffer, uintmri_t address);
static int isWordAligned(uintmri_t value);

uintmri_t ReadMemoryIntoHexBuffer(Buffer* pBuffer, uintmri_t address, uintmri_t readByteCount)
{
    switch (readByteCount)
    {
    case 2:
        return readMemoryHalfWordIntoHexBuffer(pBuffer, address);
    case 4:
        return readMemoryWordIntoHexBuffer(pBuffer, address);
    default:
        return readMemoryBytesIntoHexBuffer(pBuffer, address, readByteCount);
    }
}

static uint32_t readMemoryBytesIntoHexBuffer(Buffer* pBuffer, uintmri_t address, uintmri_t readByteCount)
{
    uint32_t buffer[(readByteCount+sizeof(uint32_t)-1) / sizeof(uint32_t)];

    uint32_t bytesRead = 0;
    if (isWordAligned(readByteCount) && isWordAligned(address))
    {
        bytesRead = g_cores.readMemory(address, buffer, readByteCount, SWD::TRANSFER_32BIT);
    }
    else
    {
        bytesRead = g_cores.readMemory(address, buffer, readByteCount, SWD::TRANSFER_8BIT);
    }
    return writeBytesToBufferAsHex(pBuffer, buffer, bytesRead);
}

static uint32_t readMemoryHalfWordIntoHexBuffer(Buffer* pBuffer, uintmri_t address)
{
    uint16_t value;

    if (isNotHalfWordAligned(address))
    {
        return readMemoryBytesIntoHexBuffer(pBuffer, address, 2);
    }

    value = Platform_MemRead16(address);
    if (Platform_WasMemoryFaultEncountered())
    {
        return 0;
    }
    return writeBytesToBufferAsHex(pBuffer, &value, sizeof(value));
}

static int isNotHalfWordAligned(uintmri_t address)
{
    return address & 1;
}

static uint32_t readMemoryWordIntoHexBuffer(Buffer* pBuffer, uintmri_t address)
{
    uint32_t value;

    if (!isWordAligned(address))
    {
        return readMemoryBytesIntoHexBuffer(pBuffer, address, 4);
    }

    value = Platform_MemRead32(address);
    if (Platform_WasMemoryFaultEncountered())
    {
        return 0;
    }
    return writeBytesToBufferAsHex(pBuffer, &value, sizeof(value));
}

static int isWordAligned(uintmri_t value)
{
    return (value & 3) == 0;
}


static int writeHexBufferToByteMemory(Buffer* pBuffer, uintmri_t address, uintmri_t writeByteCount);
static int writeHexBufferToHalfWordMemory(Buffer* pBuffer, uintmri_t address);
static int readBytesFromHexBuffer(Buffer* pBuffer, void* pvMemory, size_t length);
static int writeHexBufferToWordMemory(Buffer* pBuffer, uintmri_t address);
int WriteHexBufferToMemory(Buffer* pBuffer, uintmri_t address, uintmri_t writeByteCount)
{
    switch (writeByteCount)
    {
    case 2:
        return writeHexBufferToHalfWordMemory(pBuffer, address);
    case 4:
        return writeHexBufferToWordMemory(pBuffer, address);
    default:
        return writeHexBufferToByteMemory(pBuffer, address, writeByteCount);
    }
}

static int writeHexBufferToByteMemory(Buffer* pBuffer, uintmri_t address, uintmri_t writeByteCount)
{
    uint32_t buffer[(writeByteCount+sizeof(uint32_t)-1) / sizeof(uint32_t)];

    if (writeByteCount > count_of(buffer))
    {
        writeByteCount = count_of(buffer);
    }
    if (readBytesFromHexBuffer(pBuffer, buffer, writeByteCount) == 0)
    {
        return 0;
    }

    uint32_t bytesWritten = 0;
    if (isWordAligned(writeByteCount) && isWordAligned(address))
    {
        bytesWritten = g_cores.writeMemory(address, buffer, writeByteCount, SWD::TRANSFER_32BIT);
    }
    else
    {
        bytesWritten = g_cores.writeMemory(address, buffer, writeByteCount, SWD::TRANSFER_8BIT);
    }

    return bytesWritten;
}

static int writeHexBufferToHalfWordMemory(Buffer* pBuffer, uintmri_t address)
{
    uint16_t value;

    if (isNotHalfWordAligned(address))
    {
        return writeHexBufferToByteMemory(pBuffer, address, 2);
    }

    if (!readBytesFromHexBuffer(pBuffer, &value, sizeof(value)))
    {
        return 0;
    }

    Platform_MemWrite16(address, value);
    if (Platform_WasMemoryFaultEncountered())
    {
        return 0;
    }

    return 1;
}

static int readBytesFromHexBuffer(Buffer* pBuffer, void* pv, size_t length)
{
    uint8_t* pBytes = (uint8_t*)pv;
    while (length--)
    {
        __try
        {
            *pBytes++ = Buffer_ReadByteAsHex(pBuffer);
        }
        __catch
        {
            __rethrow_and_return(0);
        }
    }
    return 1;
}

static int writeHexBufferToWordMemory(Buffer* pBuffer, uintmri_t address)
{
    uint32_t value;

    if (!isWordAligned(address))
    {
        return writeHexBufferToByteMemory(pBuffer, address, 4);
    }

    if (!readBytesFromHexBuffer(pBuffer, &value, sizeof(value)))
    {
        return 0;
    }

    Platform_MemWrite32(address, value);
    if (Platform_WasMemoryFaultEncountered())
    {
        return 0;
    }

    return 1;
}


static int  writeBinaryBufferToByteMemory(Buffer*  pBuffer, uintmri_t address, uintmri_t writeByteCount);
static int  writeBinaryBufferToHalfWordMemory(Buffer* pBuffer, uintmri_t address);
static int readBytesFromBinaryBuffer(Buffer*  pBuffer, void* pvMemory, uint32_t writeByteCount);
static int  writeBinaryBufferToWordMemory(Buffer* pBuffer, uintmri_t address);
int WriteBinaryBufferToMemory(Buffer* pBuffer, uintmri_t address, uintmri_t writeByteCount)
{
    switch (writeByteCount)
    {
    case 2:
        return writeBinaryBufferToHalfWordMemory(pBuffer, address);
    case 4:
        return writeBinaryBufferToWordMemory(pBuffer, address);
    default:
        return writeBinaryBufferToByteMemory(pBuffer, address, writeByteCount);
    }
}

static int writeBinaryBufferToByteMemory(Buffer*  pBuffer, uintmri_t address, uintmri_t writeByteCount)
{
    uint32_t buffer[(writeByteCount+sizeof(uint32_t)-1) / sizeof(uint32_t)];
    if (readBytesFromBinaryBuffer(pBuffer, buffer, writeByteCount) == 0)
    {
        return 0;
    }

    uint32_t bytesWritten = 0;
    if (isWordAligned(writeByteCount) && isWordAligned(address))
    {
        bytesWritten = g_cores.writeMemory(address, buffer, writeByteCount, SWD::TRANSFER_32BIT);
    }
    else
    {
        bytesWritten = g_cores.writeMemory(address, buffer, writeByteCount, SWD::TRANSFER_8BIT);
    }

    return bytesWritten;

}

static int writeBinaryBufferToHalfWordMemory(Buffer* pBuffer, uintmri_t address)
{
    uint16_t value;

    if (isNotHalfWordAligned(address))
    {
        return writeBinaryBufferToByteMemory(pBuffer, address, 2);
    }

    if (!readBytesFromBinaryBuffer(pBuffer, &value, sizeof(value)))
    {
        return 0;
    }

    Platform_MemWrite16(address, value);
    if (Platform_WasMemoryFaultEncountered())
    {
        return 0;
    }

    return 1;
}

static int readBytesFromBinaryBuffer(Buffer*  pBuffer, void* pvMemory, uint32_t writeByteCount)
{
    uint8_t* p = (uint8_t*) pvMemory;

    while (writeByteCount-- > 0)
    {
        __try
        {
            *p++ = Buffer_ReadChar(pBuffer);
        }
        __catch
        {
            __rethrow_and_return(0);
        }
    }

    return 1;
}

static int writeBinaryBufferToWordMemory(Buffer* pBuffer, uintmri_t address)
{
    uint32_t value;

    if (!isWordAligned(address))
    {
        return writeBinaryBufferToByteMemory(pBuffer, address, 4);
    }

    if (!readBytesFromBinaryBuffer(pBuffer, &value, sizeof(value)))
    {
        return 0;
    }

    Platform_MemWrite32(address, value);
    if (Platform_WasMemoryFaultEncountered())
    {
        return 0;
    }

    return 1;
}




// *********************************************************************************************************************
// Routines called by the MRI core to enable/disable single stepping on the target CPU.
// *********************************************************************************************************************
void Platform_EnableSingleStep(void)
{
    g_cores.enableSingleStep();
}

void Platform_DisableSingleStep(void)
{
    g_cores.disableSingleStep();
}

int Platform_IsSingleStepping(void)
{
    return g_cores.isSingleStepping();
}




// *********************************************************************************************************************
// Routines called by the MRI core to access the CPU's program counter.
// *********************************************************************************************************************
static uint16_t getFirstHalfWordOfCurrentInstruction(void);
static uint16_t throwingMemRead16(uint32_t address);
static bool isInstruction32Bit(uint16_t firstWordOfInstruction);

uintmri_t Platform_GetProgramCounter(void)
{
    return g_cores.readRegisterOnHaltedCore(PC);
}

void Platform_SetProgramCounter(uintmri_t newPC)
{
    return g_cores.writeRegisterOnHaltedCore(PC, newPC);
}

void Platform_AdvanceProgramCounterToNextInstruction(void)
{
    uint16_t  firstWordOfCurrentInstruction;

    __try
    {
        firstWordOfCurrentInstruction = getFirstHalfWordOfCurrentInstruction();
    }
    __catch
    {
        // Will get here if PC isn't pointing to valid memory so don't bother to advance.
        clearExceptionCode();
        return;
    }

    if (isInstruction32Bit(firstWordOfCurrentInstruction))
    {
        /* 32-bit Instruction. */
        Platform_SetProgramCounter(Platform_GetProgramCounter() + sizeof(uint32_t));
    }
    else
    {
        /* 16-bit Instruction. */
        Platform_SetProgramCounter(Platform_GetProgramCounter() + sizeof(uint16_t));
    }
}

static uint16_t getFirstHalfWordOfCurrentInstruction(void)
{
    return throwingMemRead16(Platform_GetProgramCounter());
}

static uint16_t throwingMemRead16(uint32_t address)
{
    uint16_t instructionWord = Platform_MemRead16(address);
    if (Platform_WasMemoryFaultEncountered())
    {
        __throw_and_return(memFaultException, 0);
    }
    return instructionWord;
}

static bool isInstruction32Bit(uint16_t firstWordOfInstruction)
{
    uint16_t maskedOffUpper5BitsOfWord = firstWordOfInstruction & 0xF800;

    // 32-bit instructions start with 0b11101, 0b11110, 0b11111 according to page A5-152 of the
    // ARMv7-M Architecture Manual.
    return !!(maskedOffUpper5BitsOfWord == 0xE800 ||
              maskedOffUpper5BitsOfWord == 0xF000 ||
              maskedOffUpper5BitsOfWord == 0xF800);
}

int Platform_WasProgramCounterModifiedByUser(void)
{
    return Platform_GetProgramCounter() != g_originalPC;
}



// *********************************************************************************************************************
// Routines called by the MRI core to set and clear breakpoints.
// *********************************************************************************************************************
static bool doesKindIndicate32BitInstruction(uint32_t kind);

__throws void Platform_SetHardwareBreakpointOfGdbKind(uintmri_t address, uintmri_t kind)
{
    bool is32BitInstruction = false;

    __try
    {
        is32BitInstruction = doesKindIndicate32BitInstruction(kind);
    }
    __catch
    {
        __rethrow;
    }

    uint32_t FPBBreakpointComparator = g_cores.setBreakpoint(address, is32BitInstruction);
    if (FPBBreakpointComparator == 0)
    {
        __throw(exceededHardwareResourcesException);
    }
    logInfoF("Hardware breakpoint set at address 0x%08X.", address);
}

static bool doesKindIndicate32BitInstruction(uint32_t kind)
{
    switch (kind)
    {
        case 2:
            return false;
        case 3:
        case 4:
            return true;
        default:
            __throw_and_return(invalidArgumentException, false);
    }
}

__throws void Platform_SetHardwareBreakpoint(uint32_t address)
{
    uint16_t  firstInstructionWord = 0;
     __try
    {
        firstInstructionWord = throwingMemRead16(address);
    }
    __catch
    {
        __rethrow;
    }

    uint32_t FPBBreakpointComparator = g_cores.setBreakpoint(address, isInstruction32Bit(firstInstructionWord));
    if (FPBBreakpointComparator == 0)
    {
        __throw(exceededHardwareResourcesException);
    }
    logInfoF("Hardware breakpoint set at address 0x%08lX.", address);
}

__throws void Platform_ClearHardwareBreakpointOfGdbKind(uintmri_t address, uintmri_t kind)
{
    bool is32BitInstruction = false;
    __try
    {
        is32BitInstruction = doesKindIndicate32BitInstruction(kind);
    }
    __catch
    {
        __rethrow;
    }

    g_cores.clearBreakpoint(address, is32BitInstruction);
}

__throws void Platform_ClearHardwareBreakpoint(uintmri_t address)
{
    uint16_t  firstInstructionWord;

     __try
    {
        firstInstructionWord = throwingMemRead16(address);
    }
    __catch
    {
        __rethrow;
    }

    g_cores.clearBreakpoint(address, isInstruction32Bit(firstInstructionWord));
}




// *********************************************************************************************************************
// Routines called by the MRI core to set and clear watch points.
// *********************************************************************************************************************
__throws void Platform_SetHardwareWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type)
{
    g_cores.setWatchpoint(address, size, type);
}

__throws void Platform_ClearHardwareWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type)
{
    g_cores.clearWatchpoint(address, size, type);
}



// *********************************************************************************************************************
// Routines called by the MRI core when dealing with exception/fault causes.
// *********************************************************************************************************************
uint8_t Platform_DetermineCauseOfException(void)
{
    return g_cores.determineCauseOfException(g_wasStopFromGDB);
}

PlatformTrapReason Platform_GetTrapReason(void)
{
    return g_cores.getTrapReason();
}

void Platform_DisplayFaultCauseToGdbConsole(void)
{
    g_cores.displayFaultCauseToGdbConsole();
}



// *********************************************************************************************************************
// Routines called by the MRI core to retrieve the XML describing the CPU's register context.
// *********************************************************************************************************************
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

uintmri_t Platform_GetTargetXmlSize(void)
{
    if (g_cores.hasFPU() >= CpuCores::FPU_MAYBE)
    {
        return sizeof(g_targetFpuXML);
    }
    else
    {
        return sizeof(g_targetXML);
    }
}

const char* Platform_GetTargetXml(void)
{
    if (g_cores.hasFPU() >= CpuCores::FPU_MAYBE)
    {
        return g_targetFpuXML;
    }
    else
    {
        return g_targetXML;
    }
}



// *********************************************************************************************************************
// Routines called by the MRI core to retrieve the XML describing the CPU's memory layout.
// *********************************************************************************************************************
uintmri_t Platform_GetDeviceMemoryMapXmlSize(void)
{
    return g_cores.getMemoryLayoutXMLSize();
}

const char* Platform_GetDeviceMemoryMapXml(void)
{
    return g_cores.getMemoryLayoutXML();
}



// *********************************************************************************************************************
// Routines called by the MRI core to reset the device.
// *********************************************************************************************************************
void Platform_ResetDevice(void)
{
    g_cores.reset();
    g_isResetting = true;
}



// *********************************************************************************************************************
// Platform_HandleGDBCommand() is called by the MRI core to give platforms a chance to override and handle any command
// sent from GDB. The mri-swd platform implements vFlash* support to enable programming of targets. It also implements
// code which can force the MRI core to return to the main debugger loop when TCP/IP or SWD issues are encountered that
// require waiting for the target or the WiFi router to reappear.
// *********************************************************************************************************************
static uint32_t forceMriCoreToReturn(Buffer* pBuffer);
static uint32_t handleMonitorCommand(Buffer* pBuffer);
static bool dispatchMonitorCommandToDevice(Buffer* pBuffer);
static uint32_t handleMonitorDetachCommand();
static uint32_t handleMonitorResetCommand(Buffer* pBuffer);
static uint32_t handleMonitorVersionCommand(Buffer* pBuffer);
static uint32_t handleMonitorHelpCommand();
static uint32_t handleFlashEraseCommand(Buffer* pBuffer);
static void sendOkResponseWithNewPacketBuffer();
static uint32_t handleFlashWriteCommand(Buffer* pBuffer);
static uint32_t handleFlashDoneCommand(Buffer* pBuffer);
uint32_t Platform_HandleGDBCommand(Buffer* pBuffer)
{
    if (!g_isSwdConnected || !g_isNetworkConnected)
    {
        // Something has happened to the target device or the WiFi router connection.
        // Force the MRI core to return and let the main loop attempt to reconnect.
        return forceMriCoreToReturn(pBuffer);
    }

    const char rcmdCommand[] = "qRcmd";
    const char vFlashEraseCommand[] = "vFlashErase";
    const char vFlashWriteCommand[] = "vFlashWrite";
    const char vFlashDoneCommand[] = "vFlashDone";

    Buffer_Reset(pBuffer);
    if (Buffer_MatchesString(pBuffer, rcmdCommand, sizeof(rcmdCommand)-1))
    {
        return handleMonitorCommand(pBuffer);
    }
    else if (Buffer_MatchesString(pBuffer, vFlashEraseCommand, sizeof(vFlashEraseCommand)-1))
    {
        return handleFlashEraseCommand(pBuffer);
    }
    else if (Buffer_MatchesString(pBuffer, vFlashWriteCommand, sizeof(vFlashWriteCommand)-1))
    {
        return handleFlashWriteCommand(pBuffer);
    }
    else if (Buffer_MatchesString(pBuffer, vFlashDoneCommand, sizeof(vFlashDoneCommand)-1))
    {
        return handleFlashDoneCommand(pBuffer);
    }
    else
    {
        // Returning 0 means that MRI should handle this command instead.
        return 0;
    }
}

static uint32_t forceMriCoreToReturn(Buffer* pBuffer)
{
    logInfo("Forcing MRI core to exit due to network or SWD error.");
    PrepareStringResponse("E99");
    return HANDLER_RETURN_RESUME_PROGRAM | HANDLER_RETURN_RETURN_IMMEDIATELY;
}

static uint32_t handleMonitorCommand(Buffer* pBuffer)
{
    const char detach[] = "detach";
    const char reset[] = "reset";
    const char help[] = "help";
    const char version[] = "version";

    if (!Buffer_IsNextCharEqualTo(pBuffer, ','))
    {
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
        return 0;
    }

    // "monitor help" can be handled by both the device specific code and this more generic code to allow listing of all
    // commands supported by the core and the device.
    Buffer bufferCopy = *pBuffer;
    bool isMonitorHelp = Buffer_MatchesHexString(&bufferCopy, help, sizeof(help)-1);

    // Allow the device specific code the chance to handle the monitor command.
    bool handledByDevice = dispatchMonitorCommandToDevice(pBuffer);
    if (isMonitorHelp)
    {
        // Even if device specific handler handled "monitor help" command, we still want to list the core commands
        // supported.
        return handleMonitorHelpCommand();
    }
    if (handledByDevice)
    {
        return HANDLER_RETURN_HANDLED;
    }

    // Command wasn't handled by the device specific code so handle it in this core code instead.
    if (Buffer_MatchesHexString(pBuffer, detach, sizeof(detach)-1))
    {
        return handleMonitorDetachCommand();
    }
    else if (Buffer_MatchesHexString(pBuffer, reset, sizeof(reset)-1))
    {
        return handleMonitorResetCommand(pBuffer);
    }
    else if (Buffer_MatchesHexString(pBuffer, version, sizeof(version)-1))
    {
        return handleMonitorVersionCommand(pBuffer);
    }
    else
    {
        // Returning 0 means that MRI should handle this command instead.
        return 0;
    }
}

static bool dispatchMonitorCommandToDevice(Buffer* pBuffer)
{
    // Convert the hex string into a normal C string.
    Buffer bufferCopy = *pBuffer;
    size_t commandLength = Buffer_BytesLeft(&bufferCopy);
    if (commandLength % 2 != 0)
    {
        // GDB didn't send an even number of hex digits for the command string.
        return false;
    }
    commandLength = commandLength / 2 + 1;
    char commandCopy[commandLength];

    char* pDest = &commandCopy[0];
    for (size_t i = 0 ; i < commandLength - 1 ; i++)
    {
        *pDest++ = Buffer_ReadByteAsHex(&bufferCopy);
    }
    assert ( pDest == &commandCopy[commandLength-1] );
    *pDest++ = '\0';

    // Break up whitespace separated tokens into argv, argc parameters.
    const char* arguments[10];
    size_t argCount = 0;
    bool skippingWhitespace = true;
    for (char* pCurr = &commandCopy[0] ; *pCurr != '\0' ; pCurr++)
    {
        char byte = *pCurr;
        if (byte == ' ' || byte == '\t' || byte == '\r' || byte == '\n')
        {
            if (!skippingWhitespace)
            {
                *pCurr = '\0';
            }
            skippingWhitespace = true;
        }
        else if (skippingWhitespace)
        {
            if (argCount >= count_of(arguments))
            {
                break;
            }
            arguments[argCount++] = pCurr;
            skippingWhitespace = false;
        }
    }

    return g_cores.dispatchMonitorCommandToDevice(&arguments[0], argCount);
}

static uint32_t handleMonitorDetachCommand()
{
    g_isDetaching = true;
    WriteStringToGdbConsole("Will detach on next continue. Debugger will require reset to reconnect.\r\n");
    PrepareStringResponse("OK");
    return HANDLER_RETURN_HANDLED;
}

static uint32_t handleMonitorResetCommand(Buffer* pBuffer)
{
    // If no optional parameters are specified then let MRI core handle this request instead.
    if (Buffer_BytesLeft(pBuffer) == 0)
    {
        return 0;
    }

    // Next character should be hex version of space character.
    if (Buffer_ReadByteAsHex(pBuffer) != ' ')
    {
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
        return HANDLER_RETURN_HANDLED;
    }

    const char halt[] = "halt";
    if (Buffer_MatchesHexString(pBuffer, halt, sizeof(halt)-1))
    {
        // A request to reset and then halt in Reset Handler has been made.
        RequestResetOnNextContinue();
        g_cores.enableResetVectorCatch();
        WriteStringToGdbConsole("Will reset and then halt on next continue.\r\n");
        PrepareStringResponse("OK");
        return HANDLER_RETURN_HANDLED;
    }

    PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
    return HANDLER_RETURN_HANDLED;
}

static uint32_t handleMonitorVersionCommand(Buffer* pBuffer)
{
    WriteStringToGdbConsole("|mri-swd| Monitor for Remote Inspection - SWD Edition\r\n");
    WriteStringToGdbConsole(" mri-swd  Version: " MRI_SWD_VERSION_STRING " [" MRI_SWD_BRANCH "]\r\n");
    WriteStringToGdbConsole(" mri-core Version: " MRI_VERSION_STRING "\r\n");
    PrepareStringResponse("OK");
    return HANDLER_RETURN_HANDLED;
}

static uint32_t handleMonitorHelpCommand()
{
    WriteStringToGdbConsole("Supported monitor commands:\r\n");
    WriteStringToGdbConsole("detach\r\n");
    WriteStringToGdbConsole("reset [halt]\r\n");
    WriteStringToGdbConsole("showfault\r\n");
    WriteStringToGdbConsole("version\r\n");
    PrepareStringResponse("OK");
    return HANDLER_RETURN_HANDLED;
}


// Have any FLASH errors been encountered since the erase command was started.
static bool g_flashErrorDetected = false;

/* Handle the 'vFlashErase' command which erases the specified pages in FLASH.

    Command Format:     vFlashErase:AAAAAAAA,LLLLLLLL
    Response Format:    OK

    Where AAAAAAAA is the hexadecimal representation of the address where the erase is to start.
          LLLLLLLL is the hexadecimal representation of the length (in bytes) of the erase to be conducted.
*/
static uint32_t  handleFlashEraseCommand(Buffer* pBuffer)
{
    AddressLength        addressLength;

    __try
    {
        __throwing_func( ThrowIfNextCharIsNotEqualTo(pBuffer, ':') );
        __throwing_func( ReadAddressAndLengthArguments(pBuffer, &addressLength) );
    }
    __catch
    {
        logError("Failed parsing vFlashErase command.");
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
        return HANDLER_RETURN_HANDLED;
    }

    if (!g_cores.supportsFlashProgramming())
    {
        logError("mri-swd doesn't know how to FLASH program this device type.");
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
        return HANDLER_RETURN_HANDLED;
    }
    logInfoF("Erasing 0x%X (%u) bytes of FLASH at 0x%08X.", addressLength.length, addressLength.length, addressLength.address);
    if (!g_cores.flashBegin())
    {
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
        return HANDLER_RETURN_HANDLED;
    }

    // Return OK packet now before starting the erase so that GDB can start sending the next packet at the same time.
    sendOkResponseWithNewPacketBuffer();
    g_flashErrorDetected = false;

    if (!g_cores.flashErase(addressLength.address, addressLength.length))
    {
        // Set flag to return error to GDB on next FLASHing operation.
        g_flashErrorDetected = true;
    }

    // Let higher level MRI code know that we handled this command and already sent back the "OK" response.
    return HANDLER_RETURN_HANDLED | HANDLER_RETURN_RETURN_IMMEDIATELY;
}

static void sendOkResponseWithNewPacketBuffer()
{
    char   packetBuffer[10];
    Packet packet;

    Packet_Init(&packet, packetBuffer, sizeof(packetBuffer));
    Buffer_WriteString(&packet.dataBuffer, "OK");
    Buffer_SetEndOfBuffer(&packet.dataBuffer);
    Packet_SendToGDB(&packet);
}

/* Handle the 'vFlashWrite' command which writes to the specified location in FLASH.

    Command Format:     vFlashWrite:AAAAAAAA:XX...
    Response Format:    OK

    Where AAAAAAAA is the hexadecimal representation of the address where the write is to start.
          xx is the byte in escaped binary format of the first byte to be written to the specified location.
          ... continue returning the rest of the bytes in escaped binary format.
*/
static uint32_t  handleFlashWriteCommand(Buffer* pBuffer)
{
    // First check to see if the previous erase or write already failed.
    if (g_flashErrorDetected)
    {
        PrepareStringResponse(MRI_ERROR_MEMORY_ACCESS_FAILURE);
        return HANDLER_RETURN_HANDLED;
    }

    uint32_t  address = 0;
    __try
    {
        __throwing_func( ThrowIfNextCharIsNotEqualTo(pBuffer, ':') );
        __throwing_func( address = ReadUIntegerArgument(pBuffer) );
        __throwing_func( ThrowIfNextCharIsNotEqualTo(pBuffer, ':') );
    }
    __catch
    {
        logError("Failed parsing vFlashWrite command.");
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
        return HANDLER_RETURN_HANDLED;
    }
    assert ( g_cores.supportsFlashProgramming() );

    // Return OK packet now before starting the write so that GDB can start sending the next packet at the same time.
    sendOkResponseWithNewPacketBuffer();

    uint32_t length = Buffer_BytesLeft(pBuffer);
    logInfoF("Writing 0x%lX (%lu) bytes to FLASH at 0x%08lX.", length, length, address);
    if (!g_cores.flashProgram(address, pBuffer->pCurrent, length))
    {
        // Set flag to return error to GDB on next FLASHing operation.
        g_flashErrorDetected = true;
    }

    // Let higher level MRI code know that we handled this command and already sent back the "OK" response.
    return HANDLER_RETURN_HANDLED | HANDLER_RETURN_RETURN_IMMEDIATELY;
}


/* Handle the 'vFlashDone' command which lets us know that the FLASH update is now complete.

    Command Format:     vFlashDone
    Response Format:    OK
*/
static uint32_t handleFlashDoneCommand(Buffer* pBuffer)
{
    assert ( g_cores.supportsFlashProgramming() );

    logInfo("Completed FLASH update.");

    // First check to see if the previous write failed.
    if (g_flashErrorDetected)
    {
        PrepareStringResponse(MRI_ERROR_MEMORY_ACCESS_FAILURE);
        return HANDLER_RETURN_HANDLED;
    }

    if (!g_cores.flashEnd())
    {
        logError("Failed to end FLASHing operation.");
    }

    // Want to reset on next continue to place microcontroller in clean state for running new code now in FLASH.
    RequestResetOnNextContinue();
    PrepareStringResponse("OK");
    return HANDLER_RETURN_HANDLED;
}




// *********************************************************************************************************************
// Device specific code can call these routines to have desired code executed on the target device.
// *********************************************************************************************************************
bool startCodeOnDevice(const CortexM_Registers* pRegistersIn)
{
    return g_cores.startCodeOnDevice(pRegistersIn);
}

bool waitForCodeToHalt(CortexM_Registers* pRegistersOut, uint32_t timeout_ms)
{
    return g_cores.waitForCodeToHalt(pRegistersOut, timeout_ms);
}

bool runCodeOnDevice(CortexM_Registers* pRegistersInOut, uint32_t timeout_ms)
{
    return g_cores.runCodeOnDevice(pRegistersInOut, timeout_ms);
}




uintmri_t Platform_RtosGetHaltedThreadId(void)
{
    return 0;
}

uintmri_t Platform_RtosGetFirstThreadId(void)
{
    return 0;
}

uintmri_t Platform_RtosGetNextThreadId(void)
{
    return 0;
}

const char* Platform_RtosGetExtraThreadInfo(uintmri_t threadId)
{
    return NULL;
}

MriContext* Platform_RtosGetThreadContext(uintmri_t threadId)
{
    return NULL;
}

int Platform_RtosIsThreadActive(uintmri_t threadId)
{
    return 0;
}

int Platform_RtosIsSetThreadStateSupported(void)
{
    return 0;
}

void Platform_RtosSetThreadState(uintmri_t threadId, PlatformThreadState state)
{
}

void Platform_RtosRestorePrevThreadState(void)
{
}



void Platform_HandleFaultFromHighPriorityCode(void)
{
}



// *********************************************************************************************************************
// Routines called by the MRI core to determine if the stop was caused by semihost request.
// *********************************************************************************************************************
static int isInstructionMbedSemihostBreakpoint(uint16_t instruction);
static int isInstructionNewlibSemihostBreakpoint(uint16_t instruction);
static int isInstructionHardcodedBreakpoint(uint16_t instruction);
PlatformInstructionType Platform_TypeOfCurrentInstruction(void)
{
    uint16_t currentInstruction;

    __try
    {
        currentInstruction = getFirstHalfWordOfCurrentInstruction();
    }
    __catch
    {
        /* Will get here if PC isn't pointing to valid memory so treat as other. */
        clearExceptionCode();
        return MRI_PLATFORM_INSTRUCTION_OTHER;
    }

    bool isGdbTryingToBreakIn = g_gdbSocket.isGdbConnected() && !g_gdbSocket.m_tcpToMriQueue.isEmpty();
    if (isInstructionMbedSemihostBreakpoint(currentInstruction))
    {
        return isGdbTryingToBreakIn ? MRI_PLATFORM_INSTRUCTION_HARDCODED_BREAKPOINT :
                                      MRI_PLATFORM_INSTRUCTION_MBED_SEMIHOST_CALL;
    }
    else if (isInstructionNewlibSemihostBreakpoint(currentInstruction))
    {
        return isGdbTryingToBreakIn ? MRI_PLATFORM_INSTRUCTION_HARDCODED_BREAKPOINT :
                                      MRI_PLATFORM_INSTRUCTION_NEWLIB_SEMIHOST_CALL;
    }
    else if (isInstructionHardcodedBreakpoint(currentInstruction))
    {
        return MRI_PLATFORM_INSTRUCTION_HARDCODED_BREAKPOINT;
    }
    else
    {
        return MRI_PLATFORM_INSTRUCTION_OTHER;
    }
}

static int isInstructionMbedSemihostBreakpoint(uint16_t instruction)
{
    const uint16_t mbedSemihostBreakpointMachineCode = 0xbeab;

    return mbedSemihostBreakpointMachineCode == instruction;
}

static int isInstructionNewlibSemihostBreakpoint(uint16_t instruction)
{
    const uint16_t newlibSemihostBreakpointMinMachineCode = 0xbe00 | MRI_NEWLIB_SEMIHOST_MIN;
    const uint16_t newlibSemihostBreakpointMaxMachineCode = 0xbe00 | MRI_NEWLIB_SEMIHOST_MAX;

    return (instruction >= newlibSemihostBreakpointMinMachineCode &&
            instruction <=  newlibSemihostBreakpointMaxMachineCode);
}

static int isInstructionHardcodedBreakpoint(uint16_t instruction)
{
    const uint16_t hardCodedBreakpointMachineCode = 0xbe00;

    return (hardCodedBreakpointMachineCode == instruction);
}


PlatformSemihostParameters Platform_GetSemihostCallParameters(void)
{
    PlatformSemihostParameters parameters;

    parameters.parameter1 = g_cores.readRegisterOnHaltedCore(R0);
    parameters.parameter2 = g_cores.readRegisterOnHaltedCore(R1);
    parameters.parameter3 = g_cores.readRegisterOnHaltedCore(R2);
    parameters.parameter4 = g_cores.readRegisterOnHaltedCore(R3);

    return parameters;
}


void Platform_SetSemihostCallReturnAndErrnoValues(int returnValue, int errNo)
{
    g_cores.writeRegisterOnHaltedCore(R0, returnValue);
}



// *********************************************************************************************************************
// Semihost functionality for redirecting operations such as file I/O to the GNU debugger.
// *********************************************************************************************************************
static int writeToGdbConsole(const TransferParameters* pParameters);
int Semihost_IsDebuggeeMakingSemihostCall(void)
{
    PlatformInstructionType instructionType = Platform_TypeOfCurrentInstruction();

    return (instructionType == MRI_PLATFORM_INSTRUCTION_MBED_SEMIHOST_CALL ||
            instructionType == MRI_PLATFORM_INSTRUCTION_NEWLIB_SEMIHOST_CALL);
}


int Semihost_HandleSemihostRequest(void)
{
    PlatformInstructionType    instructionType = Platform_TypeOfCurrentInstruction();
    PlatformSemihostParameters parameters = Platform_GetSemihostCallParameters();

    if (instructionType == MRI_PLATFORM_INSTRUCTION_MBED_SEMIHOST_CALL)
        return Semihost_HandleMbedSemihostRequest(&parameters);
    else if (instructionType == MRI_PLATFORM_INSTRUCTION_NEWLIB_SEMIHOST_CALL)
        return Semihost_HandleNewlibSemihostRequest(&parameters);
    else
        return 0;
}


int Semihost_WriteToFileOrConsole(const TransferParameters* pParameters)
{
    const uint32_t STDOUT_FILE_NO = 1;
    if (pParameters->fileDescriptor == STDOUT_FILE_NO)
    {
        return writeToGdbConsole(pParameters);
    }
    return IssueGdbFileWriteRequest(pParameters);
}

static int writeToGdbConsole(const TransferParameters* pParameters)
{
    uint32_t address = pParameters->bufferAddress;
    uint32_t length = pParameters->bufferSize;
    char buffer[length];

    if (!g_cores.readMemory(address, buffer, length, SWD::TRANSFER_8BIT))
    {
        logErrorF("Failed to read %lu bytes of stdout output from 0x%08lX.", length, address);
        length = 0;
    }

    size_t charsWritten = WriteSizedStringToGdbConsole(buffer, length);
    SetSemihostReturnValues(charsWritten, 0);
    FlagSemihostCallAsHandled();

    if (WasControlCEncountered())
    {
        SetSignalValue(SIGINT);
        return 0;
    }
    return 1;
}



// *********************************************************************************************************************
// MBED semihosting API calls these routines to retrieve previosuly cached Ethernet UID on mbed-LPC1768 devices.
// *********************************************************************************************************************
const uint8_t* Platform_GetUid(void)
{
    return NULL;
}


uintmri_t Platform_GetUidSize(void)
{
    return 0;
}
