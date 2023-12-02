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
// CpuCore and CpuCores classes which expose debug operations on multi-core and single core Cortex-M devices.
#ifndef CPU_CORE_H_
#define CPU_CORE_H_

#include <stdlib.h>
#include "swd.h"
#include "mri_platform.h"
#include "devices/devices.h"

// MRI C headers
extern "C"
{
    #include <core/context.h>
    #include <core/core.h>
    #include <core/gdb_console.h>
    #include <core/platforms.h>
    #include <core/signal.h>
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



// Expose debug operations on a single Cortex-M core. The CpuCores object can contain multiple of these objects.
class CpuCore
{
public:
    CpuCore();

    // Initialize the CPU core object after establishing a new SWD connection.
    // This object will call the pHandler function if unrecoverable SWD connection errors are detected at runtime.
    // This handler is typically used to set a flag noting the connection error and force the MRI core code to exit.
    bool init(SWD* pSwdBus, void (*pHandler)(void), uint32_t coreId);

    // Initialize the CPU core object after establishing a new SWD connection based off of a template object.
    bool init(CpuCore* pTemplate, uint32_t coreId)
    {
        return init(pTemplate->m_pSwdBus, pTemplate->m_connectionFailureHandler, coreId);
    }

    // Cleanup the CPU core object after disconnecting the SWD bus. Call init() again later once the SWD
    // connection comes back up.
    void uninit();

    // Power down the SWD Debug Access Ports of all this CPU core. This reduces power usage on the target device.
    // Similar to uninit() but you aren't really expected to call init() again after calling this method since
    // that would mean the DAP would be powered up again.
    void disconnect();

    // Enables this core for debugging. This enables vector catch along with the Data Watchpoint and Flash Breakpoint
    // functionality.
    void initForDebugging();

    // Attempt to enable halting debug mode on this core.
    bool enableHaltDebugging(uint32_t timeout_ms);

    // Read and cache the most recent state of the CPU core.
    // Call isResetting() or isHalted() after to determine which of these state, if any, are
    // currently being experienced by the core.
    bool refreshCoreState(uint32_t timeout_ms)
    {
        uint32_t dhcsr = 0;
        bool result = readDHCSRWithRetry(&dhcsr, timeout_ms);

        // The m_currentDHCSR field should be set inside the above call to readDHCSRWithRetry().
        assert ( dhcsr == m_currentDHCSR );
        return result;
    }
    bool isResetting()
    {
        const uint32_t S_RESET_ST_Bit = 1 << 25;

        return !!(m_currentDHCSR & S_RESET_ST_Bit);
    }
    bool isHalted()
    {
        const uint32_t DHCSR_S_HALT_Bit = 1 << 17;
        return !!(m_currentDHCSR & DHCSR_S_HALT_Bit);
    }

    // Request the core to halt.
    bool requestHalt();

    // Track what the debugger knows about the FPU status of the cores so far.
    enum FpuDiscoveryStates
    {
        // Knows for sure that the device doesn't have a FPU.
        FPU_NONE = 0,
        // Cortex-M4 or higher MIGHT have a FPU. Will need to query CPACR on first halt to know for sure.
        FPU_MAYBE,
        // Knows for sure that the device has a FPU.
        FPU_AVAILABLE,
        // Later determined the device doesn't have a FPU but target XML indicates that it does.
        FPU_NOT_AVAILABLE
    };

    // Once default core is halted, this method can be called to update *pFpuState if it is still FPU_MAYBE.
    void checkForFpu(FpuDiscoveryStates* pFpuState);

    // Reads the core's registers to place them in MriContext structure which is then returned from this method.
    MriContext* getContext(FpuDiscoveryStates fpu)
    {
        readContext(fpu);
        return &m_context;
    }

    // Writes the register context back out to the core, if it was previously loaded with getContext(). Is a NOP if
    // getContext() hasn't been called since the last writeContext() call.
    void writeContext(FpuDiscoveryStates fpu);

    // Determine the reason this core halted: breakpoint, watchpoint, other.
    PlatformTrapReason determineTrapReason();

    // Ask the cores to halt immediately after coming out of reset. Should be called before reset() if this is the
    // desired behaviour.
    void enableResetVectorCatch();

    // Disable halting on device reset for this core.
    void disableResetVectorCatch();

    // Clears the Debug Fault Status Register on this core. Typically done after debugging a halted core.
    void clearDFSR();

    // Resumes execution of this core.
    bool resume();

    // Ask the core to reset.
    void reset();


    // Read from the requested memory range on this core. Can be used for reading memory mapped registers as those
    // reads often need to be directed at a specific core.
    uint32_t readMemory(uint32_t address, void* pvBuffer, uint32_t bufferSize, SWD::TransferSize readSize);

    // Write to the requested memory range on this core. Can be used for writing memory mapped registers as those
    // writes often need to be directed at a specific core.
    uint32_t writeMemory(uint32_t address, const void* pvBuffer, uint32_t bufferSize, SWD::TransferSize writeSize);


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
    // Can throw exceededHardwareResourcesException if all of the watchpoint comparators are already in use.
    // These exceptions will be caught and handled by the MRI core.
    __throws void setWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type);

    // Clear a watchpoint (read and/or write) at the specified memory range.
    // Can throw invalidArgumentException if the address, size, or type aren't supported for Cortex-M devices.
    // These exceptions will be caught and handled by the MRI core.
    __throws void clearWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type);


    // Returns the signal value (SIGINT, SIGTRAP, etc) most appropriate for the reason that this core was halted. This
    // is returned to GDB and displayed to the user.
    //  wasStopFromGDB parameter indicates whether the target was interrupted by GDB.
    uint8_t determineCauseOfException(bool wasStopFromGDB);

    // If an ARMv7M microcontroller hits a fault while running under the debugger, this method can be called to display
    // information about the type of fault encountered to the GDB console before halting.
    void displayFaultCauseToGdbConsole();


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


    // Fetch the underlying SwdTarget object used for communicating with this core.
    SwdTarget* getTarget()
    {
        return &m_swd;
    }




protected:
    bool initSWD(SWD* pSwdBus);

    void enableDWTandVectorCatches();
    bool setOrClearBitsInDEMCR(uint32_t bitMask, bool set);
    void handleUnrecoverableSwdError();
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

    bool readDHCSRWithRetry(uint32_t* pValue, uint32_t timeout_ms);
    bool readDHCSR(uint32_t* pValue);
    bool writeDHCSR(uint32_t DHCSR_Value);

    void readContext(FpuDiscoveryStates fpu);
    bool readCpuRegister(uint32_t registerIndex, uint32_t* pValue);
    void waitForRegisterTransferToComplete();
    static bool hasRegisterTransferCompleted(uint32_t DHCSR_Value);

    bool writeCpuRegister(uint32_t registerIndex, uint32_t value);

    bool readDFSR(uint32_t* pDFSR);
    PlatformTrapReason findMatchedWatchpoint(void);
    PlatformTrapReason getReasonFromMatchComparator(uint32_t comparatorAddress, uint32_t function);

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



    // Pointer to lower level SWD bus object used to communicate with all target/cores on the device.
    SWD*           m_pSwdBus = NULL;

    // Callback to call when SWD connection errors are detected.
    void (*m_connectionFailureHandler)(void) = NULL;

    // CPU register context information is stored here.
    uintmri_t      m_contextRegisters[registerCountFPU];
    ContextSection m_contextEntries = { .pValues = &m_contextRegisters[0], .count = 0 };
    MriContext     m_context;

    // Higher level SWD target object which is configured to communicate with this core.
    SwdTarget      m_swd;

    // Most recent value of the Debug Halting Control & Status Register value read out by a call to the
    // readDHCSR() method.
    uint32_t       m_currentDHCSR = 0;

    // The index of CPU core to display when logging messages from this core.
    uint32_t        m_coreId = -1;

    // Set to true if any read/writeMemory() call fails because of a SWD protocol error which means that the
    // target has stopped responding.
    bool           m_hasDetectedSwdDisconnect = false;
};



// Expose debug operations on Cortex-M cores. It can handle targets which have more than one core such as the RP2040.
class CpuCores
{
public:
    CpuCores();


    // Initialize the CPU core objects after establishing a new SWD connection.
    // The pHandler callback function is stored and will be called later if any unrecoverable SWD connection errors are
    // detected at runtime. Handler is typically used to force the MRI core code to exit.
    bool init(SWD* pSwdBus, void (*pHandler)(void));

    // Cleanup the CPU core objects after disconnecting the SWD bus. Call init() again later once the SWD
    // connection comes back up.
    void uninit();

    // Power down the SWD Debug Access Ports of all the CPU cores. This reduces power usage on the target
    // device. Similar to uninit() but you aren't really expected to call init() again after calling this method since
    // that would mean the DAP would be powered up again.
    void disconnect();

    // Enables the CPU cores for debugging. This enables vector catch along with the Data Watchpoint
    // and Flash Breakpoint functionality.
    void initForDebugging();

    // Attempt to enable halting debug mode on the CPU cores.
    bool enableHaltDebugging(uint32_t timeout_ms);

    // Read the current state of each core on the device and store it away for subsequent indexOf*Core() calls.
    bool refreshCoreStates(uint32_t timeout_ms);

    // Special core index that indicates that no core matched requested state.
    static const int CORE_NONE = -1;

    // Returns the lowest index of a resetting core, CORE_NONE if none are resetting.
    int indexOfResettingCore();

    // Returns the loweset index of a halted core, CORE_NONE if none are halting.
    int indexOfHaltedCore();

    // Has the halted core been externally reset while halted in debugger?
    bool isHaltedCoreResetting()
    {
        assert ( m_pHaltedCore != NULL );
        if (!m_pHaltedCore->refreshCoreState(0))
        {
            return false;
        }
        return m_pHaltedCore->isResetting();
    }

    // Request the CPU cores to halt, skipping the alreadyHaltedCore.
    // Set alreadyHaltedCore to CORE_NONE to halt ALL cores.
    bool requestCoresToHalt(int alreadyHaltedCore);


    // Enter MRI by calling mriDebugException() after grabbing the required register context of the specified halted
    // core.
    void enterMriCore(int haltedCore);

    // Typically called from Platform_EnteringDebugger() to let this object know that the MRI core was just entered.
    // It caches the reason (interrupt from GDB, breakpoint, watchpoint, etc) for the halting core and makes sure that
    // reset vector catch is disabled for all of the cores.
    void enteringDebugger();

    // Typically called from Platform_LeavingDebugger() to let this object know that the MRI core is about to exit and
    // allow execution to be resumed. It makes sure that the Debug Fault Status Register (DFSR) is cleared since we no
    // longer need to know about the cause of the last halt.
    void leavingDebugger();

    // Ask execution to resume on all of the cores. It also takes care of restoring any modified register context before
    // resuming.
    void resume();

    // Ask the cores to halt immediately after coming out of reset. Should be called before reset() if this is the
    // desired behaviour.
    void enableResetVectorCatch();

    // Ask the cores to reset.
    void reset();

    // Should be called as soon as the main loop determines that a reset has occurred so that the object can
    // re-initialize the information it has about all of the cores after their reset.
    void resetCompleted()
    {
        assert ( m_connectionFailureHandler != NULL );
        checkForMultipleCores(m_connectionFailureHandler);
    }


    // Read the requested register on the halted core.
    uintmri_t readRegisterOnHaltedCore(size_t index)
    {
        assert ( m_pHaltedCore != NULL );
        return Context_Get(m_pHaltedCore->getContext(m_fpu), index);
    }

    // Write a new value to the requested register on the halted core.
    void writeRegisterOnHaltedCore(size_t index, uintmri_t newValue)
    {
        assert ( m_pHaltedCore != NULL );
        Context_Set(m_pHaltedCore->getContext(m_fpu), index, newValue);
    }

    // Read from the requested memory range in FLASH, ROM, or RAM. Not to be used for reading memory mapped registers as
    // those reads need to be directed at a specific core.
    uint32_t readMemory(uint32_t address, void* pvBuffer, uint32_t bufferSize, SWD::TransferSize readSize)
    {
        // Make sure that we don't try to read registers with this method.
        assert ( address < 0xA0000000 );
        assert ( m_pDefaultCore != NULL );
        return m_pDefaultCore->readMemory(address, pvBuffer, bufferSize, readSize);
    }

    // Write to the requested memory range in FLASH, ROM, or RAM. Not to be used for writing memory mapped registers as
    // those writes need to be directed at a specific core.
    uint32_t writeMemory(uint32_t address, const void* pvBuffer, uint32_t bufferSize, SWD::TransferSize writeSize)
    {
        // Make sure that we don't try to write registers with this method.
        assert ( address < 0xA0000000 );
        assert ( m_pDefaultCore != NULL );
        return m_pDefaultCore->writeMemory(address, pvBuffer, bufferSize, writeSize);
    }


    // UNDONE: Might just ignore this for multithread and use thread state instead.
    // Enable single stepping.
    void enableSingleStep()
    {
        assert ( m_pDefaultCore != NULL );
        m_pDefaultCore->enableSingleStep();
    }

    // Disable single stepping.
    void disableSingleStep()
    {
        assert ( m_pDefaultCore != NULL );
        m_pDefaultCore->disableSingleStep();
    }

    // Is single stepping enabled?
    bool isSingleStepping()
    {
        assert ( m_pDefaultCore != NULL );
        return m_pDefaultCore->isSingleStepping();
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
    uint8_t determineCauseOfException(bool wasStopFromGDB)
    {
        assert ( m_pHaltedCore != NULL );
        return m_pHaltedCore->determineCauseOfException(wasStopFromGDB);
    }

    // Returns whether the CPU was halted because of a breakpoint, watchpoint, or some other reason. This is returned to
    // GDB and used to display which breakpoint or watchpoint, if any, caused the halt to occur. This returned reason
    // was actually determined earlier when enteringDebugger() was called because watchpoint hit state can only be
    // queried once.
    PlatformTrapReason getTrapReason()
    {
        return m_trapReason;
    }

    // If an ARMv7M microcontroller hits a fault while running under the debugger, this method can be called to display
    // information about the type of fault encountered to the GDB console before halting.
    void displayFaultCauseToGdbConsole()
    {
        assert ( m_pHaltedCore != NULL );
        return m_pHaltedCore->displayFaultCauseToGdbConsole();
    }


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
    bool startCodeOnDevice(const CortexM_Registers* pRegistersIn)
    {
        assert ( m_pDefaultCore != NULL );
        // Make sure that original register context has been saved away for the default core before modifying its
        // registers to start new code on it.
        m_pDefaultCore->getContext(m_fpu);
        return m_pDefaultCore->startCodeOnDevice(pRegistersIn);
    }

    // Waits for code previously started by startCodeOnDevice() to complete. The code should signal its completion by
    // executing a hardcoded BKPT instruction. The values of the registers at the time of the BKPT can be found in the
    // pRegistersOut structure.
    bool waitForCodeToHalt(CortexM_Registers* pRegistersOut, uint32_t timeout_ms)
    {
        assert ( m_pDefaultCore != NULL );
        return m_pDefaultCore->waitForCodeToHalt(pRegistersOut, timeout_ms);
    }

    // This combines calls to startCodeOnDevice() and waitForCodeToWait() in one call.
    bool runCodeOnDevice(CortexM_Registers* pRegistersInOut, uint32_t timeout_ms)
    {
        assert ( m_pDefaultCore != NULL );
        return m_pDefaultCore->runCodeOnDevice(pRegistersInOut, timeout_ms);
    }


    // Does the device have a FPU?
    CpuCore::FpuDiscoveryStates hasFPU()
    {
        return m_fpu;
    }

    // Retrieve a string pointer and the length for the XML describing the memory layout of this device.
    const char* getMemoryLayoutXML()
    {
        return m_pMemoryLayoutXML;
    }
    size_t getMemoryLayoutXMLSize()
    {
        return m_memoryLayoutSize;
    }

protected:
    void searchSupportedDevicesList();
    void determineFpuAvailability();
    void updateSwdFrequencyForDevice();
    void constructMemoryLayoutXML();
    void checkForMultipleCores(void (*pHandler)(void));
    void cleanupDeviceObject();
    void disableResetVectorCatch();



    // Callback to call when SWD connection errors are detected.
    void (*m_connectionFailureHandler)(void);
    // Pointer to lower level SWD bus object used to communicate with all target/cores on the device.
    SWD*       m_pSwdBus = NULL;
    // Pointer to the core which cause the current halt situation.
    CpuCore*   m_pHaltedCore = NULL;
    // Pointers to the default core (usually core 0) and its SWD target used for things like reading/writing/programming
    // RAM/FLASH.
    CpuCore*   m_pDefaultCore = NULL;

    // When mri-swd has support (including FLASH programming) for the attached device, then these globals will store
    // pointers to its function table and object data.
    DeviceFunctionTable* m_pDevice = NULL;
    DeviceObject*        m_pDeviceObject = NULL;

    // Used as storage for the device memory layout XML.
    char*   m_pMemoryLayoutXML = NULL;
    int32_t m_memoryLayoutAllocSize = 0;
    int32_t m_memoryLayoutSize = 0;

    // Cache the reason if breakpoint or watchpoint caused core to halt from the enteringDebugger() method. It is cached
    // because the bit read to determine which watchpoint was hit, if any, is cleared on read.
    PlatformTrapReason m_trapReason;

    // Does this CPU contain a FPU or not?
    CpuCore::FpuDiscoveryStates m_fpu = CpuCore::FPU_NONE;

    // The actual list of CpuCore objects and their related SWD targets.
    CpuCore    m_cores[MAX_DPV2_TARGETS];
    // The number of elements used in the above arrays for the currently attached CPU.
    size_t     m_coreCount = 0;

    // Has a SWD protocol error been detected that should cause a disconnect to be triggered.
    bool       m_hasDetectedSwdDisconnect = false;
};

#endif // CPU_CORE_H_
