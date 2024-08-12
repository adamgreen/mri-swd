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
// CpuCore and CpuCores classes which expose debug operations on multi-core and single core Cortex-M devices.
#define CPU_CORE_MODULE "cpu_core.cpp"
#include "logging.h"
#include "cpu_core.h"


CpuCore::CpuCore()
{
}

bool CpuCore::init(SWD* pSwdBus, void (*pHandler)(void), uint32_t coreId)
{
    m_coreId = coreId;
    m_ignoreReadErrors = false;
    m_prevState = MRI_PLATFORM_THREAD_THAWED;
    m_currState = MRI_PLATFORM_THREAD_THAWED;

    assert ( m_pSwdBus == NULL && m_contextEntries.count == 0 && m_currentDHCSR == 0 );
    if (!initSWD(pSwdBus))
    {
        return false;
    }
    m_connectionFailureHandler = pHandler;
    m_hasDetectedSwdDisconnect = false;

    initForDebugging();
    enableHaltDebugging(READ_DHCSR_TIMEOUT_MS);

    return true;
}

bool CpuCore::initSWD(SWD* pSwdBus)
{
    logInfo("Initializing target's debug components...");
    bool result = pSwdBus->initTargetForDebugging(m_swd);
    if (!result)
    {
        logError("Failed to initialize target's debug components.");
        return false;
    }
    m_pSwdBus = pSwdBus;
    return true;
}

void CpuCore::initForDebugging()
{
    enableDWTandVectorCatches();
    initDWT();
    initFPB();
}

void CpuCore::enableDWTandVectorCatches()
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
        logErrorF("Core%lu: Failed to set DWTENA/TRCENA and vector catch bits in DEMCR register.", m_coreId);
        return;
    }
}

bool CpuCore::setOrClearBitsInDEMCR(uint32_t bitMask, bool set)
{
    // Debug Exception and Monitor Control Register, DEMCR
    const uint32_t DEMCR_Address = 0xE000EDFC;

    uint32_t DEMCR_Value = 0;
    if (readMemory(DEMCR_Address, &DEMCR_Value, sizeof(DEMCR_Value), SWD::TRANSFER_32BIT) != sizeof(DEMCR_Value))
    {
        logErrorF("Core%lu: Failed to read DEMCR register.", m_coreId);
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

    if (writeMemory(DEMCR_Address, &DEMCR_Value, sizeof(DEMCR_Value), SWD::TRANSFER_32BIT) != sizeof(DEMCR_Value))
    {
        logErrorF("Core%lu: Failed to write DEMCR register.", m_coreId);
        return false;
    }
    return true;
}

uint32_t CpuCore::readMemory(uint32_t address, void* pvBuffer, uint32_t bufferSize, SWD::TransferSize readSize)
{
    if (m_hasDetectedSwdDisconnect || bufferSize == 0)
    {
        return 0;
    }

    uint32_t bytesRead = m_swd.readMemory(address, pvBuffer, bufferSize, readSize);
    if (bytesRead == 0 && m_swd.getLastReadWriteError() == SWD::SWD_PROTOCOL && !m_ignoreReadErrors)
    {
        handleUnrecoverableSwdError();
    }
    return bytesRead;
}

void CpuCore::handleUnrecoverableSwdError()
{
    logErrorF("Encountered unrecoverable read/write error %d.", m_swd.getLastReadWriteError());
    m_hasDetectedSwdDisconnect = true;
    if (m_connectionFailureHandler)
    {
        m_connectionFailureHandler();
    }
}

uint32_t CpuCore::writeMemory(uint32_t address, const void* pvBuffer, uint32_t bufferSize, SWD::TransferSize writeSize)
{
    if (m_hasDetectedSwdDisconnect || bufferSize == 0)
    {
        return 0;
    }

    uint32_t bytesWritten = m_swd.writeMemory(address, pvBuffer, bufferSize, writeSize);
    if (bytesWritten == 0 && m_swd.getLastReadWriteError() == SWD::SWD_PROTOCOL)
    {
        handleUnrecoverableSwdError();
    }
    return bytesWritten;
}

void CpuCore::initDWT()
{
    uint32_t watchpointCount = clearDWTComparators();
    logInfoF("Core%lu supports %lu hardware watchpoints.", m_coreId, watchpointCount);
}

uint32_t CpuCore::clearDWTComparators()
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

uint32_t CpuCore::getDWTComparatorCount()
{
    uint32_t DWT_CTRL_Address = 0xE0001000;
    uint32_t DWT_CTRL_Value = 0;

    if (readMemory(DWT_CTRL_Address, &DWT_CTRL_Value, sizeof(DWT_CTRL_Value), SWD::TRANSFER_32BIT) != sizeof(DWT_CTRL_Value))
    {
        logErrorF("Core%lu: Failed to read DWT_CTRL register.", m_coreId);
        return 0;
    }
    return (DWT_CTRL_Value >> 28) & 0xF;
}

void CpuCore::clearDWTComparator(uint32_t comparatorAddress)
{
    //  Matched.  Read-only.  Set to 1 to indicate that this comparator has been matched.  Cleared on read.
    const uint32_t DWT_COMP_FUNCTION_DATAVMATCH_Bit = 1 << 8;
    //  Cycle Count Match.  Set to 1 for enabling cycle count match and 0 otherwise.  Only valid on comparator 0.
    const uint32_t DWT_COMP_FUNCTION_CYCMATCH_Bit = 1 << 7;
    //  Enable Data Trace Address offset packets.  0 to disable.
    const uint32_t DWT_COMP_FUNCTION_EMITRANGE_Bit = 1 << 5;
    DWT_COMP_Type dwtComp;

    if (readMemory(comparatorAddress, &dwtComp, 3*sizeof(uint32_t), SWD::TRANSFER_32BIT) != 3*sizeof(uint32_t))
    {
        logErrorF("Core%lu: Failed to read DWT_COMP/DWT_MASK/DWT_FUNCTION registers for clearing.", m_coreId);
    }

    dwtComp.comp = 0;
    dwtComp.mask = 0;
    dwtComp.function &= ~(DWT_COMP_FUNCTION_DATAVMATCH_Bit |
                          DWT_COMP_FUNCTION_CYCMATCH_Bit |
                          DWT_COMP_FUNCTION_EMITRANGE_Bit |
                          DWT_COMP_FUNCTION_FUNCTION_Mask);

    if (writeMemory(comparatorAddress, &dwtComp, 3*sizeof(uint32_t), SWD::TRANSFER_32BIT) != 3*sizeof(uint32_t))
    {
        logErrorF("Core%lu: Failed to write DWT_COMP/DWT_MASK/DWT_FUNCTION registers for clearing.", m_coreId);
    }
}

void CpuCore::initFPB()
{
    uint32_t breakpointCount = clearFPBComparators();
    logInfoF("Core%lu supports %lu hardware breakpoints.", m_coreId, breakpointCount);
    enableFPB();
}

uint32_t CpuCore::clearFPBComparators()
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

uint32_t CpuCore::getFPBCodeComparatorCount()
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

uint32_t CpuCore::readFPControlRegister()
{
    uint32_t FP_CTRL_Value = 0;
    if (readMemory(FP_CTRL_Address, &FP_CTRL_Value, sizeof(FP_CTRL_Value), SWD::TRANSFER_32BIT) != sizeof(FP_CTRL_Value))
    {
        logErrorF("Core%lu: Failed to read FP_CTRL register.", m_coreId);
        return 0;
    }
    return FP_CTRL_Value;
}

uint32_t CpuCore::getFPBLiteralComparatorCount()
{
    //  Number of instruction literal address comparators.  Read only
    const uint32_t FP_CTRL_NUM_LIT_Shift = 8;
    const uint32_t FP_CTRL_NUM_LIT_Mask = 0xF << FP_CTRL_NUM_LIT_Shift;
    uint32_t FP_CTRL_Value = readFPControlRegister();

    return ((FP_CTRL_Value & FP_CTRL_NUM_LIT_Mask) >> FP_CTRL_NUM_LIT_Shift);
}

void CpuCore::clearFPBComparator(uint32_t comparatorAddress)
{
    uint32_t comparatorValue = 0;
    if (writeMemory(comparatorAddress, &comparatorValue, sizeof(comparatorValue), SWD::TRANSFER_32BIT) != sizeof(comparatorValue))
    {
        logErrorF("Core%lu: Failed to write to FP comparator register.", m_coreId);
    }
}

void CpuCore::enableFPB()
{
    //  This Key field must be set to 1 when writing or the write will be ignored.
    const uint32_t FP_CTRL_KEY = 1 << 1;
    //  Enable bit for the FPB.  Set to 1 to enable FPB.
    const uint32_t FP_CTRL_ENABLE = 1;

    uint32_t FP_CTRL_Value = readFPControlRegister();
    FP_CTRL_Value |= (FP_CTRL_KEY | FP_CTRL_ENABLE);
    writeFPControlRegister(FP_CTRL_Value);
}

void CpuCore::writeFPControlRegister(uint32_t FP_CTRL_Value)
{
    if (writeMemory(FP_CTRL_Address, &FP_CTRL_Value, sizeof(FP_CTRL_Value), SWD::TRANSFER_32BIT) != sizeof(FP_CTRL_Value))
    {
        logErrorF("Core%lu: Failed to write FP_CTRL register.", m_coreId);
    }
}

bool CpuCore::enableHaltDebugging(uint32_t timeout_ms)
{
    uint32_t DHCSR_Val = 0;
    if (!readDHCSRWithRetry(&DHCSR_Val, timeout_ms))
    {
        logErrorF("Core%lu: Failed to read DHCSR to enable debugging.", m_coreId);
        return false;
    }

    // The values of the current DHCSR_C_* bits will be overwritten with just what is needed to enable SWD debugging
    // and maintain the target's current halted state.
    if (isHalted())
    {
        // If the CPU is currently halted then we don't want to set DHCSR with DHCSR_C_HALT_Bit cleared as that will
        // start the target running again.
        logInfoF("Core%lu was already halted at debugger init.", m_coreId);
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
        logErrorF("Core%lu: Failed to set C_DEBUGEN bit in DHCSR.", m_coreId);
        return false;
    }

    return true;
}

bool CpuCore::readDHCSRWithRetry(uint32_t* pValue, uint32_t timeout_ms)
{
    bool returnVal = false;
    absolute_time_t endTime = make_timeout_time_ms(timeout_ms);
    m_pSwdBus->disableErrorLogging();
    if (timeout_ms > 0)
    {
        logErrorDisable();
    }

    m_ignoreReadErrors = true;
    do
    {
        if (readDHCSR(pValue))
        {
            returnVal = true;
            break;
        }
    } while (absolute_time_diff_us(get_absolute_time(), endTime) > 0);
    m_ignoreReadErrors = false;

    if (timeout_ms > 0)
    {
        logErrorEnable();
    }
    m_pSwdBus->enableErrorLogging();

    if (!returnVal)
    {
        // The read errors were ignored in readMemory() so handle the SWD error here if we time out on attempting to
        // read the DHCSR.
        handleUnrecoverableSwdError();
    }

    return returnVal;
}

bool CpuCore::readDHCSR(uint32_t* pValue)
{
    if (readMemory(DHCSR_Address, pValue, sizeof(*pValue), SWD::TRANSFER_32BIT) != sizeof(*pValue))
    {
        logErrorF("Core%lu: Failed to read DHCSR register.", m_coreId);
        return false;
    }
    m_currentDHCSR = *pValue;
    return true;
}

bool CpuCore::writeDHCSR(uint32_t DHCSR_Value)
{
    // Upper 16-bits must contain DBGKEY for CPU to accept this write.
    const uint32_t DHCSR_DBGKEY_Shift = 16;
    const uint32_t DHCSR_DBGKEY_Mask = 0xFFFF << DHCSR_DBGKEY_Shift;
    const uint32_t DHCSR_DBGKEY = 0xA05F << DHCSR_DBGKEY_Shift;
    DHCSR_Value = (DHCSR_Value & ~DHCSR_DBGKEY_Mask) | DHCSR_DBGKEY;

    return writeMemory(DHCSR_Address, &DHCSR_Value, sizeof(DHCSR_Value), SWD::TRANSFER_32BIT) == sizeof(DHCSR_Value);
}

void CpuCore::uninit()
{
    m_swd.uninit();
    m_pSwdBus = NULL;
    m_contextEntries.count = 0;
    m_currentDHCSR = 0;
}

void CpuCore::disconnect()
{
    m_swd.disconnect();
    m_swd.uninit();
    m_pSwdBus = NULL;
    m_contextEntries.count = 0;
    m_currentDHCSR = 0;
}

bool CpuCore::requestHalt()
{
    // Shouldn't have valid context around when core was running.
    assert ( m_contextEntries.count == 0 );

    uint32_t DHCSR_Val = 0;
    readDHCSR(&DHCSR_Val);
    // Ignore read errors.

    DHCSR_Val |= DHCSR_C_HALT_Bit;

    if (!writeDHCSR(DHCSR_Val))
    {
        logErrorF("Core%lu: Failed to set C_HALT bit in DHCSR.", m_coreId);
        return false;
    }
    return true;
}


void CpuCore::checkForFpu(FpuDiscoveryStates* pFpuState)
{
    if (*pFpuState != FPU_MAYBE)
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
    if (readMemory(CPACR_Address, &CPACR_OrigValue, sizeof(CPACR_OrigValue), SWD::TRANSFER_32BIT) != sizeof(CPACR_OrigValue))
    {
        logErrorF("Core%lu: Failed to read CPACR register.", m_coreId);
        // On error, assume that FPU doesn't exist.
        *pFpuState = FPU_NOT_AVAILABLE;
        return;
    }
    if ((CPACR_OrigValue & CPACR_CP10_Mask) && (CPACR_OrigValue & CPACR_CP11_Mask))
    {
        // FPU exists and is enabled.
        logInfoF("Core%lu has FPU.", m_coreId);
        *pFpuState = FPU_AVAILABLE;
        return;
    }

    // Try enabling the FPU.
    uint32_t CPACR_TestValue = CPACR_CP10_Mask | CPACR_CP11_Mask | CPACR_OrigValue;
    if (writeMemory(CPACR_Address, &CPACR_TestValue, sizeof(CPACR_TestValue), SWD::TRANSFER_32BIT) != sizeof(CPACR_TestValue))
    {
        logErrorF("Core%lu: Failed to test CPACR register.", m_coreId);
        // On error, assume that FPU doesn't exist.
        *pFpuState = FPU_NOT_AVAILABLE;
        return;
    }

    // See if the FPU enable took. If so then the FPU exists.
    uint32_t CPACR_UpdatedValue = ~CPACR_TestValue;
    if (readMemory(CPACR_Address, &CPACR_UpdatedValue, sizeof(CPACR_UpdatedValue), SWD::TRANSFER_32BIT) != sizeof(CPACR_UpdatedValue))
    {
        logErrorF("Core%lu: Failed to verify CPACR register.", m_coreId);
        // On error, assume that FPU doesn't exist.
        *pFpuState = FPU_NOT_AVAILABLE;
    }
    if (CPACR_UpdatedValue == CPACR_TestValue)
    {
        logInfoF("Core%lu has FPU.", m_coreId);
        *pFpuState = FPU_AVAILABLE;
    }

    // Restore CPACR Value.
    if (writeMemory(CPACR_Address, &CPACR_OrigValue, sizeof(CPACR_OrigValue), SWD::TRANSFER_32BIT) != sizeof(CPACR_OrigValue))
    {
        logErrorF("Core%lu: Failed to restore CPACR register.", m_coreId);
    }
}

void CpuCore::readContext(FpuDiscoveryStates fpu)
{
    if (m_contextEntries.count != 0)
    {
        // Have already called readContext since the last write.
        return;
    }

    bool encounteredError = false;
    bool contextHasFpu = fpu >= FPU_MAYBE;
    if (contextHasFpu)
    {
        m_contextEntries.count = registerCountFPU;
    }
    else
    {
        m_contextEntries.count = registerCountNoFPU;
    }
    Context_Init(&m_context, &m_contextEntries, 1);

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
            if (fpu == FPU_AVAILABLE)
            {
                if (!readCpuRegister(dcrsrS0Index+i, &regValue))
                {
                    encounteredError = true;
                }
            }
            Context_Set(&m_context, S0+i, regValue);
        }

        // Transfer FPSCR register.
        if (fpu == FPU_AVAILABLE)
        {
            uint32_t regValue = 0xBAADFEED;
            if (fpu == FPU_AVAILABLE)
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
        logErrorF("Core%lu: Failed to read CPU register(s).", m_coreId);
    }
}

bool CpuCore::readCpuRegister(uint32_t registerIndex, uint32_t* pValue)
{
    uint32_t DCRSR_Value = registerIndex & DCRSR_REGSEL_Mask;
    if (writeMemory(DCRSR_Address, &DCRSR_Value, sizeof(DCRSR_Value), SWD::TRANSFER_32BIT) != sizeof(DCRSR_Value))
    {
        return false;
    }

    waitForRegisterTransferToComplete();

    return readMemory(DCRDR_Address, pValue, sizeof(*pValue), SWD::TRANSFER_32BIT) == sizeof(*pValue);
}

void CpuCore::waitForRegisterTransferToComplete()
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

bool CpuCore::hasRegisterTransferCompleted(uint32_t DHCSR_Value)
{
    const uint32_t DHCSR_S_REGRDY_Bit = 1 << 16;
    return (DHCSR_Value & DHCSR_S_REGRDY_Bit) == DHCSR_S_REGRDY_Bit;
}

void CpuCore::writeContext(FpuDiscoveryStates fpu)
{
    if (m_contextEntries.count == 0)
    {
        // readContext() hasn't been called since last writeContext() so just return.
        return;
    }

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
    if (fpu == FPU_AVAILABLE)
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
        logErrorF("Core%lu: Failed to write CPU register(s).", m_coreId);
    }

    // This context is no longer valid so mark it as such.
    m_contextEntries.count = 0;
}

bool CpuCore::writeCpuRegister(uint32_t registerIndex, uint32_t value)
{
    if (writeMemory(DCRDR_Address, &value, sizeof(value), SWD::TRANSFER_32BIT) != sizeof(value))
    {
        return false;
    }

    uint32_t DCRSR_Value = DCRSR_REGWnR_Bit | (registerIndex & DCRSR_REGSEL_Mask);
    if (writeMemory(DCRSR_Address, &DCRSR_Value, sizeof(DCRSR_Value), SWD::TRANSFER_32BIT) != sizeof(DCRSR_Value))
    {
        return false;
    }

    waitForRegisterTransferToComplete();

    return true;
}

PlatformTrapReason CpuCore::determineTrapReason()
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

bool CpuCore::readDFSR(uint32_t* pDFSR)
{
    if (readMemory(DFSR_Address, pDFSR, sizeof(*pDFSR), SWD::TRANSFER_32BIT) != sizeof(*pDFSR))
    {
        logErrorF("Core%lu: Failed to read DFSR.", m_coreId);
        return false;
    }
    return true;
}

PlatformTrapReason CpuCore::findMatchedWatchpoint(void)
{
    PlatformTrapReason reason = { MRI_PLATFORM_TRAP_TYPE_UNKNOWN, 0x00000000 };
    uint32_t currentComparatorAddress = DWT_COMP_ARRAY_Address;
    uint32_t comparatorCount = getDWTComparatorCount();
    for (uint32_t i = 0 ; i < comparatorCount ; i++)
    {
        uint32_t function = 0;
        if (readMemory(currentComparatorAddress + offsetof(DWT_COMP_Type, function), &function, sizeof(function), SWD::TRANSFER_32BIT) != sizeof(function))
        {
            logErrorF("Core%lu: Failed to read DWT function register.", m_coreId);
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

PlatformTrapReason CpuCore::getReasonFromMatchComparator(uint32_t comparatorAddress, uint32_t function)
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
    if (readMemory(comparatorAddress + offsetof(DWT_COMP_Type, comp), &compValue, sizeof(compValue), SWD::TRANSFER_32BIT) != sizeof(compValue))
    {
        logErrorF("Core%lu: Failed to read DWT comp register.", m_coreId);
        return reason;
    }
    reason.address = compValue;

    return reason;
}

void CpuCore::enableResetVectorCatch()
{
    if (!setOrClearBitsInDEMCR(DEMCR_VC_CORERESET_Bit, true))
    {
        logErrorF("Core%lu: Failed to set reset vector catch bit in DEMCR register.", m_coreId);
        return;
    }
}

void CpuCore::disableResetVectorCatch()
{
    if (!setOrClearBitsInDEMCR(DEMCR_VC_CORERESET_Bit, false))
    {
        logErrorF("Core%lu: Failed to clear reset vector catch bits in DEMCR register.", m_coreId);
        return;
    }
}

bool CpuCore::hasDebugEvent(bool ignoreHaltedEvent)
{
    uint32_t dfsr = 0;
    if (!readDFSR(&dfsr))
    {
        logErrorF("Core%lu: Failed to read DFSR for zero check.", m_coreId);
        return false;
    }
    if (ignoreHaltedEvent)
    {
        dfsr &= ~DFSR_HALTED_Bit;
    }

    return dfsr != 0;
}

void CpuCore::clearDebugEvent()
{
    uint32_t dfsr = 0;
    if (!readDFSR(&dfsr))
    {
        logErrorF("Core%lu: Failed to read DFSR to clear it.", m_coreId);
        return;
    }
    if (!writeDFSR(dfsr))
    {
        logErrorF("Core%lu: Failed to write DFSR to clear it.", m_coreId);
        return;
    }
}

bool CpuCore::writeDFSR(uint32_t dfsr)
{
    if (writeMemory(DFSR_Address, &dfsr, sizeof(dfsr), SWD::TRANSFER_32BIT) != sizeof(dfsr))
    {
        logErrorF("Core%lu: Failed to write to DFSR.", m_coreId);
        return false;
    }
    return true;
}

bool CpuCore::resume()
{
    m_prevState = m_currState;
    if (m_currState == MRI_PLATFORM_THREAD_FROZEN)
    {
        // This core/thread has been requested to be frozen by GDB so just return without resuming it.
        return true;
    }
    if (m_currState == MRI_PLATFORM_THREAD_SINGLE_STEPPING)
    {
        enableSingleStep();
    }
    else
    {
        disableSingleStep();
    }

    return clearHaltBit();
}

bool CpuCore::clearHaltBit()
{
    uint32_t DHCSR_Val = 0;
    readDHCSR(&DHCSR_Val);
    DHCSR_Val &= ~DHCSR_C_HALT_Bit;
    if (!writeDHCSR(DHCSR_Val))
    {
        logErrorF("Core%lu: Failed to clear C_HALT bit in DHCSR.", m_coreId);
        return false;
    }
    return true;
}

void CpuCore::reset()
{
    // Mark any loaded register context as stale before proceeding with reset.
    m_contextEntries.count = 0;

    const uint32_t AIRCR_Address = 0xE000ED0C;
    const uint32_t AIRCR_KEY_Shift = 16;
    const uint32_t AIRCR_KEY_Mask = 0xFFFF << AIRCR_KEY_Shift;
    const uint32_t AIRCR_KEY_VALUE = 0x05FA << AIRCR_KEY_Shift;
    const uint32_t AIRCR_SYSRESETREQ_Bit = 1 << 2;
    uint32_t AIRCR_Value = 0;
    if (readMemory(AIRCR_Address, &AIRCR_Value, sizeof(AIRCR_Value), SWD::TRANSFER_32BIT) != sizeof(AIRCR_Value))
    {
        logErrorF("Core%lu: Failed to read AIRCR register for device reset.", m_coreId);
    }

    // Clear out the existing key value and use the special ones to enable writes.
    // Then set the SYSRESETREQ bit to request a device reset.
    AIRCR_Value = (AIRCR_Value & ~AIRCR_KEY_Mask) | AIRCR_KEY_VALUE | AIRCR_SYSRESETREQ_Bit;

    if (writeMemory(AIRCR_Address, &AIRCR_Value, sizeof(AIRCR_Value), SWD::TRANSFER_32BIT) != sizeof(AIRCR_Value))
    {
        logErrorF("Core%lu: Failed to write AIRCR register for device reset.", m_coreId);
    }
}

bool CpuCore::waitForCoreToReset(uint32_t timeout_ms)
{
    absolute_time_t endTime = make_timeout_time_ms(timeout_ms);
    do
    {
        refreshCoreState(0);
        if (isResetting())
        {
            return true;
        }
    } while (absolute_time_diff_us(get_absolute_time(), endTime) > 0);

    return false;
}

bool CpuCore::waitForCoreToHalt(uint32_t timeout_ms)
{
    absolute_time_t endTime = make_timeout_time_ms(timeout_ms);
    do
    {
        refreshCoreState(0);
        if (isHalted())
        {
            return true;
        }
    } while (absolute_time_diff_us(get_absolute_time(), endTime) > 0);

    return false;
}

void CpuCore::configureSingleSteppingBitsInDHCSR(bool enableSingleStepping)
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
        logErrorF("Core%lu: Failed to update C_STEP & C_MASKINTS bits in DHCSR to %s single stepping.",
                  m_coreId, enableSingleStepping ? "enable" : "disable");
    }
}

uint32_t CpuCore::setBreakpoint(uint32_t breakpointAddress, bool is32BitInstruction)
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
        logInfoF("Core%lu: No free hardware breakpoints for setting breakpoint at address 0x%08lX.",
                 m_coreId, breakpointAddress);
        return 0;
    }

    uint32_t comparatorValue = calculateFPBComparatorValue(breakpointAddress, is32BitInstruction);
    if (writeMemory(freeFPBBreakpointComparator, &comparatorValue, sizeof(comparatorValue), SWD::TRANSFER_32BIT) != sizeof(comparatorValue))
    {
        logErrorF("Core%lu: Failed to set breakpoint at address 0x%08lX.", m_coreId, breakpointAddress);
        return 0;
    }
    return freeFPBBreakpointComparator;
}

uint32_t CpuCore::findFPBBreakpointComparator(uint32_t breakpointAddress, bool is32BitInstruction)
{
    uint32_t comparatorValueForThisBreakpoint = calculateFPBComparatorValue(breakpointAddress, is32BitInstruction);
    uint32_t codeComparatorCount = getFPBCodeComparatorCount();
    uint32_t currentComparatorAddress = FPB_COMP_ARRAY_Address;
    for (uint32_t i = 0 ; i < codeComparatorCount ; i++)
    {
        uint32_t currentComparatorValue = 0;
        if (readMemory(currentComparatorAddress, &currentComparatorValue, sizeof(currentComparatorValue), SWD::TRANSFER_32BIT) != sizeof(currentComparatorValue))
        {
            logErrorF("Core%lu: Failed to read from FPB comparator at address 0x%08lX.",
                      m_coreId, currentComparatorAddress);
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

uint32_t CpuCore::calculateFPBComparatorValue(uint32_t breakpointAddress, bool is32BitInstruction)
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

bool CpuCore::isBreakpointAddressInvalid(uint32_t breakpointAddress)
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

uint32_t CpuCore::getFPBRevision()
{
    // Flash Patch breakpoint architecture revision. 0 for revision 1 and 1 for revision 2.
    uint32_t FP_CTRL_REV_Shift = 28;
    uint32_t FP_CTRL_REV_Mask = (0xF << FP_CTRL_REV_Shift);

    uint32_t controlValue = readFPControlRegister();
    return ((controlValue & FP_CTRL_REV_Mask) >> FP_CTRL_REV_Shift);
}

bool CpuCore::isAddressOdd(uint32_t address)
{
    return !!(address & 0x1);
}

bool CpuCore::isAddressAboveLowestHalfGig(uint32_t address)
{
    return !!(address & 0xE0000000);
}

uint32_t CpuCore::calculateFPBComparatorValueRevision1(uint32_t breakpointAddress, bool is32BitInstruction)
{
    uint32_t    comparatorValue;
    comparatorValue = (breakpointAddress & FP_COMP_COMP_Mask);
    comparatorValue |= FP_COMP_ENABLE_Bit;
    comparatorValue |= calculateFPBComparatorReplaceValue(breakpointAddress, is32BitInstruction);

    return comparatorValue;
}

uint32_t CpuCore::calculateFPBComparatorReplaceValue(uint32_t breakpointAddress, bool is32BitInstruction)
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

bool CpuCore::isAddressInUpperHalfword(uint32_t address)
{
    return !!(address & 0x2);
}

uint32_t CpuCore::calculateFPBComparatorValueRevision2(uint32_t breakpointAddress)
{
    return breakpointAddress | FP_COMP_BE_Bit;
}

uint32_t CpuCore::maskOffFPBComparatorReservedBits(uint32_t comparatorValue)
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

uint32_t CpuCore::findFreeFPBBreakpointComparator()
{
    uint32_t currentComparatorAddress = FPB_COMP_ARRAY_Address;
    uint32_t codeComparatorCount = getFPBCodeComparatorCount();
    for (uint32_t i = 0 ; i < codeComparatorCount ; i++)
    {
        uint32_t currentComparatorValue = 0;
        if (readMemory(currentComparatorAddress, &currentComparatorValue, sizeof(currentComparatorValue), SWD::TRANSFER_32BIT) != sizeof(currentComparatorValue))
        {
            logErrorF("Core%lu: Failed to read from FPB comparator at address 0x%08lX.",
                      m_coreId, currentComparatorAddress);
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

bool CpuCore::isFPBComparatorEnabled(uint32_t comparator)
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

bool CpuCore::isFPBComparatorEnabledRevision1(uint32_t comparator)
{
    return !!(comparator & FP_COMP_ENABLE_Bit);
}

bool CpuCore::isFPBComparatorEnabledRevision2(uint32_t comparator)
{
    return !!((comparator & FP_COMP_BE_Bit) || (comparator & FP_COMP_FE_Bit));
}

uint32_t CpuCore::clearBreakpoint(uint32_t breakpointAddress, bool is32BitInstruction)
{
    uint32_t existingFPBBreakpoint = findFPBBreakpointComparator(breakpointAddress, is32BitInstruction);
    if (existingFPBBreakpoint != 0)
    {
        clearFPBComparator(existingFPBBreakpoint);
        logInfoF("Core%lu: Hardware breakpoint cleared at address 0x%08lX.", m_coreId, breakpointAddress);
    }

    return existingFPBBreakpoint;
}

void CpuCore::setWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type)
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
    logInfoF("Core%lu: Hardware watchpoint set at address 0x%08X.", m_coreId, address);
}

uint32_t CpuCore::convertWatchpointTypeToCortexMType(PlatformWatchpointType type)
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

bool CpuCore::isValidDWTComparatorSetting(uint32_t watchpointAddress,
                                           uint32_t watchpointSize,
                                           uint32_t watchpointType)
{
    return isValidDWTComparatorSize(watchpointSize) &&
           isValidDWTComparatorAddress(watchpointAddress, watchpointSize) &&
           isValidDWTComparatorType(watchpointType);
}

bool CpuCore::isValidDWTComparatorSize(uint32_t watchpointSize)
{
    return isPowerOf2(watchpointSize);
}

bool CpuCore::isPowerOf2(uint32_t value)
{
    return (value & (value - 1)) == 0;
}

bool CpuCore::isValidDWTComparatorAddress(uint32_t watchpointAddress, uint32_t watchpointSize)
{
    return isAddressAlignedToSize(watchpointAddress, watchpointSize);
}

bool CpuCore::isAddressAlignedToSize(uint32_t address, uint32_t size)
{
    uint32_t addressMask = ~(size - 1);
    return address == (address & addressMask);
}

bool CpuCore::isValidDWTComparatorType(uint32_t watchpointType)
{
    return (watchpointType == DWT_COMP_FUNCTION_FUNCTION_DATA_READ) ||
           (watchpointType == DWT_COMP_FUNCTION_FUNCTION_DATA_WRITE) ||
           (watchpointType == DWT_COMP_FUNCTION_FUNCTION_DATA_READWRITE);
}

uint32_t CpuCore::enableDWTWatchpoint(uint32_t watchpointAddress,
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
        logInfoF("Core%lu: No free hardware watchpoints for setting watchpoint at address 0x%08lX.",
                 m_coreId, watchpointAddress);
        return 0;
    }

    if (!attemptToSetDWTComparator(comparatorAddress, watchpointAddress, watchpointSize, watchpointType))
    {
        // Failed set due to the size being larger than supported by CPU.
        logInfoF("Core%lu: Failed to set watchpoint at address 0x%08lX of size %lu bytes.",
                 m_coreId, watchpointAddress, watchpointSize);
        return 0;
    }

    // Successfully configured a free comparator for this watchpoint.
    return comparatorAddress;
}

uint32_t CpuCore::findDWTComparator(uint32_t watchpointAddress,
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

bool CpuCore::doesDWTComparatorMatch(uint32_t comparatorAddress,
                                      uint32_t address,
                                      uint32_t size,
                                      uint32_t function)
{
    return doesDWTComparatorFunctionMatch(comparatorAddress, function) &&
           doesDWTComparatorAddressMatch(comparatorAddress, address) &&
           doesDWTComparatorMaskMatch(comparatorAddress, size);
}

bool CpuCore::doesDWTComparatorFunctionMatch(uint32_t comparatorAddress, uint32_t function)
{
    uint32_t functionValue = 0;
    if (readMemory(comparatorAddress + offsetof(DWT_COMP_Type, function), &functionValue, sizeof(functionValue), SWD::TRANSFER_32BIT) != sizeof(functionValue))
    {
        logErrorF("Core%lu: Failed to read DWT function register.", m_coreId);
        return false;
    }
    uint32_t importantFunctionBits = maskOffDWTFunctionBits(functionValue);

    return importantFunctionBits == function;
}

uint32_t CpuCore::maskOffDWTFunctionBits(uint32_t functionValue)
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

bool CpuCore::doesDWTComparatorAddressMatch(uint32_t comparatorAddress, uint32_t address)
{
    uint32_t compValue = 0;
    if (readMemory(comparatorAddress + offsetof(DWT_COMP_Type, comp), &compValue, sizeof(compValue), SWD::TRANSFER_32BIT) != sizeof(compValue))
    {
        logErrorF("Core%lu: Failed to read DWT comparator register.", m_coreId);
        return false;
    }
    return compValue == address;
}

bool CpuCore::doesDWTComparatorMaskMatch(uint32_t comparatorAddress, uint32_t size)
{
    uint32_t maskValue = 0;
    if (readMemory(comparatorAddress + offsetof(DWT_COMP_Type, mask), &maskValue, sizeof(maskValue), SWD::TRANSFER_32BIT) != sizeof(maskValue))
    {
        logErrorF("Core%lu: Failed to read DWT mask register.", m_coreId);
        return false;
    }
    return maskValue == calculateLog2(size);
}

uint32_t CpuCore::calculateLog2(uint32_t value)
{
    uint32_t log2 = 0;

    while (value > 1)
    {
        value >>= 1;
        log2++;
    }

    return log2;
}

uint32_t CpuCore::findFreeDWTComparator()
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

bool CpuCore::isDWTComparatorFree(uint32_t comparatorAddress)
{
    //  Selects action to be taken on match.
    //      Disabled
    const uint32_t DWT_COMP_FUNCTION_FUNCTION_DISABLED = 0x0;

    uint32_t functionValue = 0;
    if (readMemory(comparatorAddress + offsetof(DWT_COMP_Type, function), &functionValue, sizeof(functionValue), SWD::TRANSFER_32BIT) != sizeof(functionValue))
    {
        logErrorF("Core%lu: Failed to read DWT function register.", m_coreId);
        return false;
    }
    return (functionValue & DWT_COMP_FUNCTION_FUNCTION_Mask) == DWT_COMP_FUNCTION_FUNCTION_DISABLED;
}

bool CpuCore::attemptToSetDWTComparator(uint32_t comparatorAddress,
                                         uint32_t watchpointAddress,
                                         uint32_t watchpointSize,
                                         uint32_t watchpointType)
{
    if (!attemptToSetDWTComparatorMask(comparatorAddress, watchpointSize))
    {
        logErrorF("Core%lu: Failed to set DWT mask register to a size of %lu bytes.", m_coreId, watchpointSize);
        return false;
    }
    if (writeMemory(comparatorAddress + offsetof(DWT_COMP_Type, comp), &watchpointAddress, sizeof(watchpointAddress), SWD::TRANSFER_32BIT) != sizeof(watchpointAddress))
    {
        logErrorF("Core%lu: Failed to write DWT comparator register.", m_coreId);
        return false;
    }
    if (writeMemory(comparatorAddress + offsetof(DWT_COMP_Type, function), &watchpointType, sizeof(watchpointType), SWD::TRANSFER_32BIT) != sizeof(watchpointType))
    {
        logErrorF("Core%lu: Failed to write DWT function register.", m_coreId);
        return false;
    }
    return true;
}

bool CpuCore::attemptToSetDWTComparatorMask(uint32_t comparatorAddress, uint32_t watchpointSize)
{
    uint32_t maskBitCount;

    maskBitCount = calculateLog2(watchpointSize);
    if (writeMemory(comparatorAddress + offsetof(DWT_COMP_Type, mask), &maskBitCount, sizeof(maskBitCount), SWD::TRANSFER_32BIT) != sizeof(maskBitCount))
    {
        logErrorF("Core%lu: Failed to write DWT mask register.", m_coreId);
        return false;
    }

    // Processor may limit number of bits to be masked off so check.
    uint32_t maskValue = 0;
    if (readMemory(comparatorAddress + offsetof(DWT_COMP_Type, mask), &maskValue, sizeof(maskValue), SWD::TRANSFER_32BIT) != sizeof(maskValue))
    {
        logErrorF("Core%lu: Failed to read DWT mask register.", m_coreId);
        return false;
    }
    return maskValue == maskBitCount;
}

void CpuCore::clearWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type)
{
    uint32_t nativeType = convertWatchpointTypeToCortexMType(type);

    if (!isValidDWTComparatorSetting(address, size, nativeType))
    {
        __throw(invalidArgumentException);
    }

    disableDWTWatchpoint(address, size, nativeType);
}

uint32_t CpuCore::disableDWTWatchpoint(uint32_t watchpointAddress,
                                        uint32_t watchpointSize,
                                        uint32_t watchpointType)
{
    uint32_t comparatorAddress = findDWTComparator(watchpointAddress, watchpointSize, watchpointType);
    if (comparatorAddress == 0)
    {
        // This watchpoint not set so return NULL.
        return 0;
    }

    logInfoF("Core%lu: Hardware watchpoint cleared at address 0x%08lX.", m_coreId, watchpointAddress);
    clearDWTComparator(comparatorAddress);
    return comparatorAddress;
}

uint8_t CpuCore::determineCauseOfException(bool wasStopFromGDB)
{
    uint32_t DFSR_Value = 0;
    if (readMemory(DFSR_Address, &DFSR_Value, sizeof(DFSR_Value), SWD::TRANSFER_32BIT) != sizeof(DFSR_Value))
    {
        logErrorF("Core%lu: Failed to read DFSR register.", m_coreId);
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

uint32_t CpuCore::getExceptionNumber()
{
    assert ( m_contextEntries.count != 0 );
    return Context_Get(&m_context, CPSR) & IPSR_Mask;
}

void CpuCore::displayFaultCauseToGdbConsole()
{
    // Nothing to do on ARMv6-M devices since they don't have fault status registers.
    if (m_swd.getCpuType() < SWD::CPU_CORTEX_M3)
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

void CpuCore::displayHardFaultCauseToGdbConsole()
{
    const uint32_t HFSR_Address = 0xE000ED2C;
    const uint32_t debugEventBit = 1 << 31;
    const uint32_t forcedBit = 1 << 30;
    const uint32_t vectorTableReadBit = 1 << 1;
    uint32_t       hardFaultStatusRegister = 0xBAADFEED;

    if (readMemory(HFSR_Address, &hardFaultStatusRegister, sizeof(hardFaultStatusRegister), SWD::TRANSFER_32BIT) != sizeof(hardFaultStatusRegister))
    {
        logErrorF("Core%lu: Failed to read the HFSR register.", m_coreId);
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

void CpuCore::displayMemFaultCauseToGdbConsole()
{
    const uint32_t MMARValidBit = 1 << 7;
    const uint32_t FPLazyStatePreservationBit = 1 << 5;
    const uint32_t stackingErrorBit = 1 << 4;
    const uint32_t unstackingErrorBit = 1 << 3;
    const uint32_t dataAccess = 1 << 1;
    const uint32_t instructionFetch = 1;
    uint32_t       CFSR_Value = 0xBAADFEED;

    if (readMemory(CFSR_Address, &CFSR_Value, sizeof(CFSR_Value), SWD::TRANSFER_32BIT) != sizeof(CFSR_Value))
    {
        logErrorF("Core%lu: Failed to read the CFSR register.", m_coreId);
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
        if (readMemory(MMFAR_Address, &MMFAR_Value, sizeof(MMFAR_Value), SWD::TRANSFER_32BIT) != sizeof(MMFAR_Value))
        {
            logErrorF("Core%lu: Failed to read the MMFAR register.", m_coreId);
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
        WriteHexValueToGdbConsole(Context_Get(&m_context, SP));
    }
    if (memManageFaultStatusRegister & unstackingErrorBit)
    {
        WriteStringToGdbConsole("\n    Unstacking Error w/ SP = ");
        WriteHexValueToGdbConsole(Context_Get(&m_context, SP));
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

void CpuCore::displayBusFaultCauseToGdbConsole()
{
    const uint32_t BFARValidBit = 1 << 7;
    const uint32_t FPLazyStatePreservationBit = 1 << 5;
    const uint32_t stackingErrorBit = 1 << 4;
    const uint32_t unstackingErrorBit = 1 << 3;
    const uint32_t impreciseDataAccessBit = 1 << 2;
    const uint32_t preciseDataAccessBit = 1 << 1;
    const uint32_t instructionPrefetch = 1;
    uint32_t       CFSR_Value = 0xBAADFEED;

    if (readMemory(CFSR_Address, &CFSR_Value, sizeof(CFSR_Value), SWD::TRANSFER_32BIT) != sizeof(CFSR_Value))
    {
        logErrorF("Core%lu: Failed to read the CFSR register.", m_coreId);
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
        if (readMemory(BFAR_Address, &BFAR_Value, sizeof(BFAR_Value), SWD::TRANSFER_32BIT) != sizeof(BFAR_Value))
        {
            logErrorF("Core%lu: Failed to read the BFAR register.", m_coreId);
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
        WriteHexValueToGdbConsole(Context_Get(&m_context, SP));
    }
    if (busFaultStatusRegister & unstackingErrorBit)
    {
        WriteStringToGdbConsole("\n    Unstacking Error w/ SP = ");
        WriteHexValueToGdbConsole(Context_Get(&m_context, SP));
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

void CpuCore::displayUsageFaultCauseToGdbConsole()
{
    const uint32_t divideByZeroBit = 1 << 9;
    const uint32_t unalignedBit = 1 << 8;
    const uint32_t coProcessorAccessBit = 1 << 3;
    const uint32_t invalidPCBit = 1 << 2;
    const uint32_t invalidStateBit = 1 << 1;
    const uint32_t undefinedInstructionBit = 1;
    uint32_t       CFSR_Value = 0xBAADFEED;

    if (readMemory(CFSR_Address, &CFSR_Value, sizeof(CFSR_Value), SWD::TRANSFER_32BIT) != sizeof(CFSR_Value))
    {
        logErrorF("Core%lu: Failed to read the CFSR register.", m_coreId);
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

bool CpuCore::startCodeOnDevice(const CortexM_Registers* pRegistersIn)
{
    // Could lose original register values if context hasn't been loaded.
    assert ( m_contextEntries.count != 0 );

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
    if (!clearHaltBit())
    {
        logError("Failed to start executing debugger code on target device.");
        reenableInterrupts();
        return false;
    }

    return true;
}

bool CpuCore::disableSingleStepAndInterrupts()
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

bool CpuCore::reenableInterrupts()
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

bool CpuCore::waitForCodeToHalt(CortexM_Registers* pRegistersOut, uint32_t timeout_ms)
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
    } while (!isHalted() && absolute_time_diff_us(get_absolute_time(), endTime) > 0);
    reenableInterrupts();
    if (!isHalted())
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

bool CpuCore::runCodeOnDevice(CortexM_Registers* pRegistersInOut, uint32_t timeout_ms)
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




// Buffer to be used for storing extra thread info.
char CpuCores::m_threadExtraInfo[16];

CpuCores::CpuCores()
{
}

bool CpuCores::init(SWD* pSwdBus, void (*pHandler)(void))
{
    // Don't call init() again without first calling uninit() to cleanup from the previous SWD connection.
    assert ( m_coreCount == 0 );

    // Initialize the first/default core.
    CpuCore* pDefaultCore = &m_cores[0];
    if (!pDefaultCore->init(pSwdBus, pHandler, 0))
    {
        logError("Failed to initialize core0's debug components.");
        return false;
    }
    m_pDefaultCore = pDefaultCore;
    m_coreCount = 1;
    m_pSwdBus = pSwdBus;
    m_pHaltedCore = NULL;
    m_hasDetectedSwdDisconnect = false;
    m_connectionFailureHandler =  pHandler;

    searchSupportedDevicesList();
    determineFpuAvailability();
    updateSwdFrequencyForDevice();
    constructMemoryLayoutXML();
    checkForMultipleCores(pHandler);
    assert ( m_coreCount > 0 );

    return true;
}

// UNDONE: Device code should take CpuCore object and not SwdTarget. SwdTarget can be contained within CpuCore.
void CpuCores::searchSupportedDevicesList()
{
    assert ( m_pDevice == NULL && m_pDeviceObject == NULL );
    for (size_t i = 0; i < g_supportedDevicesLength ; i++)
    {
        DeviceFunctionTable* pDevice = g_supportedDevices[i];
        DeviceObject* pObject = pDevice->detect(m_pDefaultCore);
        if (pObject)
        {
            m_pDevice = pDevice;
            m_pDeviceObject = pObject;
            logInfoF("Found device of type %s.", pDevice->getName(m_pDeviceObject, m_pDefaultCore));
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
        // Device driver will tell us explicitly if this device supports FPU or not.
        if (m_pDevice->hasFpu(m_pDeviceObject, m_pDefaultCore))
        {
            logInfo("Device has FPU.");
            m_fpu = CpuCore::FPU_AVAILABLE;
        }
        else
        {
            m_fpu = CpuCore::FPU_NONE;
        }
    }
    else
    {
        // Don't know for sure if the device has FPU at init time so will have to determine on the fly.
        if (m_pDefaultCore->getTarget()->getCpuType() >= SWD::CPU_CORTEX_M4)
        {
            // Device might have FPU. Check on first halt.
            m_fpu = CpuCore::FPU_MAYBE;
        }
        else
        {
            // Cortex-M3 and lower never have a FPU.
            m_fpu = CpuCore::FPU_NONE;
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
    uint32_t swdFrequency = m_pDevice->getMaximumSWDClockFrequency(m_pDeviceObject, m_pDefaultCore);
    if (!m_pDefaultCore->getTarget()->setFrequency(swdFrequency))
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
        pLayout = m_pDevice->getMemoryLayout(m_pDeviceObject, m_pDefaultCore);
    }
    else
    {
        // Fall back to the default memory layout used by all Cortex-M devices. It will report regions that are larger
        // than those actually found on the device but it does know which regions are read-only and which are
        // read/write.
        pLayout = deviceDefaultMemoryLayout(NULL, m_pDefaultCore);
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

void CpuCores::checkForMultipleCores(void (*pHandler)(void))
{
    if (!m_pDevice)
    {
        return;
    }

    size_t coreCount = 0;
    if (!m_pDevice->getAdditionalTargets(m_pDeviceObject, m_pDefaultCore, &m_cores[1], count_of(m_cores)-1, &coreCount))
    {
        logError("Failed calling m_pDevice->getAdditionalTargets()");
        return;
    }
    assert ( coreCount <= count_of(m_cores)-1 );
    m_coreCount = coreCount + 1;
    logInfoF("Device has %u core(s).", m_coreCount);
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
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        m_cores[i].uninit();
    }
    m_coreCount = 0;
    m_pDefaultCore = NULL;
    m_pHaltedCore = NULL;
}

void CpuCores::cleanupDeviceObject()
{
    if (!m_pDevice)
    {
        return;
    }

    m_pDevice->free(m_pDeviceObject, m_pDefaultCore);
    m_pDeviceObject = NULL;
    m_pDevice = NULL;
}

void CpuCores::disconnect()
{
    cleanupDeviceObject();
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        m_cores[i].disconnect();
    }
    m_coreCount = 0;
    m_pDefaultCore = NULL;
    m_pHaltedCore = NULL;
}

bool CpuCores::refreshCoreStates(uint32_t timeout_ms)
{
    assert ( m_coreCount > 0 && m_coreCount <= count_of(m_cores) );
    bool result = true;
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        result &= m_cores[i].refreshCoreState(timeout_ms);
    }
    return result;
}

int CpuCores::indexOfResettingCore()
{
    assert ( m_coreCount > 0 && m_coreCount <= count_of(m_cores) );
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        if (m_cores[i].isResetting())
        {
            return i;
        }
    }
    return CORE_NONE;
}

int CpuCores::indexOfHaltedCore(bool requireDebugEvent)
{
    int haltedCore = CORE_NONE;
    assert ( m_coreCount > 0 && m_coreCount <= count_of(m_cores) );
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        if (m_cores[i].isHalted() && m_cores[i].getState() != MRI_PLATFORM_THREAD_FROZEN)
        {
            // When requiring debug event, we want to ignore the HALT debug event which would have been generated when
            // some cores were forced to halt.
            if (m_cores[i].hasDebugEvent(requireDebugEvent))
            {
                // Always have a preference for halted cores which have events to be debugged.
                return i;
            }
            else if (!requireDebugEvent && haltedCore == CORE_NONE)
            {
                // If requireDebugEvent is false then just return first halted core detected.
                haltedCore = i;
            }
        }
    }
    if (!requireDebugEvent && haltedCore != CORE_NONE)
    {
        logInfoF("Treating Core%d as halted even though it doesn't have a debug event.", haltedCore);
    }
    return haltedCore;
}

bool CpuCores::requestCoresToHalt(int alreadyHaltedCore)
{
    // UNDONE: Might not need alreadyHaltedCore parameter now.
    assert ( m_coreCount > 0 && m_coreCount <= count_of(m_cores) );
    bool result = true;
    for (int i = 0 ; i < (int)m_coreCount ; i++)
    {
        if (!m_cores[i].isHalted())
        {
            result &= m_cores[i].requestHalt();
        }
    }
    return result;
}

bool CpuCores::waitForCoresToHalt(uint32_t timeout_ms)
{
    size_t coresToHalt = m_coreCount;
    uint32_t coresHalted = 0;
    absolute_time_t endTime = make_timeout_time_ms(timeout_ms);
    do
    {
        for (size_t i = 0 ; i < m_coreCount ; i++)
        {
            uint32_t coreMask = 1 << i;
            if ((coresHalted & coreMask) == 0)
            {
                m_cores[i].refreshCoreState(0);
                if (m_cores[i].isHalted())
                {
                    coresHalted |= coreMask;
                    coresToHalt--;
                }
            }
        }
    } while (coresToHalt > 0 && absolute_time_diff_us(get_absolute_time(), endTime) > 0);
    return coresToHalt == 0;
}

void CpuCores::enterMriCore(int haltedCore)
{
    assert ( haltedCore >= 0 && haltedCore < (int)count_of(m_cores) );
    m_pHaltedCore = &m_cores[haltedCore];

    markAllCoresFrozen();
    m_pDefaultCore->checkForFpu(&m_fpu);
    mriDebugException(m_pHaltedCore->getContext(m_fpu));
}

void CpuCores::markAllCoresFrozen()
{
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        m_cores[i].setState(MRI_PLATFORM_THREAD_FROZEN);
    }
}

void CpuCores::enteringDebugger()
{
    assert ( m_pHaltedCore != NULL );

    m_trapReason = m_pHaltedCore->determineTrapReason();
    disableResetVectorCatchAndSingleStepping();
}

void CpuCores::disableResetVectorCatchAndSingleStepping()
{
    assert ( m_coreCount > 0 && m_coreCount <= count_of(m_cores) );
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        m_cores[i].disableResetVectorCatch();
        m_cores[i].disableSingleStep();
    }
}

void CpuCores::leavingDebugger()
{
    assert ( m_pHaltedCore != NULL );
    m_pHaltedCore->clearDebugEvent();
}

void CpuCores::resume()
{
    assert ( m_coreCount > 0 && m_coreCount <= count_of(m_cores) );

    // We are done debugging the current core so forget about it in m_pHaltedCore.
    assert ( m_pHaltedCore != NULL && !m_pHaltedCore->hasDebugEvent(true) );
    m_pHaltedCore = NULL;

    // Write out the context for all cores which might have been modified.
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        m_cores[i].writeContext(m_fpu);
    }

    // Don't actually resume execution if there are still any cores with debug events to be debugged. The main loop
    // will detect the still halted core and jump back into debug mode for GDB to debug this core.
    if (indexOfHaltedCore(true) != CORE_NONE)
    {
        return;
    }

    // Debug events on all cores have been seen by GDB so it is safe to resume execution of all cores again.
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        m_cores[i].resume();
    }
}

void CpuCores::reset(bool haltOnReset, uint32_t timeout_ms)
{
    // Walk the core list backwards so that core 0 is reset last.
    // Reset each core and wait for it to halt at the reset vector.
    assert ( m_coreCount > 0 && m_coreCount <= count_of(m_cores) );
    for (int i = m_coreCount - 1 ; i >= 0 ; i--)
    {
        logInfoF("Resetting%s Core%d.", haltOnReset ? " and halting" : "", i);
        m_cores[i].enableResetVectorCatch();
        m_cores[i].reset();
        if (!m_cores[i].waitForCoreToReset(timeout_ms))
        {
            logErrorF("Core%d: Timed out waiting for reset.", i);
        }
        if (!m_cores[i].waitForCoreToHalt(timeout_ms))
        {
            logErrorF("Core%d: Timed out waiting for halt after reset.", i);
        }
        m_cores[i].disableResetVectorCatch();
        m_cores[i].clearDebugEvent();
    }

    // We are currently halted at the reset vector at this point.
    if (haltOnReset)
    {
        return;
    }

    // Walk the core list backwards so that core 0 is started last.
    for (int i = m_coreCount - 1 ; i >= 0 ; i--)
    {
        m_cores[i].setState(MRI_PLATFORM_THREAD_THAWED);
        if (!m_cores[i].resume())
        {
            logErrorF("Core%lu: Failed to resume.", i);
        }
    }
}

void CpuCores::enableResetVectorCatch()
{
    assert ( m_coreCount > 0 && m_coreCount <= count_of(m_cores) );
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        m_cores[i].enableResetVectorCatch();
    }
}

uint32_t CpuCores::setBreakpoint(uint32_t breakpointAddress, bool is32BitInstruction)
{
    uint32_t retValue = 0xFFFFFFFF;
    assert ( m_coreCount > 0 && m_coreCount <= count_of(m_cores) );
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        uint32_t result = m_cores[i].setBreakpoint(breakpointAddress, is32BitInstruction);
        // Expect all cores to be configured the same.
        assert ( retValue == 0xFFFFFFFF || result == retValue );
        retValue = result;
    }
    return retValue;
}

uint32_t CpuCores::clearBreakpoint(uint32_t breakpointAddress, bool is32BitInstruction)
{
    uint32_t retValue = 0xFFFFFFFF;
    assert ( m_coreCount > 0 && m_coreCount <= count_of(m_cores) );
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        uint32_t result = m_cores[i].clearBreakpoint(breakpointAddress, is32BitInstruction);
        // Expect all cores to be configured the same.
        assert ( retValue == 0xFFFFFFFF || result == retValue );
        retValue = result;
    }
    return retValue;
}

void CpuCores::setWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type)
{
    int test = -1;
    assert ( m_coreCount > 0 && m_coreCount <= count_of(m_cores) );
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        int exceptionCode = noException;
        __try
        {
            m_cores[i].setWatchpoint(address, size, type);
        }
        __catch
        {
            exceptionCode = getExceptionCode();
        }
        // Expect all cores to be configured the same.
        assert ( test == -1 || exceptionCode == test );
        test = exceptionCode;
    }
}

void CpuCores::clearWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type)
{
    int test = -1;
    assert ( m_coreCount > 0 && m_coreCount <= count_of(m_cores) );
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        int exceptionCode = noException;
        __try
        {
            m_cores[i].clearWatchpoint(address, size, type);
        }
        __catch
        {
            exceptionCode = getExceptionCode();
        }
        // Expect all cores to be configured the same.
        assert ( test == -1 || exceptionCode == test );
        test = exceptionCode;
    }
}

bool CpuCores::dispatchMonitorCommandToDevice(const char** ppArgs, size_t argCount)
{
    if (m_pDevice == NULL || m_pDevice->handleMonitorCommand == NULL)
    {
        // No device specific command handler so just return false.
        return false;
    }

    assert ( m_pDefaultCore != NULL );
    return m_pDevice->handleMonitorCommand(m_pDeviceObject, m_pDefaultCore, ppArgs, argCount);
}

bool CpuCores::flashBegin()
{
    assert ( m_pDefaultCore != NULL );
    return m_pDevice->flashBegin(m_pDeviceObject, m_pDefaultCore);
}

bool CpuCores::flashErase(uint32_t addressStart, uint32_t length)
{
    assert ( m_pDefaultCore != NULL );
    return m_pDevice->flashErase(m_pDeviceObject, m_pDefaultCore, addressStart, length);
}

bool CpuCores::flashProgram(uint32_t addressStart, const void* pBuffer, size_t bufferSize)
{
    assert ( m_pDefaultCore != NULL );
    return m_pDevice->flashProgram(m_pDeviceObject, m_pDefaultCore, addressStart, pBuffer, bufferSize);
}

bool CpuCores::flashEnd()
{
    assert ( m_pDefaultCore != NULL );
    return m_pDevice->flashEnd(m_pDeviceObject, m_pDefaultCore);
}

const char* CpuCores::getExtraThreadInfo(uintmri_t threadId)
{
    if (!isThreadActive(threadId))
    {
        return NULL;
    }

    snprintf(m_threadExtraInfo, sizeof(m_threadExtraInfo), "Core%u", threadId-1);
    return m_threadExtraInfo;
}

void CpuCores::restorePrevThreadState()
{
    for (size_t i = 0 ; i < m_coreCount ; i++)
    {
        m_cores[i].restoreThreadState();
    }
}

void CpuCores::setThreadState(uintmri_t threadId, PlatformThreadState state)
{
    if (threadId == MRI_PLATFORM_ALL_THREADS || threadId == MRI_PLATFORM_ALL_FROZEN_THREADS)
    {
        for (size_t i = 0 ; i < m_coreCount ; i++)
        {
            if (threadId == MRI_PLATFORM_ALL_THREADS ||
                (threadId == MRI_PLATFORM_ALL_FROZEN_THREADS && m_cores[i].getState() == MRI_PLATFORM_THREAD_FROZEN))
            {
                m_cores[i].setState(state);
            }
        }

        return;
    }
    else if (isThreadActive(threadId))
    {
        m_cores[threadId-1].setState(state);
    }
}
