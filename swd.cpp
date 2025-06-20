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
// Use the RP2040's PIO state machines to implement the ARM Serial Wire Debug (SWD) low level signalling.
#define SWD_MODULE "swd.cpp"
#include "logging.h"
#include <string.h>
#include <hardware/clocks.h>
#include <swd.pio.h>
#include "swd.h"


// Array of known DPv2 targets.
static SWD::DPv2Targets g_knownDPv2Targets[] =
{
    SWD::RP2040_CORE0,
    SWD::RP2350,
};


void SWD::disableErrorLogging()
{
    logErrorDisable();
}

void SWD::enableErrorLogging()
{
    logErrorEnable();
}



SWD::SWD()
{
}

bool SWD::init(PIO pio, uint32_t frequency, uint32_t swclkPin, uint32_t swdioPin)
{
    // Cleanup the PIO if it was already initialized before continuing to initialize it again, with probably different
    // parameters (ie. frequency).
    uninit();

    if (!pio_can_add_program(pio, &swd_program))
    {
        return false;
    }
    m_pio = pio;

    // Find an unused state machine in the PIO to run the SWD I/O code.
    const bool panicIfNotAvailable = true;
    const bool noPanicIfNotAvailable = !panicIfNotAvailable;
    int32_t stateMachine = pio_claim_unused_sm(m_pio, noPanicIfNotAvailable);
    if (stateMachine < 0)
    {
        // No free state machines so return a failure code.
        return false;
    }
    m_stateMachine = stateMachine;

    // Remember the SWD pins.
    m_swclkPin = swclkPin;
    m_swdioPin = swdioPin;

    // Load the PIO program.
    // If the PIO clock can be run at 4x the desired SWD frequency then 1 delay cycle will need to be added to each
    // instruction which contains side setting of the SWCLK. Why do this? Because it allows the non-side setting
    // instructions to run twice as fast and waste less time between actual bit read/writes.
    uint32_t cpuFrequency = clock_get_hz(clk_sys);
    float frequencyScale = 2.0f;
    if (frequency <= cpuFrequency / 4)
    {
        // Modify program to enable 4x PIO clock rate.
        uint16_t programInstructions[count_of(swd_program_instructions)];
        for (size_t i = 0 ; i < count_of(swd_program_instructions) ; i++)
        {
            // If this bit is set in a PIO instruction then it is side setting the clock signal and should have an extra
            // delay cycle added to allow running the PIO clock twice as fast.
            const uint16_t optionalSideSetBit = 1 << 12;
            // The delay bits start at this bit offset.
            const uint32_t delayShift = 8;
            // Asserting this value into an instruction will add 1 delay cycle.
            const uint32_t delayOfOne = 1 << delayShift;

            uint16_t instr = swd_program_instructions[i];
            if (instr & optionalSideSetBit)
            {
                // Add 1 cycle of delay to this instruction before storing in RAM.
                programInstructions[i] = instr | delayOfOne;
            }
            else
            {
                // Not side setting SWCLK so just copy as is.
                programInstructions[i] = instr;
            }
        }

        // Load modified program into PIO.
        pio_program swdProgram = swd_program;
        swdProgram.instructions = programInstructions;
        m_programOffset = pio_add_program(m_pio, &swdProgram);
        frequencyScale = 4.0f;
    }
    else
    {
        // Load original program into PIO.
        m_programOffset = pio_add_program(m_pio, &swd_program);
        frequencyScale = 2.0f;
    }

    // Configure both SWDIO and SWCLK pins to be output pin controlled by PIO and default them both to be
    // output enabled. Start out with both SWDIO and SWCLK high.
    pio_sm_set_pins_with_mask(m_pio, m_stateMachine, 0xFFFFFFFF, (1 << swdioPin) | (1 << swclkPin));
    pio_sm_set_pindirs_with_mask(m_pio, m_stateMachine, 0xFFFFFFFF, (1 << swdioPin) | (1 << swclkPin));
    pio_gpio_init(m_pio, swdioPin);
    pio_gpio_init(m_pio, swclkPin);

    // Enable pull-up on SWDIO and pull-down on SWCLK so that disconnected signals give known values.
    // This pull-up/down configuration matches some of the target chips I have checked like the nRF52840.
    gpio_pull_up(swdioPin);
    gpio_pull_down(swclkPin);

    // Fetch the default state machine configuration for running this PIO program on a state machine.
    pio_sm_config smConfig = swd_program_get_default_config(m_programOffset);

    // Side Set needs to be configured for the SWCLK pin.
    sm_config_set_sideset_pins(&smConfig, swclkPin);

    // OUT pins should be configured for the SWDIO pin.
    sm_config_set_out_pins(&smConfig, swdioPin, 1);

    // SET pins should be configured for the SWDIO pin as well.
    sm_config_set_set_pins(&smConfig, swdioPin, 1);

    // OSR should be configured for auto pull with a threshold of 32.
    // Right shift through OSR as lsb is to be sent first.
    const bool shiftRight = true;
    const bool autoPull = true;
    const uint32_t pullThreshold = 32;
    sm_config_set_out_shift(&smConfig, shiftRight, autoPull, pullThreshold);

    // ISR should be configured for auto push with a threshold of 32.
    // Right shift through ISR as lsb are read in first.
    const bool autoPush = true;
    const uint32_t pushThreshold = 32;
    sm_config_set_in_shift(&smConfig, shiftRight, autoPush, pushThreshold);

    // INPUT pin to SWDIO as well.
    sm_config_set_in_pins(&smConfig, swdioPin);

    // Set the SWCLK frequency by setting the PIO state machine clock divisor.
    sm_config_set_clkdiv(&smConfig, (float)cpuFrequency / ((float)frequency * frequencyScale));

    // Complete the state machine configuration.
    pio_sm_init(m_pio, m_stateMachine, m_programOffset, &smConfig);

    // Now start the SWD state machine.
    pio_sm_set_enabled(m_pio, m_stateMachine, true);

    m_signalMode = BIT_PATTERN;

    return true;
}

bool SWD::init(uint32_t frequency, uint32_t swclkPin, uint32_t swdioPin)
{
    if (init(pio0, frequency, swclkPin, swdioPin) || init(pio1, frequency, swclkPin, swdioPin))
    {
        return true;
    }
    return false;
}

void SWD::uninit()
{
    if (m_stateMachine == UNKNOWN_VAL)
    {
        return;
    }

    // Stop the SWD state machine and free it up.
    pio_sm_set_enabled(m_pio, m_stateMachine, false);
    pio_sm_unclaim(m_pio, m_stateMachine);

    // Free up the space in the PIO instance.
    pio_remove_program(m_pio, &swd_program, m_programOffset);

    // Switch SWD pins to be inputs with previously configured pull-up/downs.
    gpio_set_dir_in_masked((1 << m_swclkPin) | (1 << m_swdioPin));
    gpio_set_function(m_swclkPin, GPIO_FUNC_SIO);
    gpio_set_function(m_swdioPin, GPIO_FUNC_SIO);

    m_stateMachine = UNKNOWN_VAL;
    m_pio = NULL;
    m_programOffset = UNKNOWN_VAL;
}

bool SWD::sendJtagToSwdSequence()
{
    setBitPatternSignalMode();

    assert ( m_lineResetHighClocks > 32 && m_lineResetHighClocks <= 64 );
    const uint32_t jtagToSwdSequence = 0xE79E;
    uint32_t dataToSend[] =
    {
        // Make sure that JTAG controllers are in idle state if currently selected.
        // Send a number (LINE_RESET_CLK_PULSES) of clock transitions with SWDIO held high.
        m_lineResetHighClocks-1,
        -1UL,
        -1UL,
        // Send the 16-bit sequence that switches from JTAG to SWD.
        16-1,
        jtagToSwdSequence
    };
    writeAndOptionalReadPIO(dataToSend, count_of(dataToSend), NULL, 0);

    invalidateDpApState();
    disableErrorLogging();
        bool result = sendLineReset();
        if (result)
        {
            m_target = DPv1;
        }
    enableErrorLogging();
    return result;
}

bool SWD::sendLineReset()
{
    sendRawLineReset();
    bool result = readDPIDR();
    if (!result)
    {
        logError("Failed call to readDPIDR()");
    }
    return result;
}

void SWD::sendRawLineReset()
{
    setBitPatternSignalMode();

    // Send LINE_RESET_CLK_PULSES clock transitions with SWDIO held high and then IDLE_PULSES low.
    assert ( m_lineResetHighClocks + m_lineResetLowClocks > 32 &&  m_lineResetHighClocks + m_lineResetLowClocks <= 64 );
    uint32_t dataToSend[] =
    {
        m_lineResetHighClocks+m_lineResetLowClocks-1,
        -1UL,
        (1UL << (m_lineResetHighClocks-32))-1UL
    };
    writeAndOptionalReadPIO(dataToSend, count_of(dataToSend), NULL, 0);

    // Unselects the current target.
    m_target = UNKNOWN;
    m_dpidr = UNKNOWN_VAL;
    m_dpVersion = 0;
}

void SWD::writeAndOptionalReadPIO(uint32_t* pWrite, uint32_t writeLength, uint32_t* pRead, uint32_t readLength)
{
    // This function runs out of RAM with -O3 enabled and manual inlining of the PIO FIFO functions so that the
    // operations can be queued up as fast as possible so that PIO state machine doesn't stall with SWCLK low while
    // waiting for the data in the FIFO.
    for (uint32_t i = 0 ; i < writeLength ; i++)
    {
        while (m_pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + m_stateMachine)))
        {
            // Busy wait for the TX FIFO to not be full.
        }
        m_pio->txf[m_stateMachine] = pWrite[i];
    }
    for (uint32_t i = 0 ; i < readLength ; i++)
    {
        while (m_pio->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + m_stateMachine)))
        {
            // Busy wait for RX FIFO to not be empty.
        }
        pRead[i] = m_pio->rxf[m_stateMachine];
    }
}

void SWD::switchSwdOutOfDormantMode()
{
    setBitPatternSignalMode();

    const uint32_t selectionAlert[] = { 0x6209F392, 0x86852D95, 0xE3DDAFE9, 0x19BC0EA2 };
    const uint32_t swdActivationCode = 0x1A; // 0b0101_1000 in reverse bit order.
    uint32_t dataToSend[] =
    {
        // Need to send atleast 8 cycles with SWDIO high before selection alert.
        8-1,
        -1UL,
        // Send the 128-bit selection alert.
        128-1,
        selectionAlert[0],
        selectionAlert[1],
        selectionAlert[2],
        selectionAlert[3],
        // Send 4 cycles with SWDIO low.
        4-1,
        0,
        // Send the 8-bit activation code to activate the SWD debug ports from dormant mode.
        8-1,
        swdActivationCode
    };
    writeAndOptionalReadPIO(dataToSend, count_of(dataToSend), NULL, 0);
}

void SWD::switchJtagIntoDormantMode()
{
    setBitPatternSignalMode();

    const uint32_t jtag2dsSelectionSequence = 0x33BBBBBA;
    uint32_t dataToSend[] =
    {
        // Need to send atleast 5 cycles with SWDIO high before sending JTAG to DS sequence.
        5-1,
        -1UL,
        // Send the 31-bit JTAG to DS selection sequence
        31-1,
        jtag2dsSelectionSequence
    };
    writeAndOptionalReadPIO(dataToSend, count_of(dataToSend), NULL, 0);
}

void SWD::switchSwdIntoDormantMode()
{
    setBitPatternSignalMode();

    const uint32_t swd2dsSelectionSequence = 0xE3BC;
    uint32_t dataToSend[] =
    {
        // Need to send atleast 50 cycles with SWDIO high before sending SWD to DS sequence.
        50-1,
        -1UL,
        -1UL,
        // Send the 16-bit SWD to DS selection sequence.
        16-1,
        swd2dsSelectionSequence
    };
    writeAndOptionalReadPIO(dataToSend, count_of(dataToSend), NULL, 0);
}

bool SWD::selectSwdTarget(DPv2Targets target)
{
    assert ( target != UNKNOWN );

    // If this target is already selected then just return.
    if (m_target == target)
    {
        return true;
    }

    // Use sendJtagToSwdSequence() method to switch to DPv1 target instead of using TARGETSEL.
    if (target == DPv1)
    {
        return sendJtagToSwdSequence();
    }

    // Send line reset to SWD ports to prepare them to be selected by a TARGETSEL write.
    sendRawLineReset();

    // Attempt TARGETSEL. Ignore return value as target won't return ack response before selection is completed.
    writeDP(DP_TARGETSEL, target);
    invalidateDpApState();

    // Read the DPIDR to see if there is now a responding debug port.
    bool result = readDPIDR();
    if (!result)
    {
        logError("Failed call to readDPIDR()");
        return false;
    }

    // UNDONE: Might want to stop doing this now as I am going to be calling this function more often on the RP2040.
    // Verify that the target info in the TARGETID and the DLPIDR registers matches the desired target selection.
    uint32_t targetId = 0xBAADFEED;
    result = readDP(DP_TARGETID, &targetId);
    if (!result)
    {
        logError("Failed call to readDP(DP_TARGETID, &targetId)");
        return false;
    }
    uint32_t dlpId = 0xBAADFEED;
    result = readDP(DP_DLPIDR, &dlpId);
    if (!result)
    {
        logError("Failed call to readDP(DP_DLPIDR, &dlpId)");
        return false;
    }

    uint32_t targetInstance = dlpId & (0xF << 28);
    uint32_t targetPartNoAndDesigner = targetId & 0xFFFFFFE;
    uint32_t actualTargetInfo = targetInstance | targetPartNoAndDesigner;
    uint32_t expectedTargetInfo = target & 0xFFFFFFFE;
    if (actualTargetInfo != expectedTargetInfo)
    {
        logErrorF("Failed to verify target info. Expected:0x%08lX Actual:0x%08lX", expectedTargetInfo, actualTargetInfo);
        return false;
    }

    m_target = target;
    return result;
}

bool SWD::searchForKnownSwdTarget()
{
    switchJtagIntoDormantMode();
    switchSwdOutOfDormantMode();

    disableErrorLogging();
    bool result = false;
    for (size_t i = 0 ; i < count_of(g_knownDPv2Targets) ; i++)
    {
        result = selectSwdTarget(g_knownDPv2Targets[i]);
        if (result)
        {
            break;
        }
    }
    enableErrorLogging();

    return result;
}

bool SWD::findFirstSwdTarget(DPv2Targets* pTarget)
{
    // UNDONE: Can start with continuation code of 9, used by Raspberry Pi to speed up search for testing purposes only.
    m_nextTargetDesignerContinuationCode = 0;
    m_nextTargetDesignerIdCode = 1;
    m_nextTargetPartNo = 0;
    m_nextTargetInstance = 0;
    return findNextSwdTarget(pTarget);
}

bool SWD::findNextSwdTarget(DPv2Targets* pTarget)
{
    bool result = false;
    disableErrorLogging();
    while (m_nextTargetDesignerContinuationCode <= 9 && (result = selectSwdTarget(currentTargetId())) == false)
    {
        advanceTargetId(false);
    }
    if (result)
    {
        advanceTargetId(true);
    }
    enableErrorLogging();
    *pTarget = getTarget();
    return result;
}

SWD::DPv2Targets SWD::currentTargetId()
{
    uint32_t targetId = (m_nextTargetInstance << 28) |
                        (m_nextTargetPartNo << 12) |
                        (m_nextTargetDesignerContinuationCode << 8) |
                        (m_nextTargetDesignerIdCode << 1) | 1;
    return (DPv2Targets)targetId;
}

void SWD::advanceTargetId(bool foundTarget)
{
    // Increment instance if we are looking at more than the 0th instance for this particular designer/part_no.
    if (foundTarget || m_nextTargetInstance > 0)
    {
        m_nextTargetInstance++;
    }
    if (m_nextTargetInstance > 0xF)
    {
        m_nextTargetInstance = 0;
    }
    if (m_nextTargetInstance == 0)
    {
        m_nextTargetPartNo++;
    }
    if (m_nextTargetPartNo > 0xFFFF)
    {
        m_nextTargetPartNo = 0;
        m_nextTargetDesignerIdCode++;
    }
    if (m_nextTargetDesignerIdCode > 126)
    {
        m_nextTargetDesignerIdCode = 1;
        m_nextTargetDesignerContinuationCode++;
    }
}

bool SWD::initTargetDpForDebugging(SwdTarget& target)
{
    bool result = controlPower(true, true);
    if (!result)
    {
        logError("Failed to call controlPower(true, true)");
        return false;
    }
    result = initControlStatRegister();
    if (!result)
    {
        logError("Failed to call initControlStatRegister()");
        return false;
    }
    result = setTurnAroundPeriod(1);
    if (!result)
    {
        logError("Failed to call setTurnAroundPeriod(1)");
        return false;
    }
    result = initDpVersion3Registers();
    if (!result)
    {
        logError("Failed to call initDpVersion3Registers()");
        return false;
    }
    result = findDebugMemAP();
    if (!result)
    {
        logError("Failed to call findDebugMemAP()");
        return false;
    }
    result = target.init(this);
    if (!result)
    {
        logError("Failed to call target.init(this)");
        return false;
    }

    return true;
}

bool SWD::initApTarget(SwdTarget& target)
{
    bool result = target.init(this);
    if (!result)
    {
        logError("Failed to call target.init(this)");
        return false;
    }

    return true;
}

bool SWD::controlPower(bool systemPower /* = true */, bool debugPower /* = true */)
{
    // Power up the debug and system modules.
    const uint32_t sysPowerUpReq = 1 << 30;
    const uint32_t dbgPowerUpReq = 1 << 28;
    const uint32_t sysPowerUpAck = 1 << 31;
    const uint32_t dbgPowerUpAck = 1 << 29;
    const uint32_t powerReqBits = sysPowerUpReq | dbgPowerUpReq;
    const uint32_t powerAckBits = sysPowerUpAck | dbgPowerUpAck;
    uint32_t desiredPowerReqBits = (systemPower ? sysPowerUpReq : 0) | (debugPower ? dbgPowerUpReq : 0);
    uint32_t desiredPowerAckBits = (systemPower ? sysPowerUpAck : 0) | (debugPower ? dbgPowerUpAck : 0);
    bool powerUpRequestSent = false;
    bool poweredUp = false;

    absolute_time_t endTime = make_timeout_time_us(m_maxPowerMicroSeconds);
    do
    {
        uint32_t ctrlStat;

        bool result = readDP(DP_CTRL_STAT, &ctrlStat);
        if (!result)
        {
            logError("Failed to call readDP(DP_CTRL_STAT, &ctrlStat)");
            return false;
        }
        if ((ctrlStat & powerAckBits) == desiredPowerAckBits)
        {
            // All powered up so can exit the loop.
            poweredUp = true;
            break;
        }

        if (!powerUpRequestSent)
        {
            ctrlStat &= ~powerReqBits;
            ctrlStat |= desiredPowerReqBits;
            result = writeDP(DP_CTRL_STAT, ctrlStat);
            if (!result)
            {
                logErrorF("Failed to call writeDP(DP_CTRL_STAT, 0x%lX)", ctrlStat);
                return false;
            }

            powerUpRequestSent = true;
        }
    } while (absolute_time_diff_us(get_absolute_time(), endTime) > 0);
    if (!poweredUp)
    {
        logError("TIMEOUT!");
        return false;
    }
    return true;
}

bool SWD::initControlStatRegister()
{
    const uint32_t TRNCNT_Mask = 0xFFF << 12;
    const uint32_t TRNMODE_Mask = 0x1 << 2;
    const uint32_t ERRMODE_Mask = 0x1 << 24;
    const uint32_t ORUNDETECT_Mask = 1 << 0;
    const uint32_t bitsToClear = TRNCNT_Mask | TRNMODE_Mask | ERRMODE_Mask;
    const uint32_t bitsToSet = ORUNDETECT_Mask;

    // Read out the current CONFIG/STAT value.
    uint32_t ctrlStat = 0;
    bool result = readDP(DP_CTRL_STAT, &ctrlStat);
    if (!result)
    {
        logError("Failed to call readDP(DP_CTRL_STAT, &ctrlStat)");
        return false;
    }

    // Calculate the new CTRL/STAT register value by clearing and setting the necessary bits to clear the push operation
    // control bits (TRNCNT & TRNMODE) and the ERRMODE bits and set the ORUNDETECT bit to enable it.
    uint32_t newCtrlStat = (ctrlStat & ~bitsToClear) | bitsToSet;
    if (newCtrlStat == ctrlStat)
    {
        // Already set as requested.
        return true;
    }

    // Write out the updated CTRL/STAT register value.
    result = writeDP(DP_CTRL_STAT, newCtrlStat);
    if (!result)
    {
        logErrorF("Failed to call writeDP(DP_CTRL_STAT, 0x%lX)", newCtrlStat);
    }
    return result;
}

bool SWD::setTurnAroundPeriod(uint32_t cycles)
{
    assert ( cycles > 0 && cycles <= 4 );

    // Read out the current DLCR value.
    uint32_t dlcr = 0;
    bool result = readDP(DP_DLCR, &dlcr);
    if (!result)
    {
        logError("Failed to call readDP(DP_DLCR, &dlcr)");
        return false;
    }

    // Set the TURNAROUND bits to desired cycle count.
    const uint32_t TURNAROUND_Shift = 8;
    const uint32_t TURNAROUND_Mask = 0x3 << TURNAROUND_Shift;
    uint32_t period = cycles - 1;
    dlcr = (dlcr & ~TURNAROUND_Mask) | (period << TURNAROUND_Shift);

    // Write out the updated DLCR register value.
    result = writeDP(DP_DLCR, dlcr);
    if (!result)
    {
        logErrorF("Failed to call writeDP(DP_DLCR, 0x%lX)", dlcr);
    }
    return result;
}

bool SWD::initDpVersion3Registers()
{
    // Assume that AP SELECT address is 8-bits in length unless found to be different in DPIDR1.
    m_apAddressSize = 8;

    if (m_dpVersion < 3)
    {
        // Skip this step if DP version is less than 3.
        return true;
    }

    // Check that AP address size is 32-bit or less and log an error if not but continue along anyway.
    uint32_t dpidr1 = 0;
    bool result = readDP(DP_DPIDR1, &dpidr1);
    if (!result)
    {
        logError("Failed to call readDP(DP_DPIdR1, &dpidr1)");
        return false;
    }

    uint32_t addressSize = dpidr1 & 0x7F;
    if (addressSize > 32)
    {
        // Just log the error and continue execution.
        logErrorF("DPIDR1.ASIZE of %lu was unexpectedly greater than 32", addressSize);
    }

    // The number of upper bits in the SELECT register which are used for selecting the current AP.
    m_apAddressSize = addressSize - 12;

    // Clear the upper 32-bits of the SELECT AP address (SELECT1).
    result = writeDP(DP_SELECT1, 0x0);
    if (!result)
    {
        logError("Failed to call writeDP(DP_SELECT1, 0x0)");
        return false;
    }
    return result;
}

bool SWD::findDebugMemAP()
{
    uint32_t maxAddress = (1 << m_apAddressSize) - 1;
    for (uint32_t i = 0 ; i <= maxAddress ; i++)
    {
        bool result = checkAP(i);
        if (result)
        {
            return true;
        }
    }

    logError("Didn't find MEM-AP.");
    return false;
}

bool SWD::checkAP(uint32_t ap)
{
    bool result = selectAP(ap);
    if (!result)
    {
        logErrorF("Failed to call selectAP(0x%lX)", ap);
        return false;
    }

    // APv1 and APv2 use different AP register banks.
    uint32_t BASE = APv1_BASE;
    uint32_t IDR = APv1_IDR;
    if (m_dpVersion >= 3)
    {
        BASE = APv2_BASE;
        IDR = APv2_IDR;
    }

    // Read in the AP's BASE and IDR values to determine if this is a memory AP connected to a Cortex-M's debug
    // hardware.
    //
    // AP reads are pipelined so throwaway first read result. BASE value will be returned by next read.
    uint32_t dummy;
    result = readAP(BASE, &dummy);
    if (!result)
    {
        logError("Failed to call readAP(BASE, &dummy)");
        return false;
    }
    uint32_t base = 0xBAADFEED;
    result = readAP(IDR, &base);
    if (!result)
    {
        logError("Failed to call readAP(IDR, &base)");
        return false;
    }
    uint32_t id = 0xBAADFEED;
    result = readDP(DP_RDBUFF, &id);
    if (!result)
    {
        logError("Failed to call readDP(DP_RDBUFF, &id)");
        return false;
    }

    // The Cortex-M ROM table starts at 0xE00FF000. The least significant bits also need to be set, with bit 0
    // indicating whether it is a debug entry.
    const uint32_t expectedBaseValue = 0xE00FF003;
    // The Class bits in the IDR should indicate that this is a MEM-AP.
    const uint32_t idrClassBitmask = 0xF << 13;
    const uint32_t idrMemApClass = 0x8 << 13;
    if ((id & idrClassBitmask) != idrMemApClass || base != expectedBaseValue)
    {
        return false;
    }

    return true;
}

uint32_t SWD::readTargetMemory(uint32_t address, void* pvBuffer, uint32_t bufferSize, TransferSize readSize)
{
    uint32_t totalBytesRead = 0;
    uint32_t bytesLeft = bufferSize;
    uint8_t* pBuffer = (uint8_t*)pvBuffer;
    uint32_t retryCount = 0;
    while (bytesLeft > 0)
    {
        uint32_t bytesRead = readTargetMemoryInternal(address, pBuffer, bytesLeft, readSize);
        assert ( bytesRead % (readSize / 8) == 0 );
        if (bytesRead == 0)
        {
            if (getLastReadWriteError() == SWD_FAULT_ERROR)
            {
                // This typically means that GDB tried to access an invalid memory location so don't bother to try
                // again.
                logInfoF("readTargetMemoryInternal() failed to read from address 0x%08lX.", address);
                return totalBytesRead;
            }
            else if (retryCount < m_maxReadRetries)
            {
                logError("readTargetMemoryInternal() returned 0 bytes. Retrying!");
                retryCount++;
                m_totalMemoryReadRetries++;
                continue;
            }
            else
            {
                logError("readTargetMemoryInternal() returned 0 bytes. Hit maximum retry");
                return totalBytesRead;
            }
        }

        retryCount = 0;
        address += bytesRead;
        pBuffer += bytesRead;
        bytesLeft -= bytesRead;
        totalBytesRead += bytesRead;
    }

    return totalBytesRead;
}

uint32_t SWD::readTargetMemoryInternal(uint32_t address, uint8_t* pDest, uint32_t bufferSize, TransferSize readSize)
{
    uint32_t sizeInBytes = readSize / 8;
    assert ( pDest != NULL );
    assert ( bufferSize % sizeInBytes == 0 );
    assert ( address % sizeInBytes == 0 );

    // APv1 and APv2 use different AP register banks.
    uint32_t TAR = APv1_TAR;
    uint32_t DRW = APv1_DRW;
    if (m_dpVersion >= 3)
    {
        TAR = APv2_TAR;
        DRW = APv2_DRW;
    }

    // Have the read stop at the next 1k limit and let the caller deal with starting the next 1k chunk.
    // This is done because the auto-incrementing on the TAR may wrap at 10-bits.
    uint32_t roundUp = (address + 1024) & ~(1024-1);
    uint32_t bytesToNextChunk = roundUp - address;
    if (bufferSize > bytesToNextChunk)
    {
        bufferSize = bytesToNextChunk;
    }

    // Setup for auto-incrementing AP read accesses of the correct size.
    bool result = updateCSW(ADDR_INC_SINGLE_ENABLED, readSize);
    if (!result)
    {
        logErrorF("Failed to call updateCSW(ADDR_INC_SINGLE_ENABLED, %lu)", readSize);
        return 0;
    }
    // Set the starting address in the TAR.
    result = writeAP(TAR, address);
    if (!result)
    {
        logErrorF("Failed to call writeAP(TAR, 0x%08lx)", address);
        return 0;
    }

    uint32_t bytesRead = 0;
    uint32_t lastRead = bufferSize - sizeInBytes;
    bool dummyRead = true;
    while (bytesRead < bufferSize)
    {
        uint32_t curr;
        if (dummyRead || bytesRead < lastRead)
        {
            result = readAP(DRW, &curr);
        }
        else
        {
            result = readDP(DP_RDBUFF, &curr);
        }
        if (!result)
        {
            if (m_lastReadWriteError != SWD_FAULT_ERROR)
            {
                // Silencing errors on access violations and only reporting higher up the call stack.
                logError("Failed to call readAP/readDP()");
            }
            return bytesRead;
        }

        if (!dummyRead)
        {
            // Reads are delayed by one transfer so this data is from the previous read address.
            uint32_t prevAddress = address - sizeInBytes;
            switch (readSize)
            {
                case TRANSFER_8BIT:
                    *pDest = (curr >> (8 * (prevAddress & 0x3))) & 0xFF;
                    break;
                case TRANSFER_16BIT:
                    *(uint16_t*)pDest = (curr >> (8 * (prevAddress & 0x2))) & 0xFFFF;
                    break;
                case TRANSFER_32BIT:
                    *(uint32_t*)pDest = curr;
                    break;
                default:
                    return 0;
            }
            pDest += sizeInBytes;
            bytesRead += sizeInBytes;
        }
        dummyRead = false;

        address += sizeInBytes;
    }
    assert ( bytesRead == bufferSize );
    return bytesRead;
}

uint32_t SWD::writeTargetMemory(uint32_t address, const void* pvBuffer, uint32_t bufferSize, TransferSize writeSize)
{
    uint32_t totalBytesWritten = 0;
    uint32_t bytesLeft = bufferSize;
    const uint8_t* pBuffer = (const uint8_t*)pvBuffer;
    uint32_t retryCount = 0;
    while (bytesLeft > 0)
    {
        uint32_t bytesWritten = writeTargetMemoryInternal(address, pBuffer, bytesLeft, writeSize);
        assert ( bytesWritten % (writeSize / 8) == 0 );
        if (bytesWritten == 0)
        {
            if (getLastReadWriteError() == SWD_FAULT_ERROR)
            {
                // This typically means that GDB tried to access an invalid memory location so don't bother to try
                // again.
                logInfoF("writeTargetMemoryInternal() failed to write to address 0x%08lX.", address);
                return totalBytesWritten;
            }
            else if (retryCount < m_maxWriteRetries)
            {
                logError("Write returned 0 bytes. Retrying!");
                retryCount++;
                m_totalMemoryWriteRetries++;
                continue;
            }
            else
            {
                logError("Write returned 0 bytes. Hit maximum retry");
                return totalBytesWritten;
            }
        }

        retryCount = 0;
        address += bytesWritten;
        pBuffer += bytesWritten;
        bytesLeft -= bytesWritten;
        totalBytesWritten += bytesWritten;
    }

    return totalBytesWritten;
}

uint32_t SWD::writeTargetMemoryInternal(uint32_t address, const uint8_t* pSrc, uint32_t bufferSize, TransferSize writeSize)
{
    uint32_t sizeInBytes = writeSize / 8;

    assert ( pSrc != NULL );
    assert ( bufferSize % sizeInBytes == 0 );
    assert ( address % sizeInBytes == 0 );

    // APv1 and APv2 use different AP register banks.
    uint32_t TAR = APv1_TAR;
    uint32_t DRW = APv1_DRW;
    if (m_dpVersion >= 3)
    {
        TAR = APv2_TAR;
        DRW = APv2_DRW;
    }

    // Have the write stop at the next 1k limit and let the caller deal with starting the next 1k chunk.
    // This is done because the auto-incrementing on the TAR may wrap at 10-bits.
    uint32_t roundUp = (address + 1024) & ~(1024-1);
    uint32_t bytesToNextChunk = roundUp - address;
    if (bufferSize > bytesToNextChunk)
    {
        bufferSize = bytesToNextChunk;
    }

    // Setup for auto-incrementing AP accesses of the correct size.
    bool result = updateCSW(ADDR_INC_SINGLE_ENABLED, writeSize);
    if (!result)
    {
        logErrorF("Failed to call updateCSW(ADDR_INC_SINGLE_ENABLED, %lu)", writeSize);
        return 0;
    }
    // Set the starting address in the TAR.
    result = writeAP(TAR, address);
    if (!result)
    {
        logErrorF("Failed to call writeAP(TAR, 0x%08lX)", address);
        return 0;
    }

    uint32_t bytesPended = 0;
    uint32_t startAddress = address;
    while (bytesPended < bufferSize)
    {
        uint32_t drwVal;
        switch (writeSize)
        {
            case TRANSFER_8BIT:
                drwVal = *pSrc << (8 * (address & 0x3));
                break;
            case TRANSFER_16BIT:
                drwVal = *(uint16_t*)pSrc << (8 * (address & 0x2));
                break;
            case TRANSFER_32BIT:
                drwVal = *(uint32_t*)pSrc;
                break;
            default:
                return 0;
        }
        result = writeAP(DRW, drwVal);
        if (!result)
        {
            if (m_lastReadWriteError != SWD_FAULT_ERROR)
            {
                // Silencing errors on access violations and only reporting higher up the call stack.
                logErrorF("Failed to call writeAP(DRW, 0x%lX)", drwVal);
            }
            return calculateTransferCount(startAddress, address);
        }
        pSrc += sizeInBytes;
        bytesPended += sizeInBytes;
        address += sizeInBytes;
    }
    return calculateTransferCount(startAddress, address);
}

bool SWD::updateCSW(CSW_AddrIncs addrInc, TransferSize transferSize)
{
    CSW_Sizes cswSize;
    switch (transferSize)
    {
        case TRANSFER_8BIT:
            cswSize = CSW_SIZE_8BIT;
            break;
        case TRANSFER_16BIT:
            cswSize = CSW_SIZE_16BIT;
            break;
        case TRANSFER_32BIT:
            cswSize = CSW_SIZE_32BIT;
            break;
        default:
            return false;
    }

    // APv1 and APv2 use different AP register banks.
    uint32_t CSW = APv1_CSW;
    if (m_dpVersion >= 3)
    {
        CSW = APv2_CSW;
    }

    // Bitmasks for fields to be set in the CSW register.
    const uint32_t addrIncMask = 3 << 4;
    const uint32_t sizeMask = 7 << 0;
    const uint32_t protMask = 0x7F << 24;
    const uint32_t protAddrIncAndSizeMask = protMask | addrIncMask | sizeMask;

    // PROT value of 0x23 means:
    //      Requester ID for the AHB-AP
    //      Data Transfer
    //      Privileged Transfer
    const uint32_t protValue = 0x23;
    uint32_t       bitsToUpdate = (protValue << 24) | (addrInc << 4) | cswSize;
    if (m_cswValid && (m_csw & protAddrIncAndSizeMask) == bitsToUpdate)
    {
        // The CSW bits are already set to the needed value.
        return true;
    }

    uint32_t csw;
    if (m_cswValid)
    {
        csw = m_csw;
    }
    else
    {
        uint32_t dummy;
        bool result = readAP(CSW, &dummy);
        if (!result)
        {
            logError("Failed to call readAP(CSW, &dummy)");
            return false;
        }
        result = readDP(DP_RDBUFF, &csw);
        if (!result)
        {
            logError("Failed to call readDP(DP_RDBUFF, &csw)");
            return false;
        }
    }
    csw = (csw & ~protAddrIncAndSizeMask) | bitsToUpdate;
    bool result = writeAP(CSW, csw);
    if (!result)
    {
        logErrorF("Failed to call writeAP(CSW, 0x%08lX)", csw);
        return false;
    }
    // UNDONE: How do I make sure that this AP write has completed?

    m_csw = csw;
    m_cswValid = true;
    return true;
}

uint32_t SWD::calculateTransferCount(uint32_t startAddress, uint32_t expectedAddress)
{
    // APv1 and APv2 use different AP register banks.
    uint32_t TAR = APv1_TAR;
    if (m_dpVersion >= 3)
    {
        TAR = APv2_TAR;
    }

    uint32_t dummy;
    bool result = readAP(TAR, &dummy);
    if (!result)
    {
        logError("Failed to call readAP(TAR, &dummy)");
        return 0;
    }
    uint32_t tar = 0;
    result = readDP(DP_RDBUFF, &tar);
    if (!result)
    {
        logError("Failed to call readDP(DP_RDBUFF, &tar)");
        return 0;
    }

    // Handle the case where the auto increment wraps around at 1k or larger intervals.
    if (tar < expectedAddress && ((expectedAddress - tar) & (1024-1)) == 0)
    {
        tar = expectedAddress;
    }

    if (tar < startAddress)
    {
        logErrorF("Failed to read appropriate value from TAR. 0x%08X is less than 0x%08X", tar, startAddress);
        return 0;
    }
    return tar - startAddress;
}







// Error bits that can be set in CTRL/STAT register.
static const uint32_t STAT_STICKY_OVERRUN_BIT = 1 << 1;
static const uint32_t STAT_STICKY_COMPARE_BIT = 1 << 4;
static const uint32_t STAT_STICKY_ERROR_BIT = 1 << 5;
static const uint32_t STAT_WRITE_DATA_ERROR_BIT = 1 << 7;

// Error bits that can be cleared in the ABORT register.
static const uint32_t ABORT_OVERRUN_ERROR_CLEAR_BIT = 1 << 4;
static const uint32_t ABORT_WRITE_DATA_ERROR_CLEAR_BIT = 1 << 3;
static const uint32_t ABORT_STICKY_ERROR_CLEAR_BIT  = 1 << 2;
static const uint32_t ABORT_STICKY_COMPARE_CLEAR_BIT = 1 << 1;

bool SWD::clearAbortErrorBits()
{
    uint32_t bitsToClear =
        ABORT_OVERRUN_ERROR_CLEAR_BIT | ABORT_WRITE_DATA_ERROR_CLEAR_BIT |
        ABORT_STICKY_ERROR_CLEAR_BIT | ABORT_STICKY_COMPARE_CLEAR_BIT;
    bool result = writeDP(DP_ABORT, bitsToClear);
    if (!result)
    {
        logErrorF("Failed to call writeDP(DP_ABORT, 0x%08lX)", bitsToClear);
    }
    return result;
}

void SWD::setBitPatternSignalMode()
{
    if (m_signalMode == BIT_PATTERN)
    {
        return;
    }
    restartStateMachineInSignalMode();
}

void SWD::setPacketSignalMode()
{
    if (m_signalMode == PACKET)
    {
        return;
    }

    // Sending a bit pattern length of 0 causes the PIO state machine to switch into packet mode.
    pio_sm_put_blocking(m_pio, m_stateMachine, 0);
    m_signalMode = PACKET;
}

void SWD::restartStateMachineInSignalMode()
{
    // UNDONE: Could move the conditional into its own routine.
    // Wait for state machine to finish sending last packet.
    uint8_t pioWaitingForPacket = m_programOffset + swd_offset_packet_start;
    while (!pio_sm_is_tx_fifo_empty(m_pio, m_stateMachine) || pio_sm_get_pc(m_pio, m_stateMachine) != pioWaitingForPacket)
    {
    }
    assert ( pio_sm_is_rx_fifo_empty(m_pio, m_stateMachine) );

    // Start running the state machine from the beginning so that it will accept long bit sequences.
    pio_sm_set_enabled(m_pio, m_stateMachine, false);
        pio_sm_restart(m_pio, m_stateMachine);
        pio_sm_exec(m_pio, m_stateMachine, pio_encode_jmp(m_programOffset));
    pio_sm_set_enabled(m_pio, m_stateMachine, true);
    m_signalMode = BIT_PATTERN;
}

bool SWD::readDPIDR()
{
    bool result = readDP(DP_DPIDR, &m_dpidr);
    if (!result)
    {
        logError("Failed to call readDP(DP_DPIDR, &m_dpidr)");
        m_dpidr = UNKNOWN_VAL;
        m_dpVersion = 0;
        return false;
    }

    // Extract the DP version as versions >= 3 function a bit differently (including AP upgrade to version 2).
    m_dpVersion = (m_dpidr >> 12) & 0xF;

    // Clear any sticky error bits from previous interactions with this target.
    clearAbortErrorBits();
    return result;
}

void SWD::idleBus(uint32_t cyclesToIdle)
{
    setBitPatternSignalMode();

    assert ( cyclesToIdle >= 8 && cyclesToIdle <= 0x7FFFFFFF );
    uint32_t dataToSend[] =
    {
        cyclesToIdle - 1,
        0
    };
    writeAndOptionalReadPIO(dataToSend, count_of(dataToSend), NULL, 0);
    int32_t bitsLeft = (int32_t)cyclesToIdle - 32;
    while (bitsLeft > 0)
    {
        uint32_t wordToSend = 0;
        writeAndOptionalReadPIO(&wordToSend, 1, NULL, 0);
        bitsLeft -= 32;
    }
}


// 3-bit responses that can be returned by the target to packet requests.
static const uint32_t ACK_OK = 1 << 0;
static const uint32_t ACK_WAIT = 1 << 1;
static const uint32_t ACK_FAIL = 1 << 2;

// If all bits are high in target response then it didn't respond.
static const uint32_t ACK_PROT_ERROR = 7;
// Internal code used to flag a parity error.
static const uint32_t ACK_PARITY = 1 << 3;


bool SWD::read(SwdApOrDp APnDP, uint32_t address, uint32_t* pData)
{
    // Make sure that we are allowed to read from this register.
    assert ( address & AP_DP_RO );

    if (APnDP == AP)
    {
        if (m_apWritePending)
        {
            // If last AP request was a write then delay for a bit before issuing the first read.
            idleBus(8);
        }
        m_apWritePending = false;
    }

    if (!selectBank(APnDP, address))
    {
        logErrorF("Failed to call selectBank(0x%lX, 0x%lX)", APnDP, address);
        return false;
    }

    setPacketSignalMode();
    uint32_t request = buildPacketRequest(APnDP, READ, address);
    absolute_time_t endTime = make_timeout_time_us(m_maxReadMicroseconds);
    do
    {
        uint32_t responses[3];
        writeAndOptionalReadPIO(&request, 1, responses, count_of(responses));

        // Read out the ACK, value, and parity.
        uint32_t ack = responses[0];
        uint32_t data = responses[1];
        uint32_t actualParity = responses[2];

        if (ack == ACK_OK && calculateParity(data) != actualParity)
        {
            ack = ACK_PARITY;
        }

        bool retryTransfer = false;
        bool result = handleTransferResponse(ack, address == DP_DPIDR, &retryTransfer);
        if (!result)
        {
            if (retryTransfer)
            {
                continue;
            }
            if (m_lastReadWriteError == SWD_FAULT_ERROR)
            {
                // Can silence these error messages by turning off debug level diagnostics in config.h
                logDebugF("Failed call to handleTransferResponse(0x%lX, %s, &retryTransfer)",
                          ack, address == DP_DPIDR ? "true" : "false");
            }
            else if (m_lastReadWriteError != SWD_PROTOCOL || address != DP_DPIDR)
            {
                logErrorF("Failed call to handleTransferResponse(0x%lX, %s, &retryTransfer)",
                        ack, address == DP_DPIDR ? "true" : "false");
            }
            return false;
        }
        *pData = data;
        return true;
    } while (absolute_time_diff_us(get_absolute_time(), endTime) > 0);

    // Get here if timeout has been encountered.
    logError("TIMEOUT!");
    return false;
}

bool SWD::write(SwdApOrDp APnDP, uint32_t address, uint32_t data)
{
    // Make sure that we are allowed to write to this register.
    assert ( address & AP_DP_WO );

    if (APnDP == AP)
    {
        // Remember that we just pended an AP write.
        m_apWritePending = true;
    }

    if (!selectBank(APnDP, address))
    {
        logErrorF("Failed to call selectBank(0x%lX, 0x%08lX)", APnDP, address);
        return false;
    }

    setPacketSignalMode();
    uint32_t request = buildPacketRequest(APnDP, WRITE, address);
    absolute_time_t endTime = make_timeout_time_us(m_maxWriteMicroseconds);
    do
    {
        uint32_t dataToSend[] =
        {
            request,
            data,
            calculateParity(data)
        };
        uint32_t ack;
        writeAndOptionalReadPIO(dataToSend, count_of(dataToSend), &ack, 1);

        bool retryTransfer = false;
        bool result = handleTransferResponse(ack, address == DP_TARGETSEL, &retryTransfer);
        if (!result)
        {
            if (retryTransfer)
            {
                continue;
            }
            if (m_lastReadWriteError == SWD_FAULT_ERROR)
            {
                // Can silence these error messages by turning off debug level diagnostics in config.h
                logDebugF("Failed to call handleTransferResponse(0x%lX, %s, &retryTransfer)",
                          ack, address == DP_TARGETSEL ? "true" : "false");
            }
            else if (m_lastReadWriteError != SWD_PROTOCOL || address != DP_TARGETSEL)
            {
                logErrorF("Failed to call handleTransferResponse(0x%lX, %s, &retryTransfer)",
                        ack, address == DP_TARGETSEL ? "true" : "false");
            }
            return false;
        }
        return true;
    } while (absolute_time_diff_us(get_absolute_time(), endTime) > 0);

    // Get here if timeout has been encountered.
    logError("TIMEOUT!");
    return false;
}

bool SWD::handleTransferResponse(uint32_t ack, bool ignoreProtocolError, bool* pRetry)
{
    if (ack != ACK_OK && ack != ACK_WAIT && ack != ACK_FAIL && ack != ACK_PARITY)
    {
        // Any unrecognized response will be treated as a protocol error.
        ack = ACK_PROT_ERROR;
    }

    if (ack != ACK_OK && m_clearingErrors > 0)
    {
        // Just return error code with no retries if already trying to handle errors.
        logErrorF("Ignoring failure 0x%lX while handling ACK_FAIL.", ack);
        return false;
    }
    else if (ack == ACK_WAIT)
    {
        // Clear sticky overrun error and retry on wait responses.
        handleStickyErrors(ack, pRetry);
        m_totalWaitRetries++;
        return false;
    }
    else if (ack == ACK_PROT_ERROR)
    {
        if (!ignoreProtocolError)
        {
            // Send line reset and retry.
            logError("Encountered protocol error.");
            handleProtocolAndParityErrors();
            m_totalProtocolErrorRetries++;
            *pRetry = true;
        }
        m_lastReadWriteError = SWD_PROTOCOL;
        return false;
    }
    else if (ack == ACK_FAIL)
    {
        handleStickyErrors(ack, pRetry);
        m_totalFailErrors++;
        return false;
    }
    else if (ack == ACK_PARITY)
    {
        // Send line reset and retry.
        logError("Encountered parity error.");
        handleProtocolAndParityErrors();
        m_totalParityErrorRetries++;
        *pRetry = true;
        m_lastReadWriteError = SWD_PARITY;
        return false;;
    }

    return true;
}

void SWD::handleStickyErrors(uint32_t ack, bool* pRetry)
{
    // This bit will be cleared in DP_CTRL_STAT if something has caused the debug port to be powered down.
    const uint32_t DP_CTRL_STAT_CDBGPWRUPACK_Bit = 1 << 29;

    // Flag that we are handling FAIL/WAIT so that we don't infinite loop trying to clear error bits.
    m_clearingErrors++;
        uint32_t stat = 0;
        if (!readDP(DP_CTRL_STAT, &stat))
        {
            // Try a line reset if the first step to clear the sticky error fails and request a retry.
            logError("Failed to readDP(DP_CTRL_STAT, &stat). Attempting to clear error with line reset.");
            handleProtocolAndParityErrors();
            m_lastReadWriteError = SWD_FAULT_WDATAERR;
            *pRetry = true;
            m_clearingErrors--;
            return;
        }

        // Clear the sticky errors via the ABORT register.
        uint32_t abortBitsToClear = 0;
        if (stat & STAT_STICKY_OVERRUN_BIT)
        {
            abortBitsToClear |= ABORT_OVERRUN_ERROR_CLEAR_BIT;
            // Set this error code first since the other 2 have higher precedence and should overwrite this one.
            m_lastReadWriteError = SWD_FAULT_OVERRUN;
            *pRetry = true;
        }
        if (stat & STAT_WRITE_DATA_ERROR_BIT)
        {
            // Treat the same as read parity errors.
            handleProtocolAndParityErrors();
            m_totalParityErrorRetries++;
            *pRetry = true;
            abortBitsToClear |= ABORT_WRITE_DATA_ERROR_CLEAR_BIT;
            m_lastReadWriteError = SWD_FAULT_WDATAERR;
        }
        if (stat & STAT_STICKY_ERROR_BIT)
        {
            abortBitsToClear |= ABORT_STICKY_ERROR_CLEAR_BIT;
            // Set this error code last as it is the most important one for the caller to know about.
            m_lastReadWriteError = SWD_FAULT_ERROR;
        }
        if (!(stat & DP_CTRL_STAT_CDBGPWRUPACK_Bit))
        {
            // Debug port has powered down for some reason, probably external reset.
            *pRetry = false;
            m_lastReadWriteError = SWD_PROTOCOL;
        }
        if (m_lastReadWriteError == SWD_FAULT_ERROR)
        {
            // Can silence these error messages by turning off debug level diagnostics in config.h
            logDebugF("Encountered %s w/ DP_CTRL_STAT=0x%08lX.", ack == ACK_FAIL ? "ACK_FAIL" : "ACK_WAIT", stat);
        }
        else
        {
            logErrorF("Encountered %s w/ DP_CTRL_STAT=0x%08lX.", ack == ACK_FAIL ? "ACK_FAIL" : "ACK_WAIT", stat);
        }
        writeDP(DP_ABORT, abortBitsToClear);
    m_clearingErrors--;
}

bool SWD::selectBank(SwdApOrDp APnDP, uint32_t address)
{
    bool result = false;

    if (APnDP == DP)
    {
        result = selectDpBank(address);
        if (!result)
        {
            logErrorF("Failed call to selectDpBank(0x%lX)", address);
        }
        return result;
    }
    else if (APnDP == AP)
    {
        result = selectApBank(address);
        if (!result)
        {
            logErrorF("Failed call to selectApBank(0x%lX)", address);
        }
        return result;
    }
    else
    {
        assert ( APnDP == DP || APnDP == AP );
        return false;
    }
}

bool SWD::selectDpBank(uint32_t address)
{
    uint32_t registerIndex = address & 0xF;
    uint32_t registerBank = (address >> 4) & 0xF;
    if (registerIndex == 0x8 || registerIndex == 0xC || (m_dpVersion < 3 && registerIndex == 0x0) || m_dpBank == registerBank)
    {
        // DP bank only matters for register 4 for DP versions less than 3 and for registers 4 AND 0 in subsequent DP
        // versions.
        // Only need to set the bank if not already set.
        return true;
    }

    m_dpBank = registerBank;
    bool result = updateSelect();
    if (!result)
    {
        logError("Failed call to updateSelect()");
        return false;
    }
    return true;
}

bool SWD::selectApBank(uint32_t address)
{
    uint32_t registerBank = (address >> 4) & 0xF;
    if (m_dpVersion >= 3)
    {
        // AP bank is 8-bit in DPv3/APv2.
        registerBank = (address >> 4) & 0xFF;
    }

    if (m_apBank == registerBank)
    {
        // Only need to set the bank if not already set.
        return true;
    }

    m_apBank = registerBank;
    bool result = updateSelect();
    if (!result)
    {
        logError("Failed call to updateSelect()");
        return false;
    }
    return true;
}

bool SWD::selectAP(uint32_t ap)
{
    if (m_ap == ap)
    {
        // Just return if the AP is already set to this value.
        return true;
    }

    // Don't call updateSelect() at this time and wait for the next DP or AP register read/write. Setting m_dpBank and
    // m_apBank to UNKNOWN_VAL will make sure that updateSelect() gets called on that next read/write.
    m_dpBank = UNKNOWN_VAL;
    m_apBank = UNKNOWN_VAL;
    m_cswValid = false;
    m_ap = ap;

    return true;
}

bool SWD::updateSelect()
{
    // Give unknown AP/DP indices valid values before setting into SELECT register.
    if (m_dpBank == UNKNOWN_VAL)
    {
        m_dpBank = 0;
    }
    if (m_apBank == UNKNOWN_VAL)
    {
        m_apBank = 0;
    }
    assert ( m_dpBank <= 0xF );
    assert ( (m_dpVersion < 3 && m_ap <= 0xFF)    || (m_dpVersion >= 3 && m_ap <= 0xFFFFF) );
    assert ( (m_dpVersion < 3 && m_apBank <= 0xF) || (m_dpVersion >= 3 && m_apBank <= 0xFF) );

    uint32_t dpBankSel = m_dpBank & 0xF;
    uint32_t apBankSel = m_apBank & 0xF;
    uint32_t apSel = m_ap & 0xFF;
    uint32_t selectValue = (apSel << 24) | (apBankSel << 4) | dpBankSel;
    if (m_dpVersion >= 3)
    {
        // The SELECT register has a different layout in DPv3 to allow for more AP register addresses.
        apBankSel = m_apBank & 0xFF;
        apSel = m_ap & 0xFFFFF;
        selectValue = (apSel << 12) | (apBankSel << 4) | dpBankSel;
    }
    bool result = writeDP(DP_SELECT, selectValue);
    if (!result)
    {
        logErrorF("Failed call to writeDP(DP_SELECT, 0x%lX)", selectValue);
        m_dpBank = UNKNOWN_VAL;
        m_apBank = UNKNOWN_VAL;
        return false;
    }
    return true;
}

uint32_t SWD::buildPacketRequest(SwdApOrDp APnDP, SwdReadOrWrite RnW, uint8_t address)
{
    // Only bits 2 and 3 are specified by address. Bits 0 and 1 are always 0.
    assert ( (address & 0x3) == 0 );

    const uint32_t startBit = 1 << 0;
    uint32_t dpAccessBit = (APnDP & 1) << 1;
    uint32_t readBit = (RnW & 1) << 2;
    uint32_t addressBits = ((address >> 2) & 0x3) << 3;
    // Parity, calculated below, goes in bit 5.
    const uint32_t stopBit = 0 << 6;
    const uint32_t parkBit = 1 << 7;
    // Repeat the RnW bit at bit offset 8 for PIO code usage only (not sent to target).
    uint32_t readCodePathBit = (RnW & 1) << 8;

    // Calculate parity over the request bits which are covered by it.
    uint32_t bitsForParity = dpAccessBit | readBit | addressBits;
    uint32_t parityBit = calculateParity(bitsForParity) << 5;

    return startBit | bitsForParity | parityBit | stopBit | parkBit | readCodePathBit;
}

uint32_t SWD::calculateParity(uint32_t bitsForParity)
{
    uint32_t parity = 0;
    for (int i = 0 ; i < 32 ; i++)
    {
        parity ^= bitsForParity & 1;
        bitsForParity >>= 1;
    }
    return parity;
}

void SWD::handleProtocolAndParityErrors()
{
    // UNDONE: Could drop the SWD clock rate here too.
    m_clearingErrors++;
        if (m_target == DPv1 || m_target == UNKNOWN)
        {
            // Line reset is enough for DPv1 targets.
            sendLineReset();
            m_target = DPv1;
        }
        else
        {
            // Need to resend TARGETSEL after line reset for DPv2 targets.
            DPv2Targets origTarget = m_target;
            m_target = UNKNOWN;
            selectSwdTarget(origTarget);
        }
    m_clearingErrors--;
}




bool SwdTarget::init(SWD* pSWD)
{
    m_pSWD = pSWD;
    m_target = pSWD->getTarget();
    m_ap = pSWD->m_ap;
    m_dpidr = pSWD->getDPIDR();
    return fetchTargetDetails();
}


bool SwdTarget::fetchTargetDetails()
{
    // Read the peripheral and components IDs from the target.
    uint32_t bytesRead = readMemory(0xE00FFFD0,
                                    m_peripheralComponentIDs, sizeof(m_peripheralComponentIDs), SWD::TRANSFER_32BIT);
    if (bytesRead < sizeof(m_peripheralComponentIDs))
    {
        logError("Failed to call readTargetMemory(0xE00FFFD0, ...)");
        return false;
    }
    logDebug("peripheralComponentIDs[]=");
    logDebug("{");
    for (size_t i = 0 ; i < count_of(m_peripheralComponentIDs) ; i++)
    {
        logDebugF("    0x%08lX ", m_peripheralComponentIDs[i]);
    }
    logDebug("}");

    // Read in the CPUID for the Cortex-M processor as well.
    bytesRead = readMemory(0xE000ED00, &m_cpuID, sizeof(m_cpuID), SWD::TRANSFER_32BIT);
    if (bytesRead != sizeof(m_cpuID))
    {
        logError("Failed to call readTargetMemory(0xE000ED00, &m_cpuID, ...)");
        return false;
    }
    logDebugF("CPUID=0x%08lX", m_cpuID);

    determineCpuType();
    logDebugF("Detected CPU type = %s", getCpuTypeString(m_cpu));

    return true;
}

void SwdTarget::determineCpuType()
{
    // Check component/peripheral ID for known Cortex-M CPUs.
    const uint32_t armComponentID[4] = { 0x0000000D, 0x00000010, 0x00000005, 0x000000B1 };
    struct
    {
        uint32_t      peripheralID[8];
        SWD::CpuTypes type;
    } const knownPeripheralIDs[] =
    {
        {
            {
                0x00000004, 0x00000000, 0x00000000, 0x00000000,
                0x00000071, 0x000000B4, 0x0000000B, 0x00000000
            },
            SWD::CPU_CORTEX_M0
        },
        {
            {
                0x00000004, 0x00000000, 0x00000000, 0x00000000,
                0x000000C0, 0x000000B4, 0x0000000B, 0x00000000
            },
            SWD::CPU_CORTEX_M0_PLUS
        },
        {
            {
                0x00000004, 0x00000000, 0x00000000, 0x00000000,
                0x0000000C, 0x000000B0, 0x0000000B, 0x00000000
            },
            SWD::CPU_CORTEX_M3
        },
        {
            {
                0x00000004, 0x00000000, 0x00000000, 0x00000000,
                0x000000C4, 0x000000B4, 0x0000000B, 0x00000000
            },
            SWD::CPU_CORTEX_M4
        },
        {
            {
                0x00000004, 0x00000000, 0x00000000, 0x00000000,
                0x000000C8, 0x000000B4, 0x0000000B, 0x00000000
            },
            SWD::CPU_CORTEX_M7
        },
        {
            {
                0x00000004, 0x00000000, 0x00000000, 0x00000000,
                0x000000C9, 0x000000B4, 0x0000000B, 0x00000000
            },
            SWD::CPU_CORTEX_M33
        },
    };
    m_cpu = SWD::CPU_UNKNOWN;
    if (0 == memcmp(&m_peripheralComponentIDs[8], &armComponentID[0], sizeof(armComponentID)))
    {
        for (size_t i = 0 ; i < count_of(knownPeripheralIDs) ; i++)
        {
            if (0 == memcmp(&m_peripheralComponentIDs[0], &knownPeripheralIDs[i].peripheralID[0], sizeof(knownPeripheralIDs[i].peripheralID)))
            {
                m_cpu = knownPeripheralIDs[i].type;
                return;
            }
        }
    }

    // If peripheral ID isn't recognized then try using the CPU ID instead.
    struct
    {
        uint32_t      cpuID;
        SWD::CpuTypes type;
    } const knownCpuIds[] =
    {
        {
            0x410CC200,
            SWD::CPU_CORTEX_M0
        },
        {
            0x410CC600,
            SWD::CPU_CORTEX_M0_PLUS
        },
        {
            0x412FC230,
            SWD::CPU_CORTEX_M3
        },
        {
            0x410FC240,
            SWD::CPU_CORTEX_M4
        },
        {
            0x410C270,
            SWD::CPU_CORTEX_M7
        },
        {
            0x411FD210,
            SWD::CPU_CORTEX_M33
        },
    };
    const uint32_t CPUID_REVISION_MASK = 0xF;
    uint32_t cpuID = m_cpuID & ~CPUID_REVISION_MASK;
    for (size_t i = 0 ; i < count_of(knownCpuIds) ; i++)
    {
        if (knownCpuIds[i].cpuID == cpuID)
        {
            m_cpu = knownCpuIds[i].type;
            return;
        }
    }
}

const char* SwdTarget::getCpuTypeString(SWD::CpuTypes cpu)
{
    switch (cpu)
    {
        case SWD::CPU_CORTEX_M0:
            return "Cortex-M0";
        case SWD::CPU_CORTEX_M0_PLUS:
            return "Cortex-M0+";
        case SWD::CPU_CORTEX_M3:
            return "Cortex-M3";
        case SWD::CPU_CORTEX_M4:
            return "Cortex-M4";
        case SWD::CPU_CORTEX_M7:
            return "Cortex-M7";
        case SWD::CPU_CORTEX_M33:
            return "Cortex-M33";
        default:
            return "Unknown";
    }
}
