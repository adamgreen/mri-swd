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
#ifndef SWD_H_
#define SWD_H_

#include <string.h>
#include <pico/stdlib.h>
#include <hardware/pio.h>

class SwdTarget;

class SWD
{
    friend SwdTarget;

    public:
        // Enumeration of DPv2 SWD targets supported by this debugger.
        // Target items followed by '*' can be detected by searchForKnownSwdTarget().
        enum DPv2Targets
        {
            // DP TARGETSEL values.
            RP2040_CORE0 = 0x01002927,  // *
            RP2040_CORE1 = 0x11002927,
            RP2040_RESCUE = 0xf1002927,
            RP2350 = 0x00040927,        // *
            // No known target is currently selected on the SWD bus.
            UNKNOWN = 0,
            // A DPv1 target which doesn't require a DPv2 TARGETSEL value is selected on the SWD bus.
            DPv1 = 0xFFFFFFFF
        };

        // Enumeration of Cortex-M CPU types supported by this debugger.
        // The order of this enumeration matters:
        // Values <= CPU_CORTEX_M0_PLUS are ARMv6M (except for CPU_UNKNOWN).
        // Values > CPU_CORTEX_M0_PLUS are ARMv7M
        enum CpuTypes
        {
            CPU_UNKNOWN = 0,
            CPU_CORTEX_M0,
            CPU_CORTEX_M0_PLUS,
            CPU_CORTEX_M3,
            CPU_CORTEX_M4,
            CPU_CORTEX_M7,
            CPU_CORTEX_M33,
        };

        // Constructor just sets up object. Need to call init() methods to actually initialize the PIO state machine.
        SWD();

        // Call this init() method to load the assembly language code into the caller specified PIO (pio0 or pio1).
        //
        // pio - The PIO instance to be used, pio0 or pio1.
        // frequency - Frequency to run the SWCLK in Hz.
        // swclkPin - The pin number of the SWCLK signal.
        // swdioPin - The pin number of the SWDIO signal.
        //
        // Returns true if everything was initialized successfully.
        // Returns false if the assembly language code fails to load into the specified PIO instance.
        bool init(PIO pio, uint32_t frequency, uint32_t swclkPin, uint32_t swdioPin);

        // Call this init() method to load the assembly language code into the first available PIO instance
        // available.
        //
        // frequency - Frequency to run the SWCLK in Hz.
        // swclkPin - The pin number of the SWCLK signal.
        // swdioPin - The pin number of the SWDIO signal.
        //
        // Returns true if everything was initialized successfully.
        // Returns false if the assembly language code fails to load into either PIO instance.
        bool init(uint32_t frequency, uint32_t swclkPin, uint32_t swdioPin);

        // Call this uninit() method to shutdown the PIO state machine and free up the associated resources, including
        // the program space.
        void uninit();

        // Call this method to just update the SWCLK frequency after calling init().
        //
        // frequency - Frequency to run the SWCLK in Hz.
        //
        // Returns true if everything was initialized successfully and false otherwise.
        bool setFrequency(uint32_t frequency)
        {
            return init(m_pio, frequency, m_swclkPin, m_swdioPin);
        }


        // Method which attempts switching a SWJ-DP from JTAG to SWD mode and reads DPIDR.
        //
        // Returns true if debug port was switched successfully. getDPIDR() will now return valid ID.
        // Returns false otherwise.
        bool sendJtagToSwdSequence();

        // Method to return the DPIDR value for the currently selected DP.
        //
        // Returns the 32-bit DPIDR value if DP has successfully been selected.
        // Returns 0xFFFFFFFF otherwise.
        uint32_t getDPIDR()
        {
            return m_dpidr;
        }

        // Method to place DPv2 JTAG-DP ports into dormant mode by sending special bit sequence.
        void switchJtagIntoDormantMode();

        // Method to place DPv2 SWD ports into dormant mode by sending special bit sequence.
        void switchSwdIntoDormantMode();

        // Method to wakeup dormant DPv2 SW-DP ports by sending special bit sequence.
        void switchSwdOutOfDormantMode();

        // UNDONE: Maybe this can search for DPv1 and DPv2 targets in the future.
        // Method to search for known DPv2 DP targets. The targets that can be detected by this method are starred in
        // the DPv2Targets enumeration above.
        //
        // Returns true if target debug port was found. getDPIDR() will now return valid ID.
        // Returns false otherwise.
        bool searchForKnownSwdTarget();

        // Method to attempt switching to the specified DPv2 capable SW-DP target and read DPIDIR.
        // Before the first attempt you will probably need to call switchSwdOutOfDormantMode() to wake up the SW-DP
        // ports since they are all in dormant mode upon power up.
        //
        // target - DPv2 capable SW-DP target to attempt selecting.
        //
        // Returns true if target debug port was found. getDPIDR() will now return valid ID.
        // Returns false otherwise.
        bool selectSwdTarget(DPv2Targets target);

        // Method to initialize a target for debugging. Typically called once after connecting to a new target for
        // the first time. It includes:
        //  Enabling debug and system power for the currently select DP.
        //  Enabling overrun detection which is required for proper functioning of this SWD driver.
        //  Finding the MEM-AP which is the debug entry.
        //
        // target - SwdTarget to be filled in with information unique to the current CPU core.
        //
        // Returns true if init was successful and false otherwise.
        bool initTargetDpForDebugging(SwdTarget& target);

        // Method to initialize a SwdTarget for a CPU core that shares the DP with other cores and instead uses a
        // different AP.
        //
        // target - SwdTarget to be filled in with information unique to the current CPU core.
        //
        // Returns true if init was successful and false otherwise.
        bool initApTarget(SwdTarget& target);

        // Method which returns the currently connected DPv2 target, if any.
        //
        // Returns one of the DPv2Targets elements or DPv2Targets::UNKNOWN if no DPv2 target is connected.
        DPv2Targets getTarget()
        {
            return m_target;
        }

        // Should byte, half-word, or word sized target accesses be used by read/writeTargetMemory()?
        enum TransferSize
        {
            TRANSFER_8BIT = 8,
            TRANSFER_16BIT = 16,
            TRANSFER_32BIT = 32
        };

        // Method to issue a target memory read.
        //
        // address - The 32-bit target address from which to start the read.
        // pvBuffer - Pointer to the data buffer to be filled in with the data read from the target.
        // bufferSize - The size of the read to be made, in bytes.
        // readSize - Is the size of each individual target read: 8, 16, or 32 bits.
        //
        // Returns the number of bytes successfully read. 0 if none were read successfully.
        uint32_t readTargetMemory(uint32_t address, void* pvBuffer, uint32_t bufferSize, TransferSize readSize);

        // Method to issue a target memory write.
        //
        // address - The 32-bit target address to which the write should be made.
        // pvBuffer - Pointer to the data buffer to be written to the target.
        // bufferSize - The size of the write to be made, in bytes.
        // writeSize - Is the size of each individual target write: 8, 16, or 32 bits.
        //
        // Returns the number of bytes successfully written. 0 if none were written successfully.
        uint32_t writeTargetMemory(uint32_t address, const void* pvBuffer, uint32_t bufferSize, TransferSize writeSize);

        // Method to find first DPv2 DP targets attached to SWD bus.
        // Can keep calling findNextSwdTarget() until it returns false to enumerate all of the connected targets.
        // WARNING: This can take a very long time to run (ie. hours). It is only useful for figuring out new
        //          hardware and not for everyday usage.
        //
        // pTarget - Points to the buffer to be filled in with the first discovered 32-bit DPv2 target ID.
        //
        // Returns true if a DPv2 DP module was found and *pTarget now contains its ID. It is also selected so any
        //         DP read/write requests made will be sent to it.
        // Return false otherwise and *pTarget isn't valid.
        bool findFirstSwdTarget(DPv2Targets* pTarget);

        // Method to find next DPv2 DP module attached to SWD bus.
        // Can keep calling findNextSwdTarget() until it returns false to enumerate all of the connected targets.
        // Must call findFirstSwdTarget() before calling this method.
        //
        // pTarget - Points to buffer to be filled in with the next discovered 32-bit DPv2 target ID.
        //
        // Returns true if a DPv2 DP module was found and *pTarget now contains its ID. It is also selected so any
        //         DP read/write requests made will be sent to it.
        // Return false otherwise and *pTarget isn't valid.
        bool findNextSwdTarget(DPv2Targets* pTarget);

        // Error codes that can be returned by getLastReadWriteError() to give more information about a failed
        // read or write call.
        enum TransferError
        {
            SWD_SUCCESS = 0,
            SWD_PROTOCOL = 1,
            SWD_WAIT = 2,
            SWD_PARITY = 3,
            SWD_FAULT_WDATAERR = 4,
            SWD_FAULT_ERROR = 5,
            SWD_FAULT_OVERRUN = 6
        };

        // Method to send SWD line reset followed by DPIDR read.
        //
        // Returns true if SWD debug port was reset successfully. getDPIDR() will now return valid ID.
        // Returns false otherwise.
        bool sendLineReset();

        // Idle the bus for the specified number of SWD clock cycles with SWDIO low.
        //
        // cyclesToIdle - The number of SWD clock cycles to idle the bus. Must be >=8.
        void idleBus(uint32_t cyclesToIdle);

        // Maximum number of microseconds to wait for the controlPower() call to take effect.
        void setMaximumPowerWait(uint32_t microseconds)
        {
            m_maxPowerMicroSeconds = microseconds;
        }

        // Maximum number of microseconds to retry failed reads due to WAIT or protocol errors.
        void setMaximumReadTime(uint32_t microseconds)
        {
            m_maxReadMicroseconds = microseconds;
        }

        // Maximum number of microseconds to retry failed writes due to WAIT or protocol errors.
        void setMaximumWriteTime(uint32_t microseconds)
        {
            m_maxWriteMicroseconds = microseconds;
        }

        // Maximum number of times to try reading from the same AP address before giving up.
        void setMaximumReadRetries(uint32_t retries)
        {
            m_maxReadRetries = retries;
        }

        // Maximum number of times to try writing to the same AP address before giving up.
        void setMaximumWriteRetries(uint32_t retries)
        {
            m_maxWriteRetries = retries;
        }

        // Retrieves the total count of WAIT retries conducted since this SWD object was instantiated.
        uint32_t getTotalWaitRetries()
        {
            return m_totalWaitRetries;
        }

        // Retrieves the total count of retries due to target entering protocol error state where it returns all 1s
        // for the ack response.
        uint32_t getTotalProtocolErrorRetries()
        {
            return m_totalProtocolErrorRetries;
        }

        // Retrieves the total count of retries due to parity errors.
        uint32_t getTotalParityErrorRetries()
        {
            return m_totalParityErrorRetries;
        }

        // Retrieves the total count of FAIL ack responses encountered.
        uint32_t getTotalFailErrorCount()
        {
            return m_totalFailErrors;
        }

        // Retrieves the total count of retries performed by readTargetMemory().
        uint32_t getTotalMemoryReadRetryCount()
        {
            return m_totalMemoryReadRetries;
        }

        // Retrieves the total count of retries performed by writeTargetMemory().
        uint32_t getTotalMemoryWriteRetryCount()
        {
            return m_totalMemoryWriteRetries;
        }

        // Number of times to pulse SWCLK with SWDIO pulled high for line reset (specification requires >= 50, cannot exceed 64).
        // The High + Low count can't exceed 64.
        void setLineResetHighClocks(uint32_t clocks = 51)
        {
            assert ( clocks >= 50 && clocks <= 64 );
            m_lineResetHighClocks = clocks;
        }

        // Number of times to pulse SWCLK with SWDIO pulled low after line reset (specification requires >= 2, cannot exceed 32).
        // The High + Low count can't exceed 64.
        void setLineResetLowClocks(uint32_t clocks = 8)
        {
            assert ( clocks >= 2 && clocks <= 32 );
            m_lineResetLowClocks = clocks;
        }

        // Enable/Disable error logging from this object.
        static void disableErrorLogging();
        static void enableErrorLogging();


    protected:
        // SWD packets can be destined to DP or AP access registers.
        enum SwdApOrDp { DP=0, AP=1 };
        // SWD packets can read or write from specified register.
        enum SwdReadOrWrite { WRITE = 0, READ = 1 };
        // The PIO based low level SWD signalling code can be in one of two modes:
        //  BIT_PATTERN: Ready to send arbitrary bit patterns like line reset, JTAG to SWD switch are such patterns.
        //  PACKET: Ready to send SWD packets with packet requests, acknowledge response, and read/write data transfers.
        enum SignalMode { BIT_PATTERN, PACKET };
        // Valid values for AP_CSW::AddrInc.
        enum CSW_AddrIncs { ADDR_INC_DISABLED = 0, ADDR_INC_SINGLE_ENABLED = 1, ADDR_INC_PACKED_ENABLED };
        // Valid values for AP_CSW::Size.
        enum CSW_Sizes
        {
            CSW_SIZE_8BIT = 0,
            CSW_SIZE_16BIT = 1,
            CSW_SIZE_32BIT = 2,
            CSW_SIZE_64BIT = 3,
            CSW_SIZE_128BIT = 4,
            CSW_SIZE_256BIT = 5
        };

    public:
        // These bits are placed in the DP_* and AP_* register definitions to indicate required setting in DPBANKSEL or
        // APBANKSEL for accesses to this register.
        static const uint32_t BANKSEL_0 = 0x0 << 4;
        static const uint32_t BANKSEL_1 = 0x1 << 4;
        static const uint32_t BANKSEL_2 = 0x2 << 4;
        static const uint32_t BANKSEL_3 = 0x3 << 4;
        static const uint32_t BANKSEL_4 = 0x4 << 4;
        static const uint32_t BANKSEL_5 = 0x5 << 4;
        static const uint32_t BANKSEL_E = 0xE << 4;
        static const uint32_t BANKSEL_F = 0xF << 4;
        static const uint32_t BANKSEL_D0 = 0xD0 << 4;
        static const uint32_t BANKSEL_D1 = 0xD1 << 4;
        static const uint32_t BANKSEL_DE = 0xDE << 4;
        static const uint32_t BANKSEL_DF = 0xDF << 4;
        static const uint32_t BANKSEL_F0 = 0xF0 << 4;
        static const uint32_t BANKSEL_FA = 0xFA << 4;
        static const uint32_t BANKSEL_FB = 0xFB << 4;
        static const uint32_t BANKSEL_FC = 0xFC << 4;
        static const uint32_t BANKSEL_FD = 0xFD << 4;
        static const uint32_t BANKSEL_FE = 0xFE << 4;
        static const uint32_t BANKSEL_FF = 0xFF << 4;

        // These bits are placed in the DP_* and AP_* register definitions to indicate whether they are RO, WO, or RW.
        static const uint32_t AP_DP_RO = 1 << 31;
        static const uint32_t AP_DP_WO = 1 << 30;
        static const uint32_t AP_DP_RW = AP_DP_RO | AP_DP_WO;

        // SWD DP Register Addresses which can be passed into readDP()/writeDP methods.
        static const uint32_t DP_DPIDR =     AP_DP_RO | BANKSEL_0 | 0x0;    // Read-only
        static const uint32_t DP_DPIDR1 =    AP_DP_RO | BANKSEL_1 | 0x0;    // Read-only
        static const uint32_t DP_BASEPTR0 =  AP_DP_RO | BANKSEL_2 | 0x0;    // Read-only
        static const uint32_t DP_BASEPTR1 =  AP_DP_RO | BANKSEL_3 | 0x0;    // Read-only
        static const uint32_t DP_ABORT =     AP_DP_WO | 0x0;                // Write-only
        static const uint32_t DP_CTRL_STAT = AP_DP_RW | BANKSEL_0 | 0x4;    // Read/Write
        static const uint32_t DP_DLCR =      AP_DP_RW | BANKSEL_1 | 0x4;    // Read/Write
        static const uint32_t DP_TARGETID =  AP_DP_RO | BANKSEL_2 | 0x4;    // Read-only
        static const uint32_t DP_DLPIDR =    AP_DP_RO | BANKSEL_3 | 0x4;    // Read-only
        static const uint32_t DP_EVENTSTAT = AP_DP_RO | BANKSEL_4 | 0x4;    // Read-only
        static const uint32_t DP_SELECT1 =   AP_DP_WO | BANKSEL_5 | 0x4;    // Write-only
        static const uint32_t DP_RESEND =    AP_DP_RO | 0x8;  // Read-only
        static const uint32_t DP_SELECT =    AP_DP_WO | 0x8;  // Write-only
        static const uint32_t DP_RDBUFF =    AP_DP_RO | 0xC;  // Read-only
        static const uint32_t DP_TARGETSEL = AP_DP_WO | 0xC;  // Write-only

        // SWD APv1 Register Addresses which can be passed into readAP()/writeAP method.
        static const uint32_t APv1_CSW  = AP_DP_RW | BANKSEL_0 | 0x0;
        static const uint32_t APv1_TAR  = AP_DP_RW | BANKSEL_0 | 0x4;
        static const uint32_t APv1_DRW  = AP_DP_RW | BANKSEL_0 | 0xC;
        static const uint32_t APv1_BD0  = AP_DP_RW | BANKSEL_1 | 0x0;
        static const uint32_t APv1_BD1  = AP_DP_RW | BANKSEL_1 | 0x4;
        static const uint32_t APv1_BD2  = AP_DP_RW | BANKSEL_1 | 0x8;
        static const uint32_t APv1_BD3  = AP_DP_RW | BANKSEL_1 | 0xC;
        static const uint32_t APv1_CFG1 = AP_DP_RO | BANKSEL_E | 0x0;
        static const uint32_t APv1_CFG  = AP_DP_RO | BANKSEL_F | 0x4;
        static const uint32_t APv1_BASE = AP_DP_RO | BANKSEL_F | 0x8;
        static const uint32_t APv1_IDR  = AP_DP_RO | BANKSEL_F | 0xC;

        // SWD APv2 Register Addresses which can be passed into readAP()/writeAP method.
        static const uint32_t APv2_CSW        = AP_DP_RW | BANKSEL_D0 | 0x0;
        static const uint32_t APv2_TAR        = AP_DP_RW | BANKSEL_D0 | 0x4;
        static const uint32_t APv2_DRW        = AP_DP_RW | BANKSEL_D0 | 0xC;
        static const uint32_t APv2_BD0        = AP_DP_RW | BANKSEL_D1 | 0x0;
        static const uint32_t APv2_BD1        = AP_DP_RW | BANKSEL_D1 | 0x4;
        static const uint32_t APv2_BD2        = AP_DP_RW | BANKSEL_D1 | 0x8;
        static const uint32_t APv2_BD3        = AP_DP_RW | BANKSEL_D1 | 0xC;
        static const uint32_t APv2_CFG1       = AP_DP_RO | BANKSEL_DE | 0x0;
        static const uint32_t APv2_CFG        = AP_DP_RO | BANKSEL_DF | 0x4;
        static const uint32_t APv2_BASE       = AP_DP_RO | BANKSEL_DF | 0x8;
        static const uint32_t APv2_IDR        = AP_DP_RO | BANKSEL_DF | 0xC;
        static const uint32_t APv2_ITCTRL     = AP_DP_RW | BANKSEL_F0 | 0x0;
        static const uint32_t APv2_CLAIMSET   = AP_DP_RW | BANKSEL_FA | 0x0;
        static const uint32_t APv2_CLAIMCLR   = AP_DP_RW | BANKSEL_FA | 0x4;
        static const uint32_t APv2_DEVAFF0    = AP_DP_RO | BANKSEL_FA | 0x8;
        static const uint32_t APv2_DEVAFF1    = AP_DP_RO | BANKSEL_FA | 0xC;
        static const uint32_t APv2_LAR        = AP_DP_WO | BANKSEL_FB | 0x0;
        static const uint32_t APv2_LSR        = AP_DP_RO | BANKSEL_FB | 0x4;
        static const uint32_t APv2_AUTHSTATUS = AP_DP_RO | BANKSEL_FB | 0x8;
        static const uint32_t APv2_DEVARCH    = AP_DP_RO | BANKSEL_FB | 0xC;
        static const uint32_t APv2_DEVID2     = AP_DP_RO | BANKSEL_FC | 0x0;
        static const uint32_t APv2_DEVID1     = AP_DP_RO | BANKSEL_FC | 0x4;
        static const uint32_t APv2_DEVID      = AP_DP_RO | BANKSEL_FC | 0x8;
        static const uint32_t APv2_DEVTYPE    = AP_DP_RO | BANKSEL_FC | 0xC;
        static const uint32_t APv2_PIDR4      = AP_DP_RO | BANKSEL_FD | 0x0;
        static const uint32_t APv2_PIDR5      = AP_DP_RO | BANKSEL_FD | 0x4;
        static const uint32_t APv2_PIDR6      = AP_DP_RO | BANKSEL_FD | 0x8;
        static const uint32_t APv2_PIDR7      = AP_DP_RO | BANKSEL_FD | 0xC;
        static const uint32_t APv2_PIDR0      = AP_DP_RO | BANKSEL_FE | 0x0;
        static const uint32_t APv2_PIDR1      = AP_DP_RO | BANKSEL_FE | 0x4;
        static const uint32_t APv2_PIDR2      = AP_DP_RO | BANKSEL_FE | 0x8;
        static const uint32_t APv2_PIDR3      = AP_DP_RO | BANKSEL_FE | 0xC;
        static const uint32_t APv2_CIDR0      = AP_DP_RO | BANKSEL_FF | 0x0;
        static const uint32_t APv2_CIDR1      = AP_DP_RO | BANKSEL_FF | 0x4;
        static const uint32_t APv2_CIDR2      = AP_DP_RO | BANKSEL_FF | 0x8;
        static const uint32_t APv2_CIDR3      = AP_DP_RO | BANKSEL_FF | 0xC;

        // Methods to read/write the DP and AP registers directly.
        // Return true if operation was successful and false otherwise.
        bool readDP(uint32_t address, uint32_t* pData)
        {
            return read(DP, address, pData);
        }
        bool writeDP(uint32_t address, uint32_t data)
        {
            return write(DP, address, data);
        }
        bool readAP(uint32_t address, uint32_t* pData)
        {
            return read(AP, address, pData);
        }
        bool writeAP(uint32_t address, uint32_t data)
        {
            return write(AP, address, data);
        }

        // UNDONE: If my current implementation works then this can be return void since it does nothing but
        //         set a few members. It can even be inlined.
        // Method to select the AP to which AP register reads/writes should be sent.
        // Returns true if operation was successful and false otherwise.
        bool selectAP(uint32_t ap);

        // Method to select the AP and verify that it is a valid MEM-AP.
        // Returns true if the check was successful and false otherwise.
        bool checkAP(uint32_t ap);


    protected:
        // Unknown value.
        static const uint32_t UNKNOWN_VAL = 0xFFFFFFFF;

        void invalidateDpApState()
        {
            // Some of the remembered AP/DP state is no longer valid once a new target has been selected.
            m_dpBank = UNKNOWN_VAL;
            m_apBank = UNKNOWN_VAL;
            m_ap = 0;
            m_cswValid = false;
        }
        bool powerDownDebugPort()
        {
            return controlPower(true, false);
        }
        void setBitPatternSignalMode();
        void setPacketSignalMode();
        void restartStateMachineInSignalMode();
        void sendRawLineReset();
        void __no_inline_not_in_flash_func(writeAndOptionalReadPIO)(uint32_t* pWrite,
                                                                    uint32_t writeLength,
                                                                    uint32_t* pRead,
                                                                    uint32_t readLength) __attribute__ ((optimize(3)));
        bool readDPIDR();
        uint32_t readTargetMemoryInternal(uint32_t address, uint8_t* pDest, uint32_t bufferSize, TransferSize readSize);
        uint32_t writeTargetMemoryInternal(uint32_t address, const uint8_t* pSrc, uint32_t bufferSize, TransferSize writeSize);
        bool updateCSW(CSW_AddrIncs addrInc, TransferSize transferSize);
        uint32_t calculateTransferCount(uint32_t startAddress, uint32_t expectedAddress);
        bool read(SwdApOrDp APnDP, uint32_t address, uint32_t* pData);
        bool write(SwdApOrDp APnDP, uint32_t address, uint32_t data);
        bool selectBank(SwdApOrDp APnDP, uint32_t address);
        bool selectDpBank(uint32_t address);
        bool selectApBank(uint32_t address);
        bool updateSelect();
        uint32_t buildPacketRequest(SwdApOrDp APnDP, SwdReadOrWrite RnW, uint8_t address);
        uint32_t calculateParity(uint32_t bitsForParity);
        DPv2Targets currentTargetId();
        void advanceTargetId(bool foundTarget);
        bool initControlStatRegister();
        bool setTurnAroundPeriod(uint32_t cycles);
        bool initDpVersion3Registers();
        bool controlPower(bool systemPower = true, bool debugPower = true);
        bool findDebugMemAP();
        bool clearAbortErrorBits();
        bool handleTransferResponse(uint32_t ack, bool ignoreProtocolError, bool* pRetry);
        void handleStickyErrors(uint32_t ack, bool* pRetry);
        void handleProtocolAndParityErrors();
        TransferError getLastReadWriteError()
        {
            return m_lastReadWriteError;
        }

        PIO             m_pio = NULL;
        uint32_t        m_stateMachine = UNKNOWN_VAL;
        uint32_t        m_programOffset = UNKNOWN_VAL;
        uint32_t        m_swclkPin = 0;
        uint32_t        m_swdioPin = 0;
        uint32_t        m_ap = 0;
        uint32_t        m_dpBank = UNKNOWN_VAL;
        uint32_t        m_apBank = UNKNOWN_VAL;
        uint32_t        m_dpidr = UNKNOWN_VAL;
        uint32_t        m_dpVersion = 0;
        uint32_t        m_apAddressSize = 8;
        uint32_t        m_csw = 0;

        // Parts of TARGETSEL used when iterating through possible targets via findFirst/NextSwdTarget().
        uint32_t        m_nextTargetDesignerContinuationCode = 0;
        uint32_t        m_nextTargetDesignerIdCode = 0;
        uint32_t        m_nextTargetPartNo = 0;
        uint32_t        m_nextTargetInstance = 0;

        uint32_t        m_lineResetHighClocks = 51;
        uint32_t        m_lineResetLowClocks = 8;
        uint32_t        m_maxPowerMicroSeconds = 1000;
        uint32_t        m_maxReadMicroseconds = 500;
        uint32_t        m_maxWriteMicroseconds = 500;
        uint32_t        m_totalWaitRetries = 0;
        uint32_t        m_totalProtocolErrorRetries = 0;
        uint32_t        m_totalParityErrorRetries = 0;
        uint32_t        m_totalFailErrors = 0;
        uint32_t        m_maxReadRetries = 2;
        uint32_t        m_maxWriteRetries = 2;
        uint32_t        m_totalMemoryReadRetries = 0;
        uint32_t        m_totalMemoryWriteRetries = 0;
        uint32_t        m_clearingErrors = 0;
        TransferError   m_lastReadWriteError = SWD_SUCCESS;
        SignalMode      m_signalMode = BIT_PATTERN;
        DPv2Targets     m_target = UNKNOWN;
        bool            m_cswValid = false;
        bool            m_apWritePending = false;
};




class SwdTarget
{
    friend SWD;

    public:
        // UNDONE: Comment needs updating: Constructor just sets up object. Need to call init() methods to actually initialize the PIO state machine.
        SwdTarget()
        {
            uninit();
        }

        // Call this method to change the SWCLK frequency of the underlying SWD bus.
        //
        // frequency - Frequency to run the SWCLK in Hz.
        //
        // Returns true if everything was initialized successfully and false otherwise.
        bool setFrequency(uint32_t frequency)
        {
            return m_pSWD->setFrequency(frequency);
        }


        // Method to return the DPIDR value for the currently selected DP.
        //
        // Returns the 32-bit DPIDR value if DP has successfully been selected.
        // Returns 0xFFFFFFFF otherwise.
        uint32_t getDPIDR()
        {
            return m_dpidr;
        }

        // Method to power down the CPU's debug subsystem and free this target object so that it can be reused.
        bool disconnect()
        {
            if (!selectCore())
            {
                return false;
            }
            bool result = m_pSWD->powerDownDebugPort();
            m_lastReadWriteError = m_pSWD->getLastReadWriteError();
            if (!result)
            {
                return false;
            }
            uninit();
            return true;
        }

        // Method to free this target object so that it can be reused.
        void uninit()
        {
            m_target = SWD::UNKNOWN;
            m_ap = 0;
            m_cpu = SWD::CPU_UNKNOWN;
            m_cpuID = 0;
            m_dpidr = SWD::UNKNOWN_VAL;
            memset(m_peripheralComponentIDs, 0, sizeof(m_peripheralComponentIDs));
        }

        // Method which returns the currently connected DPv2 target, if any.
        //
        // Returns one of the DPv2Targets elements or DPv2Targets::UNKNOWN if it is a DPv1 target is connected.
        SWD::DPv2Targets getTarget()
        {
            return m_target;
        }

        // Method which returns the Cortex-M CPU type of the currently connected device.
        //
        // Returns one of the CpuTypes elements or CpuTypes::CPU_UNKNOWN if type can't be determined.
        SWD::CpuTypes getCpuType()
        {
            return m_cpu;
        }

        // Lookup name for the specified cpu parameter.
        //
        // cpu - An element from the SWD::CpuTypes enumeration to be converted to human readable string.
        //
        // Returns the human readable name of the supplied cpu ordinal. Example: SWD::CPU_CORTEX_M0 to "Cortex-M0".
        static const char* getCpuTypeString(SWD::CpuTypes cpu);

        // Method which returns pointers to 12 element array containing device's component & peripheral ID words.
        const uint32_t* getComponentPeripheralIDs()
        {
            return &m_peripheralComponentIDs[0];
        }

        // Method which returns the 32-bit value of the Cortex-M CPUID register.
        uint32_t getCpuID()
        {
            return m_cpuID;
        }

        // Method to issue a target memory read.
        //
        // address - the 32-bit target address from which to start the read.
        // pvBuffer - Pointer to the data buffer to be filled in with the data read from the target.
        // bufferSize - The size of the read to be made, in bytes.
        // readSize - Is the size of each individual target read: 8, 16, or 32 bits.
        //
        // Returns the number of bytes successfully read. 0 if none were read successfully.
        uint32_t readMemory(uint32_t address, void* pvBuffer, uint32_t bufferSize, SWD::TransferSize readSize)
        {
            if (!selectCore())
            {
                return 0;
            }
            uint32_t bytesRead = m_pSWD->readTargetMemory(address, pvBuffer, bufferSize, readSize);
            m_lastReadWriteError = m_pSWD->getLastReadWriteError();
            return bytesRead;
        }

        // Method to issue a target memory write.
        //
        // address - the 32-bit target address to which the write should be made.
        // pvBuffer - Pointer to the data buffer to be written to the target.
        // bufferSize - The size of the write to be made, in bytes.
        // writeSize - Is the size of each individual target write: 8, 16, or 32 bits.
        //
        // Returns the number of bytes successfully written. 0 if none were written successfully.
        uint32_t writeMemory(uint32_t address, const void* pvBuffer, uint32_t bufferSize, SWD::TransferSize writeSize)
        {
            if (!selectCore())
            {
                return 0;
            }
            uint32_t bytesWritten = m_pSWD->writeTargetMemory(address, pvBuffer, bufferSize, writeSize);
            m_lastReadWriteError = m_pSWD->getLastReadWriteError();
            return bytesWritten;
        }

        // Fetch the cause of the last read or write call which has failed.
        SWD::TransferError getLastReadWriteError()
        {
            return m_lastReadWriteError;
        }

        // Fetch the SWD bus object used by SwdTarget.
        SWD* getSwdBus()
        {
            return m_pSWD;
        }


    protected:
        bool init(SWD* pSWD);
        bool fetchTargetDetails();
        void determineCpuType();
        bool selectCore()
        {
            if (!m_pSWD->selectSwdTarget(m_target))
            {
                return false;
            }
            return m_pSWD->selectAP(m_ap);
        }

        SWD*                m_pSWD;
        uint32_t            m_ap = 0;
        uint32_t            m_dpidr = SWD::UNKNOWN_VAL;
        uint32_t            m_peripheralComponentIDs[12];
        uint32_t            m_cpuID = 0;
        SWD::TransferError  m_lastReadWriteError = SWD::SWD_SUCCESS;
        SWD::DPv2Targets    m_target = SWD::UNKNOWN;
        SWD::CpuTypes       m_cpu = SWD::CPU_UNKNOWN;
};

#endif // SWD_H_
