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
const uint32_t S0 = 23;
const uint32_t FPSCR = S0 + 32;

// Does the device have a FPU?
enum FpuDiscoveryStates
{
    // We know for sure that the device doesn't have a FPU.
    FPU_NONE = 0,
    // Cortex-M4 or higher MIGHT have a FPU. Will need to query CPACR on first halt to know for sure.
    FPU_MAYBE,
    // We now know for sure that the device has a FPU.
    // We later determined the device doesn't have a FPU but target XML indicates that it does.
    FPU_AVAILABLE,
    FPU_NOT_AVAILABLE
};

// CPU register context information is stored here.
static uintmri_t      g_contextRegisters[registerCountFPU];
static ContextSection g_contextEntriesNoFPU = { .pValues = &g_contextRegisters[0], .count = registerCountNoFPU };
static ContextSection g_contextEntriesFPU = { .pValues = &g_contextRegisters[0], .count = registerCountFPU };
static MriContext     g_context;

static SWD       g_swd;
static GDBSocket g_gdbSocket;
static uint32_t  g_originalPC;
static bool      g_haltOnAttach = false;
static bool      g_wasStopFromGDB = false;
static bool      g_wasMemoryExceptionEncountered = false;
static bool      g_isResetting = false;
static bool      g_isInDebugger = false;
static bool      g_isDetaching = false;
static bool      g_doesHaltDebuggingNeedToBeEnabled = false;
static PlatformTrapReason g_trapReason;
static FpuDiscoveryStates g_fpu = FPU_NONE;

// When mri-swd has support (including FLASH programming) for the attached device, then these globals will store
// pointers to its function table and object data.
static DeviceFunctionTable* g_pDevice = NULL;
static DeviceObject*        g_pDeviceObject = NULL;

// Used as storage for the device memory layout XML.
static char*   g_pMemoryLayoutXML = NULL;
static int32_t g_memoryLayoutAllocSize = 0;
static int32_t g_memoryLayoutSize = 0;


// Forward Function Declarations.
static void cleanupDeviceObject();
static bool initSWD();
static bool waitForSwdAttach(uint32_t delayBetweenAttempts_ms);
static bool attemptSwdAttach();
static void determineCpuProperties();
static void searchSupportedDevices();
static FpuDiscoveryStates determineFpuAvailability();
static void updateSwdFrequencyForDevice();
static void setMemoryLayoutXML();
static bool initNetwork();
static void innerDebuggerLoop();
static bool isNetworkDown();
static void triggerMriCoreToExit();
static void requestCpuToHalt();
static bool readDHCSRWithRetry(uint32_t* pValue, uint32_t timeout_ms);
static bool readDHCSR(uint32_t* pValue);
static uint32_t readTargetMemory(uint32_t address, void* pvBuffer, uint32_t bufferSize, SWD::TransferSize readSize);
static void handleUnrecoverableSwdError();
static bool writeDHCSR(uint32_t DHCSR_Value);
static uint32_t writeTargetMemory(uint32_t address, const void* pvBuffer, uint32_t bufferSize, SWD::TransferSize writeSize);
static void enableHaltDebugging(uint32_t DHCSR_Val);
static bool isDeviceResetting(uint32_t DHCSR_Value);
static bool isCpuHalted(uint32_t DHCSR_Val);
static void checkForFpu();
static void saveContext();
static bool readCpuRegister(uint32_t registerIndex, uint32_t* pValue);
static void waitForRegisterTransferToComplete();
static bool hasRegisterTransferCompleted(uint32_t DHCSR_Value);
static void restoreContext();
static bool writeCpuRegister(uint32_t registerIndex, uint32_t value);
static bool requestCpuToResume();

static bool g_isSwdConnected = false;
static bool g_isNetworkConnected = false;

void mainDebuggerLoop()
{
    if (!g_swd.init(DEFAULT_SWD_CLOCK_RATE, SWCLK_PIN, SWDIO_PIN))
    {
        logError("Failed to initialize the SWD port.");
        return;
    }

    g_isSwdConnected = false;
    g_isNetworkConnected = false;
    g_isInDebugger = false;
    while (true)
    {
        if (!g_isSwdConnected)
        {
            cleanupDeviceObject();

            g_isSwdConnected = initSWD();
            if (!g_isSwdConnected)
            {
                logError("Failed to initialize SWD connection to debuggee.");
                return;
            }

            // UNDONE: Will later probably sample a physical switch here to determine if this should be done.
            g_haltOnAttach = HALT_ON_ATTACH;

            determineCpuProperties();
            updateSwdFrequencyForDevice();
            setMemoryLayoutXML();
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

        if (!g_isNetworkConnected)
        {
            g_gdbSocket.uninit();
            cyw43_arch_deinit();
        }
    }
}

static void cleanupDeviceObject()
{
    if (!g_pDevice)
    {
        return;
    }

    g_pDevice->free(g_pDeviceObject, &g_swd);
    g_pDeviceObject = NULL;
    g_pDevice = NULL;
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
    putchar('\n');
    logErrorEnable();

    return true;
}

static bool attemptSwdAttach()
{
    // Search through all known SWD DPv2 targets to see if any are found.
    // If that fails, try detecting SWD targets which don't go dormant, after making to switch SWJ-DP targets into SWD
    // mode.
    if (g_swd.searchForKnownSwdTarget())
    {
        // Have found one of the SWD DPv2 targets known by this debugger.
        logInfoF("Found DPv2 SWD Target=0x%08X with DPIDR=0x%08lX", g_swd.getTarget(), g_swd.getDPIDR());
    }
    else if (g_swd.sendJtagToSwdSequence())
    {
        // Have found a non-dormant SWD target.
        logInfoF("Found SWD Target with DPIDR=0x%08lX", g_swd.getDPIDR());
    }
    else
    {
        logError("No SWD Targets found.");
        return false;
    }

    logInfo("Initializing target's debug components...");
    bool result = g_swd.initTargetForDebugging();
    if (!result)
    {
        logError("Failed to initialize target's debug components.");
        return false;
    }
    logInfo("SWD initialization complete!");
    return true;
}

static void determineCpuProperties()
{
    searchSupportedDevices();
    g_fpu = determineFpuAvailability();
}

static void searchSupportedDevices()
{
    assert ( g_pDevice == NULL && g_pDeviceObject == NULL );
    for (size_t i = 0; i < g_supportedDevicesLength ; i++)
    {
        DeviceFunctionTable* pDevice = g_supportedDevices[i];
        DeviceObject* pObject = pDevice->detect(&g_swd);
        if (pObject)
        {
            g_pDevice = pDevice;
            g_pDeviceObject = pObject;
            logInfoF("Found device of type %s.", pDevice->getName(g_pDeviceObject, &g_swd));

            break;
        }
    }
    if (!g_pDevice)
    {
        logInfo("Unrecognized device type. FLASH programming not supported!");
    }
}

static FpuDiscoveryStates determineFpuAvailability()
{
    if (g_pDevice)
    {
        if (g_pDevice->hasFpu(g_pDeviceObject, &g_swd))
        {
            logInfo("Device has FPU.");
            return FPU_AVAILABLE;
        }
        else
        {
            return FPU_NONE;
        }
    }
    else
    {
        if (g_swd.getCpuType() >= SWD::CPU_CORTEX_M4)
        {
            return FPU_MAYBE;
        }
        else
        {
            return FPU_NONE;
        }
    }
}

static void updateSwdFrequencyForDevice()
{
    if (!g_pDevice)
    {
        return;
    }

    uint32_t swdFrequency = g_pDevice->getMaximumSWDClockFrequency(g_pDeviceObject, &g_swd);
    if (!g_swd.setFrequency(swdFrequency))
    {
        logErrorF("Failed to re-initialize the SWD port to a frequency of %lu.", swdFrequency);
        return;
    }
    logInfoF("Configured SWD frequency to %lu.", swdFrequency);
}


static void setMemoryLayoutXML()
{
    const DeviceMemoryLayout* pLayout;
    if (g_pDevice)
    {
        pLayout = g_pDevice->getMemoryLayout(g_pDeviceObject, &g_swd);
    }
    else
    {
        pLayout = deviceDefaultMemoryLayout(NULL, &g_swd);
    }
    assert ( pLayout );

    const char xmlHeader[] = "<?xml version=\"1.0\"?>"
                             "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                             "<memory-map>";
    const char xmlFooter[] = "</memory-map>";
    const char xmlLongestRegion[] = "<memory type=\"flash\" start=\"0x12345678\" length=\"0x12345678\"> <property name=\"blocksize\">0x12345678</property></memory>";

    // Calculate the maximum number of bytes needed for the full memory layout XML string and make sure that
    // g_pMemoryLayoutXML is at least this large.
    int32_t bytesRequired = sizeof(xmlHeader) + pLayout->regionCount * sizeof(xmlLongestRegion) + sizeof(xmlFooter);
    if (bytesRequired > g_memoryLayoutAllocSize)
    {
        char* pAlloc = (char*)realloc(g_pMemoryLayoutXML, bytesRequired);
        if (!pAlloc)
        {
            logErrorF("Failed to allocate %d bytes for memory layout XML.", bytesRequired);
            return;
        }
        g_pMemoryLayoutXML = pAlloc;
        g_memoryLayoutAllocSize = bytesRequired;
    }

    // Add the XML header.
    int32_t bytesLeft = g_memoryLayoutAllocSize;
    char* pDest = g_pMemoryLayoutXML;
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
    g_memoryLayoutSize = g_memoryLayoutAllocSize - bytesLeft;
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
        if (isNetworkDown())
        {
            continue;
        }

        // See if the target should be halted because of attach or because GDB has sent a command via TCP/IP.
        if (g_haltOnAttach || (g_gdbSocket.isGdbConnected() && !g_gdbSocket.m_tcpToMriQueue.isEmpty()))
        {
            logInfoF("%s has requested a CPU halt.", g_haltOnAttach ? "User" : "GDB");
            g_wasStopFromGDB = true;
            g_haltOnAttach = false;
            requestCpuToHalt();
        }

        // Read the Debug Halting Control and Status Register to query current state of CPU.
        uint32_t DHCSR_Val = 0;
        if (!readDHCSRWithRetry(&DHCSR_Val, READ_DHCSR_TIMEOUT_MS))
        {
            logInfo("SWD target is no longer responding. Will attempt to reattach.");
            g_isSwdConnected = false;
            continue;
        }
        if (g_doesHaltDebuggingNeedToBeEnabled)
        {
            g_doesHaltDebuggingNeedToBeEnabled = false;
            enableHaltDebugging(DHCSR_Val);
        }
        if (isDeviceResetting(DHCSR_Val))
        {
            if (g_isResetting)
            {
                g_isResetting = false;
                logInfo("Device RESET request completed.");
            }
            else
            {
                logInfo("External device RESET detected.");
            }
            continue;
        }
        if (!hasCpuHalted && isCpuHalted(DHCSR_Val))
        {
            logInfo("CPU has halted.");
            hasCpuHalted = true;
        }
        // UNDONE: Should check the sleep, lockup, retire bits in the DHCSR as well.

        if (!g_isResetting && g_gdbSocket.isGdbConnected() && hasCpuHalted)
        {
            hasCpuHalted = false;
            checkForFpu();
            saveContext();
            mriDebugException(&g_context);
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
                restoreContext();
                requestCpuToResume();

                // UNDONE: Move out into a detachDebugger() method.
                if (g_isDetaching)
                {
                    g_swd.powerDownDebugPort();
                    g_swd.uninit();
                    g_gdbSocket.closeClient();

                    logInfo("CPU execution has been resumed and debugger detached.");
                    logInfo("Debug hardware will need to be reset to reconnect.");
                    while (true)
                    {
                        __wfe();
                    }
                }
                else
                {
                    logInfoF("CPU execution has been resumed. %s", Platform_IsSingleStepping() ? "Single stepping enabled." : "");
                }
            }
        }
    }
}

static bool isNetworkDown()
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

static void triggerMriCoreToExit()
{
    if (!g_isInDebugger)
    {
        return;
    }

    const uint8_t emptyPacket[] = "+$#00";
    g_gdbSocket.m_tcpToMriQueue.init();
    g_gdbSocket.m_tcpToMriQueue.write(emptyPacket, sizeof(emptyPacket)-1);
}

static const uint32_t DHCSR_C_HALT_Bit = 1 << 1;

static void requestCpuToHalt()
{
    uint32_t DHCSR_Val = 0;

    readDHCSR(&DHCSR_Val);

    DHCSR_Val |= DHCSR_C_HALT_Bit;

    if (!writeDHCSR(DHCSR_Val))
    {
        logError("Failed to set C_HALT bit in DHCSR.");
    }
}

static bool readDHCSRWithRetry(uint32_t* pValue, uint32_t timeout_ms)
{
    bool returnVal = false;
    absolute_time_t endTime = make_timeout_time_ms(timeout_ms);
    g_swd.disableErrorLogging();
    logErrorDisable();
    do
    {
        if (readDHCSR(pValue))
        {
            returnVal = true;
            break;
        }
    } while (absolute_time_diff_us(get_absolute_time(), endTime) > 0);
    logErrorEnable();
    g_swd.enableErrorLogging();

    return returnVal;
}

static const uint32_t DHCSR_Address = 0xE000EDF0;

static bool readDHCSR(uint32_t* pValue)
{
    if (readTargetMemory(DHCSR_Address, pValue, sizeof(*pValue), SWD::TRANSFER_32BIT) != sizeof(*pValue))
    {
        logError("Failed to read DHCSR register.");
        return false;
    }
    return true;
}

static uint32_t readTargetMemory(uint32_t address, void* pvBuffer, uint32_t bufferSize, SWD::TransferSize readSize)
{
    if (!g_isSwdConnected)
    {
        return 0;
    }

    uint32_t bytesRead = g_swd.readTargetMemory(address, pvBuffer, bufferSize, readSize);
    if (bytesRead == 0 && g_swd.getLastReadWriteError() == SWD::SWD_PROTOCOL)
    {
        handleUnrecoverableSwdError();
    }
    return bytesRead;
}

static void handleUnrecoverableSwdError()
{
    logErrorF("Encountered unrecoverable read/write error %d.", g_swd.getLastReadWriteError());
    g_isSwdConnected = false;
    triggerMriCoreToExit();
}

static bool writeDHCSR(uint32_t DHCSR_Value)
{
    // Upper 16-bits must contain DBGKEY for CPU to accept this write.
    const uint32_t DHCSR_DBGKEY_Shift = 16;
    const uint32_t DHCSR_DBGKEY_Mask = 0xFFFF << DHCSR_DBGKEY_Shift;
    const uint32_t DHCSR_DBGKEY = 0xA05F << DHCSR_DBGKEY_Shift;
    DHCSR_Value = (DHCSR_Value & ~DHCSR_DBGKEY_Mask) | DHCSR_DBGKEY;

    return writeTargetMemory(DHCSR_Address, &DHCSR_Value, sizeof(DHCSR_Value), SWD::TRANSFER_32BIT) == sizeof(DHCSR_Value);
}

static uint32_t writeTargetMemory(uint32_t address, const void* pvBuffer, uint32_t bufferSize, SWD::TransferSize writeSize)
{
    if (!g_isSwdConnected)
    {
        return 0;
    }

    uint32_t bytesWritten = g_swd.writeTargetMemory(address, pvBuffer, bufferSize, writeSize);
    if (bytesWritten == 0 && g_swd.getLastReadWriteError() == SWD::SWD_PROTOCOL)
    {
        handleUnrecoverableSwdError();
    }
    return bytesWritten;
}

static void enableHaltDebugging(uint32_t DHCSR_Val)
{
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
    }
}

static bool isDeviceResetting(uint32_t DHCSR_Value)
{
    const uint32_t S_RESET_ST_Bit = 1 << 25;

    return !!(DHCSR_Value & S_RESET_ST_Bit);
}

static bool isCpuHalted(uint32_t DHCSR_Val)
{
    const uint32_t DHCSR_S_HALT_Bit = 1 << 17;
    return !!(DHCSR_Val & DHCSR_S_HALT_Bit);
}

static void checkForFpu()
{
    if (g_fpu != FPU_MAYBE)
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
        g_fpu = FPU_NOT_AVAILABLE;
        return;
    }
    if ((CPACR_OrigValue & CPACR_CP10_Mask) && (CPACR_OrigValue & CPACR_CP11_Mask))
    {
        // FPU exists and is enabled.
        logInfo("Device has FPU.");
        g_fpu = FPU_AVAILABLE;
        return;
    }

    // Try enabling the FPU.
    uint32_t CPACR_TestValue = CPACR_CP10_Mask | CPACR_CP11_Mask | CPACR_OrigValue;
    if (writeTargetMemory(CPACR_Address, &CPACR_TestValue, sizeof(CPACR_TestValue), SWD::TRANSFER_32BIT) != sizeof(CPACR_TestValue))
    {
        logError("Failed to test CPACR register.");
        // On error, assume that FPU doesn't exist.
        g_fpu = FPU_NOT_AVAILABLE;
        return;
    }

    // See if the FPU enable took. If so then the FPU exists.
    uint32_t CPACR_UpdatedValue = ~CPACR_TestValue;
    if (readTargetMemory(CPACR_Address, &CPACR_UpdatedValue, sizeof(CPACR_UpdatedValue), SWD::TRANSFER_32BIT) != sizeof(CPACR_UpdatedValue))
    {
        logError("Failed to verify CPACR register.");
        // On error, assume that FPU doesn't exist.
        g_fpu = FPU_NOT_AVAILABLE;
    }
    if (CPACR_UpdatedValue == CPACR_TestValue)
    {
        logInfo("Device has FPU.");
        g_fpu = FPU_AVAILABLE;
    }

    // Restore CPACR Value.
    if (writeTargetMemory(CPACR_Address, &CPACR_OrigValue, sizeof(CPACR_OrigValue), SWD::TRANSFER_32BIT) != sizeof(CPACR_OrigValue))
    {
        logError("Failed to restore CPACR register.");
    }
}

// The special registers: CONTROL, FAULTMASK, BASEPRI, and PRIMASK are accessed at this single DCRSR index.
const uint32_t specialRegisterIndex = 0x14;
// The floating point registers are found at this index in the DCRSR.
const uint32_t fpscrIndex = 0x21;
const uint32_t s0Index = 0x40;

static void saveContext()
{
    bool encounteredError = false;

    bool contextHasFpu = g_fpu >= FPU_MAYBE;
    if (contextHasFpu)
    {
        Context_Init(&g_context, &g_contextEntriesFPU, 1);
    }
    else
    {
        Context_Init(&g_context, &g_contextEntriesNoFPU, 1);
    }

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

    // Transfer FPU registers if we returned target XML that indicates they might/do exist.
    if (contextHasFpu)
    {
        // Transfer S0-S31 floating point registers.
        for (uint32_t i = 0 ; i < 32 ; i++)
        {
            uint32_t regValue = 0xBAADFEED;
            if (g_fpu == FPU_AVAILABLE)
            {
                if (!readCpuRegister(s0Index+i, &regValue))
                {
                    encounteredError = true;
                }
            }
            Context_Set(&g_context, S0+i, regValue);
        }

        // Transfer FPSCR register.
        if (g_fpu == FPU_AVAILABLE)
        {
            uint32_t regValue = 0xBAADFEED;
            if (g_fpu == FPU_AVAILABLE)
            {
                if (!readCpuRegister(fpscrIndex, &regValue))
                {
                    encounteredError = true;
                }
            }
            Context_Set(&g_context, FPSCR, regValue);
        }
    }

    if (encounteredError)
    {
        logError("Failed to read CPU register(s).");
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
    if (writeTargetMemory(DCRSR_Address, &DCRSR_Value, sizeof(DCRSR_Value), SWD::TRANSFER_32BIT) != sizeof(DCRSR_Value))
    {
        return false;
    }

    waitForRegisterTransferToComplete();

    return readTargetMemory(DCRDR_Address, pValue, sizeof(*pValue), SWD::TRANSFER_32BIT) == sizeof(*pValue);
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

    // Transfer FPU registers back to CPU if we have detected that the FPU exists for sure.
    if (g_fpu == FPU_AVAILABLE)
    {
        // Transfer S0-S31 floating point registers.
        for (uint32_t i = 0 ; i < 32 ; i++)
        {
            if (!writeCpuRegister(s0Index+i, Context_Get(&g_context, S0+i)))
            {
                encounteredError = true;
            }
        }

        // Transfer FPSCR register.
        if (!writeCpuRegister(fpscrIndex, Context_Get(&g_context, FPSCR)))
        {
            encounteredError = true;
        }
    }

    if (encounteredError)
    {
        logError("Failed to write CPU register(s).");
    }
}

static bool writeCpuRegister(uint32_t registerIndex, uint32_t value)
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

static bool requestCpuToResume()
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
static void checkForNetworkDown();
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

static void checkForNetworkDown()
{
    isNetworkDown();
}

static void checkForDeviceReset()
{
    if (!g_isSwdConnected)
    {
        return;
    }

    // See if the Debug Halting Control and Status Register indicates that the CPU has been reset.
    uint32_t DHCSR_Val = 0;
    g_swd.disableErrorLogging();
    bool result = readDHCSR(&DHCSR_Val);
    g_swd.enableErrorLogging();
    if (!result)
    {
        return;
    }
    if (isDeviceResetting(DHCSR_Val))
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
static void enableWatchpointBreakpointAndVectorCatchSupport();
static void enableDWTandVectorCatches();
static bool setOrClearBitsInDEMCR(uint32_t bitMask, bool set);
static void initDWT();
static uint32_t clearDWTComparators();
static uint32_t getDWTComparatorCount();
static void clearDWTComparator(uint32_t comparatorAddress);
static void initFPB();
static uint32_t clearFPBComparators();
static uint32_t getFPBCodeComparatorCount();
static uint32_t readFPControlRegister();
static uint32_t getFPBLiteralComparatorCount();
static void clearFPBComparator(uint32_t comparatorAddress);
static void enableFPB();
static void writeFPControlRegister(uint32_t FP_CTRL_Value);


void Platform_Init(Token* pParameterTokens)
{
    // UNDONE: I probably need to perform this for each core in a dual core system.
    enableWatchpointBreakpointAndVectorCatchSupport();
    g_doesHaltDebuggingNeedToBeEnabled = true;
}

static void enableWatchpointBreakpointAndVectorCatchSupport()
{
    enableDWTandVectorCatches();
    initDWT();
    initFPB();
}

static void enableDWTandVectorCatches()
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

static bool setOrClearBitsInDEMCR(uint32_t bitMask, bool set)
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



static void initDWT()
{
    uint32_t watchpointCount = clearDWTComparators();
    logInfoF("CPU supports %lu hardware watchpoints.", watchpointCount);
}

static uint32_t clearDWTComparators()
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

static uint32_t getDWTComparatorCount()
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

//  Selects action to be taken on match.
static const uint32_t DWT_COMP_FUNCTION_FUNCTION_Mask = 0xF;

static void clearDWTComparator(uint32_t comparatorAddress)
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

static void initFPB()
{
    uint32_t breakpointCount = clearFPBComparators();
    logInfoF("CPU supports %lu hardware breakpoints.", breakpointCount);
    enableFPB();
}

static uint32_t clearFPBComparators()
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
    if (readTargetMemory(FP_CTRL_Address, &FP_CTRL_Value, sizeof(FP_CTRL_Value), SWD::TRANSFER_32BIT) != sizeof(FP_CTRL_Value))
    {
        logError("Failed to read FP_CTRL register.");
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
    if (writeTargetMemory(comparatorAddress, &comparatorValue, sizeof(comparatorValue), SWD::TRANSFER_32BIT) != sizeof(comparatorValue))
    {
        logError("Failed to write to FP comparator register.");
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
    if (writeTargetMemory(FP_CTRL_Address, &FP_CTRL_Value, sizeof(FP_CTRL_Value), SWD::TRANSFER_32BIT) != sizeof(FP_CTRL_Value))
    {
        logError("Failed to write FP_CTRL register.");
    }
}




// *********************************************************************************************************************
// Routines called by the MRI core each time the CPU is halted and resumed.
// *********************************************************************************************************************
static PlatformTrapReason cacheTrapReason(void);
static bool readDFSR(uint32_t* pDFSR);
static PlatformTrapReason findMatchedWatchpoint(void);
static PlatformTrapReason getReasonFromMatchComparator(uint32_t comparatorAddress, uint32_t function);
static void clearDFSR();
static bool writeDFSR(uint32_t dfsr);
static void disableResetVectorCatch();

void Platform_EnteringDebugger(void)
{
    g_isInDebugger = true;
    g_wasMemoryExceptionEncountered = false;
    g_originalPC = Platform_GetProgramCounter();
    Platform_DisableSingleStep();
    g_trapReason = cacheTrapReason();
    disableResetVectorCatch();
}

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

static PlatformTrapReason cacheTrapReason(void)
{
    PlatformTrapReason reason = { MRI_PLATFORM_TRAP_TYPE_UNKNOWN, 0x00000000 };
    uint32_t debugFaultStatus = 0;
    if (!readDFSR(&debugFaultStatus))
    {
        return reason;
    }

    if (debugFaultStatus & DFSR_BKPT_Bit)
    {
        /* Was caused by hardware or software breakpoint. If PC points to BKPT then report as software breakpoint. */
        if (Platform_TypeOfCurrentInstruction() == MRI_PLATFORM_INSTRUCTION_HARDCODED_BREAKPOINT)
            reason.type = MRI_PLATFORM_TRAP_TYPE_SWBREAK;
        else
            reason.type = MRI_PLATFORM_TRAP_TYPE_HWBREAK;
    }
    else if (debugFaultStatus & DFSR_DWTTRAP_Bit)
    {
        reason = findMatchedWatchpoint();
    }
    return reason;
}

static bool readDFSR(uint32_t* pDFSR)
{
    if (readTargetMemory(DFSR_Address, pDFSR, sizeof(*pDFSR), SWD::TRANSFER_32BIT) != sizeof(*pDFSR))
    {
        logError("Failed to read DFSR.");
        return false;
    }
    return true;
}

// Base address of the DWT comp register array.
static const uint32_t DWT_COMP_ARRAY = 0xE0001020;

//  Matched.  Read-only.  Set to 1 to indicate that this comparator has been matched.  Cleared on read.
static const uint32_t DWT_COMP_FUNCTION_MATCHED = 1 << 24;

static PlatformTrapReason findMatchedWatchpoint(void)
{
    PlatformTrapReason reason = { MRI_PLATFORM_TRAP_TYPE_UNKNOWN, 0x00000000 };
    uint32_t currentComparatorAddress = DWT_COMP_ARRAY;
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

//  Selects action to be taken on match.
//      Data Read Watchpoint
static const uint32_t DWT_COMP_FUNCTION_FUNCTION_DATA_READ = 0x5;
//      Data Write Watchpoint
static const uint32_t DWT_COMP_FUNCTION_FUNCTION_DATA_WRITE = 0x6;
//      Data Read/Write Watchpoint
static const uint32_t DWT_COMP_FUNCTION_FUNCTION_DATA_READWRITE = 0x7;

static PlatformTrapReason getReasonFromMatchComparator(uint32_t comparatorAddress, uint32_t function)
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


void Platform_LeavingDebugger(void)
{
    g_wasStopFromGDB = false;
    clearDFSR();
    g_isInDebugger = false;
}

static void clearDFSR()
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

static bool writeDFSR(uint32_t dfsr)
{
    if (writeTargetMemory(DFSR_Address, &dfsr, sizeof(dfsr), SWD::TRANSFER_32BIT) != sizeof(dfsr))
    {
        logError("Failed to write to DFSR.");
        return false;
    }
    return true;
}

// Enable Reset Vector Catch. This causes a Local reset to halt a running system.
const uint32_t DEMCR_VC_CORERESET_Bit = 1 << 0;

static void disableResetVectorCatch()
{
    if (!setOrClearBitsInDEMCR(DEMCR_VC_CORERESET_Bit, false))
    {
        logError("Failed to clear reset vector catch bits in DEMCR register.");
        return;
    }
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
    {
        Buffer_WriteByteAsHex(pBuffer, *pByte++);
    }
}




// *********************************************************************************************************************
// Routines called by the MRI core to read and write memory on the target device.
// *********************************************************************************************************************
uint32_t Platform_MemRead32(uintmri_t address)
{
    g_wasMemoryExceptionEncountered = false;
    uint32_t data = 0;
    uint32_t bytesRead = readTargetMemory(address, &data, sizeof(data), SWD::TRANSFER_32BIT);
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
    uint32_t bytesRead = readTargetMemory(address, &data, sizeof(data), SWD::TRANSFER_16BIT);
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
    uint32_t bytesRead = readTargetMemory(address, &data, sizeof(data), SWD::TRANSFER_8BIT);
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
    uint32_t bytesWritten = writeTargetMemory(address, &value, sizeof(value), SWD::TRANSFER_32BIT);
    if (bytesWritten != sizeof(value))
    {
        g_wasMemoryExceptionEncountered = true;
    }
}

void Platform_MemWrite16(uintmri_t address, uint16_t value)
{
    g_wasMemoryExceptionEncountered = false;
    uint32_t bytesWritten = writeTargetMemory(address, &value, sizeof(value), SWD::TRANSFER_16BIT);
    if (bytesWritten != sizeof(value))
    {
        g_wasMemoryExceptionEncountered = true;
    }
}

void Platform_MemWrite8(uintmri_t address, uint8_t value)
{
    g_wasMemoryExceptionEncountered = false;
    uint32_t bytesWritten = writeTargetMemory(address, &value, sizeof(value), SWD::TRANSFER_8BIT);
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
    // Hex digit buffer is twice the size of binary buffer.
    uint8_t  buffer[PACKET_SIZE/2];
    assert ( readByteCount <= count_of(buffer) );
    if (readByteCount > count_of(buffer))
    {
        readByteCount = count_of(buffer);
    }

    uint32_t bytesRead = 0;
    if (isWordAligned(readByteCount) && isWordAligned(address))
    {
        bytesRead = readTargetMemory(address, buffer, readByteCount, SWD::TRANSFER_32BIT);
    }
    else
    {
        bytesRead = readTargetMemory(address, buffer, readByteCount, SWD::TRANSFER_8BIT);
    }
    writeBytesToBufferAsHex(pBuffer, buffer, bytesRead);

    return bytesRead;
}

static uint32_t readMemoryHalfWordIntoHexBuffer(Buffer* pBuffer, uintmri_t address)
{
    uint16_t value;

    if (isNotHalfWordAligned(address))
        return readMemoryBytesIntoHexBuffer(pBuffer, address, 2);

    value = Platform_MemRead16(address);
    if (Platform_WasMemoryFaultEncountered())
        return 0;
    writeBytesToBufferAsHex(pBuffer, &value, sizeof(value));

    return sizeof(value);
}

static int isNotHalfWordAligned(uintmri_t address)
{
    return address & 1;
}

static uint32_t readMemoryWordIntoHexBuffer(Buffer* pBuffer, uintmri_t address)
{
    uint32_t value;

    if (!isWordAligned(address))
        return readMemoryBytesIntoHexBuffer(pBuffer, address, 4);

    value = Platform_MemRead32(address);
    if (Platform_WasMemoryFaultEncountered())
        return 0;
    writeBytesToBufferAsHex(pBuffer, &value, sizeof(value));

    return sizeof(value);
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
    // Hex digit buffer is twice the size of binary buffer.
    uint8_t  buffer[PACKET_SIZE/2];
    assert ( writeByteCount <= count_of(buffer) );
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
        bytesWritten = writeTargetMemory(address, buffer, writeByteCount, SWD::TRANSFER_32BIT);
    }
    else
    {
        bytesWritten = writeTargetMemory(address, buffer, writeByteCount, SWD::TRANSFER_8BIT);
    }

    return bytesWritten;
}

static int writeHexBufferToHalfWordMemory(Buffer* pBuffer, uintmri_t address)
{
    uint16_t value;

    if (isNotHalfWordAligned(address))
        return writeHexBufferToByteMemory(pBuffer, address, 2);

    if (!readBytesFromHexBuffer(pBuffer, &value, sizeof(value)))
        return 0;

    Platform_MemWrite16(address, value);
    if (Platform_WasMemoryFaultEncountered())
        return 0;

    return 1;
}

static int readBytesFromHexBuffer(Buffer* pBuffer, void* pv, size_t length)
{
    uint8_t* pBytes = (uint8_t*)pv;
    while (length--)
    {
        __try
            *pBytes++ = Buffer_ReadByteAsHex(pBuffer);
        __catch
            __rethrow_and_return(0);
    }
    return 1;
}

static int writeHexBufferToWordMemory(Buffer* pBuffer, uintmri_t address)
{
    uint32_t value;

    if (!isWordAligned(address))
        return writeHexBufferToByteMemory(pBuffer, address, 4);

    if (!readBytesFromHexBuffer(pBuffer, &value, sizeof(value)))
        return 0;

    Platform_MemWrite32(address, value);
    if (Platform_WasMemoryFaultEncountered())
        return 0;

    return 1;
}


// UNDONE: Should be optimized like the hex version....might need this for load.
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
    while (writeByteCount-- > 0)
    {
        char currChar;

        __try
            currChar = Buffer_ReadChar(pBuffer);
        __catch
            __rethrow_and_return(0);

        Platform_MemWrite8(address++, (uint8_t)currChar);
        if (Platform_WasMemoryFaultEncountered())
            return 0;
    }

    return 1;
}

static int writeBinaryBufferToHalfWordMemory(Buffer* pBuffer, uintmri_t address)
{
    uint16_t value;

    if (isNotHalfWordAligned(address))
        return writeBinaryBufferToByteMemory(pBuffer, address, 2);

    if (!readBytesFromBinaryBuffer(pBuffer, &value, sizeof(value)))
        return 0;

    Platform_MemWrite16(address, value);
    if (Platform_WasMemoryFaultEncountered())
        return 0;

    return 1;
}

static int readBytesFromBinaryBuffer(Buffer*  pBuffer, void* pvMemory, uint32_t writeByteCount)
{
    uint8_t* p = (uint8_t*) pvMemory;

    while (writeByteCount-- > 0)
    {
        __try
            *p++ = Buffer_ReadChar(pBuffer);
        __catch
            __rethrow_and_return(0);
    }

    return 1;
}

static int writeBinaryBufferToWordMemory(Buffer* pBuffer, uintmri_t address)
{
    uint32_t value;

    if (!isWordAligned(address))
        return writeBinaryBufferToByteMemory(pBuffer, address, 4);

    if (!readBytesFromBinaryBuffer(pBuffer, &value, sizeof(value)))
        return 0;

    Platform_MemWrite32(address, value);
    if (Platform_WasMemoryFaultEncountered())
        return 0;

    return 1;
}




// *********************************************************************************************************************
// Routines called by the MRI core to enable/disable single stepping on the target CPU.
// *********************************************************************************************************************
static void configureSingleSteppingBitsInDHCSR(bool enableSingleStepping);
void Platform_EnableSingleStep(void)
{
    configureSingleSteppingBitsInDHCSR(true);
}

void Platform_DisableSingleStep(void)
{
    configureSingleSteppingBitsInDHCSR(false);
}

static const uint32_t DHCSR_C_STEP_Bit = 1 << 2;
static const uint32_t DHCSR_C_MASKINTS_Bit = 1 << 3;

static void configureSingleSteppingBitsInDHCSR(bool enableSingleStepping)
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

int Platform_IsSingleStepping(void)
{
    uint32_t DHCSR_Val = 0;
    readDHCSR(&DHCSR_Val);
    return DHCSR_Val & DHCSR_C_STEP_Bit;
}




// *********************************************************************************************************************
// Routines called by the MRI core to access the CPU's program counter.
// *********************************************************************************************************************
static uint16_t getFirstHalfWordOfCurrentInstruction(void);
static uint16_t throwingMemRead16(uint32_t address);
static bool isInstruction32Bit(uint16_t firstWordOfInstruction);


uintmri_t Platform_GetProgramCounter(void)
{
    return Context_Get(&g_context, PC);
}

void Platform_SetProgramCounter(uintmri_t newPC)
{
    Context_Set(&g_context, PC, newPC);
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
static uint32_t enableFPBBreakpoint(uint32_t breakpointAddress, bool is32BitInstruction);
static uint32_t findFPBBreakpointComparator(uint32_t breakpointAddress, bool is32BitInstruction);
static uint32_t calculateFPBComparatorValue(uint32_t breakpointAddress, bool is32BitInstruction);
static bool isBreakpointAddressInvalid(uint32_t breakpointAddress);
static uint32_t getFPBRevision();
static bool isAddressOdd(uint32_t address);
static bool isAddressAboveLowestHalfGig(uint32_t address);
static uint32_t calculateFPBComparatorValueRevision1(uint32_t breakpointAddress, bool is32BitInstruction);
static uint32_t calculateFPBComparatorReplaceValue(uint32_t breakpointAddress, bool is32BitInstruction);
static bool isAddressInUpperHalfword(uint32_t address);
static uint32_t calculateFPBComparatorValueRevision2(uint32_t breakpointAddress);
static uint32_t maskOffFPBComparatorReservedBits(uint32_t comparatorValue);
static uint32_t findFreeFPBBreakpointComparator();
static bool isFPBComparatorEnabled(uint32_t comparator);
static bool isFPBComparatorEnabledRevision1(uint32_t comparator);
static bool isFPBComparatorEnabledRevision2(uint32_t comparator);
static uint32_t disableFPBBreakpointComparator(uint32_t breakpointAddress, bool is32BitInstruction);


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

    uint32_t FPBBreakpointComparator = enableFPBBreakpoint(address, is32BitInstruction);
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

static uint32_t enableFPBBreakpoint(uint32_t breakpointAddress, bool is32BitInstruction)
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

uint32_t FPB_COMP_ARRAY_Address = 0xE0002008;

static uint32_t findFPBBreakpointComparator(uint32_t breakpointAddress, bool is32BitInstruction)
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

// Flash Patch breakpoint architecture revision. 0 for revision 1 and 1 for revision 2.
uint32_t FP_CTRL_REVISION2 = 0x1;

static uint32_t calculateFPBComparatorValue(uint32_t breakpointAddress, bool is32BitInstruction)
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

static bool isBreakpointAddressInvalid(uint32_t breakpointAddress)
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

static uint32_t getFPBRevision()
{
    // Flash Patch breakpoint architecture revision. 0 for revision 1 and 1 for revision 2.
    uint32_t FP_CTRL_REV_Shift = 28;
    uint32_t FP_CTRL_REV_Mask = (0xF << FP_CTRL_REV_Shift);

    uint32_t controlValue = readFPControlRegister();
    return ((controlValue & FP_CTRL_REV_Mask) >> FP_CTRL_REV_Shift);
}

static bool isAddressOdd(uint32_t address)
{
    return !!(address & 0x1);
}

static bool isAddressAboveLowestHalfGig(uint32_t address)
{
    return !!(address & 0xE0000000);
}

//  Specified bits 28:2 of the address to be use for match on this comparator.
uint32_t FP_COMP_COMP_Shift = 2;
uint32_t FP_COMP_COMP_Mask = (0x07FFFFFF << FP_COMP_COMP_Shift);
//  Enables this comparator.  Set to 1 to enable.
uint32_t FP_COMP_ENABLE_Bit = 1 << 0;

static uint32_t calculateFPBComparatorValueRevision1(uint32_t breakpointAddress, bool is32BitInstruction)
{
    uint32_t    comparatorValue;
    comparatorValue = (breakpointAddress & FP_COMP_COMP_Mask);
    comparatorValue |= FP_COMP_ENABLE_Bit;
    comparatorValue |= calculateFPBComparatorReplaceValue(breakpointAddress, is32BitInstruction);

    return comparatorValue;
}

static uint32_t calculateFPBComparatorReplaceValue(uint32_t breakpointAddress, bool is32BitInstruction)
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

static bool isAddressInUpperHalfword(uint32_t address)
{
    return !!(address & 0x2);
}

// FlashPatch Comparator Register Bits for revision 2.
//  Enables this comparator for flash patching when FP_COMP_BE is 0. Set to 1 to enable.
uint32_t FP_COMP_FE_Bit = 1 << 31;
//  Enables this comparator as a breakpoint.  Set to 1 to enable.
uint32_t FP_COMP_BE_Bit = 1 << 0;

static uint32_t calculateFPBComparatorValueRevision2(uint32_t breakpointAddress)
{
    return breakpointAddress | FP_COMP_BE_Bit;
}

static uint32_t maskOffFPBComparatorReservedBits(uint32_t comparatorValue)
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

static uint32_t findFreeFPBBreakpointComparator()
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

static bool isFPBComparatorEnabled(uint32_t comparator)
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

static bool isFPBComparatorEnabledRevision1(uint32_t comparator)
{
    return !!(comparator & FP_COMP_ENABLE_Bit);
}

static bool isFPBComparatorEnabledRevision2(uint32_t comparator)
{
    return !!((comparator & FP_COMP_BE_Bit) || (comparator & FP_COMP_FE_Bit));
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

    uint32_t FPBBreakpointComparator = enableFPBBreakpoint(address, isInstruction32Bit(firstInstructionWord));
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

    disableFPBBreakpointComparator(address, is32BitInstruction);
}

static uint32_t disableFPBBreakpointComparator(uint32_t breakpointAddress, bool is32BitInstruction)
{
    uint32_t existingFPBBreakpoint = findFPBBreakpointComparator(breakpointAddress, is32BitInstruction);
    if (existingFPBBreakpoint != 0)
    {
        clearFPBComparator(existingFPBBreakpoint);
        logInfoF("Hardware breakpoint cleared at address 0x%08lX.", breakpointAddress);
    }

    return existingFPBBreakpoint;
}

__throws void Platform_ClearHardwareBreakpoint(uintmri_t address)
{
    uint16_t  firstInstructionWord;

     __try
    {
        firstInstructionWord = throwingMemRead16(address);
    }
    __catch
        __rethrow;

    disableFPBBreakpointComparator(address, isInstruction32Bit(firstInstructionWord));
}




// *********************************************************************************************************************
// Routines called by the MRI core to set and clear watch points.
// *********************************************************************************************************************
static uint32_t convertWatchpointTypeToCortexMType(PlatformWatchpointType type);
static bool isValidDWTComparatorSetting(uint32_t watchpointAddress,
                                       uint32_t watchpointSize,
                                       uint32_t watchpointType);
static bool isValidDWTComparatorSize(uint32_t watchpointSize);
static bool isPowerOf2(uint32_t value);
static bool isValidDWTComparatorAddress(uint32_t watchpointAddress, uint32_t watchpointSize);
static bool isAddressAlignedToSize(uint32_t address, uint32_t size);
static bool isValidDWTComparatorType(uint32_t watchpointType);
static uint32_t enableDWTWatchpoint(uint32_t watchpointAddress,
                                    uint32_t watchpointSize,
                                    uint32_t watchpointType);
static uint32_t findDWTComparator(uint32_t watchpointAddress,
                                  uint32_t watchpointSize,
                                  uint32_t watchpointType);
static bool doesDWTComparatorMatch(uint32_t comparatorAddress,
                                   uint32_t address,
                                   uint32_t size,
                                   uint32_t function);
static bool doesDWTComparatorFunctionMatch(uint32_t comparatorAddress, uint32_t function);
static uint32_t maskOffDWTFunctionBits(uint32_t functionValue);
static bool doesDWTComparatorAddressMatch(uint32_t comparatorAddress, uint32_t address);
static bool doesDWTComparatorMaskMatch(uint32_t comparatorAddress, uint32_t size);
static uint32_t calculateLog2(uint32_t value);
static uint32_t findFreeDWTComparator();
static bool isDWTComparatorFree(uint32_t comparatorAddress);
static bool attemptToSetDWTComparator(uint32_t comparatorAddress,
                                      uint32_t watchpointAddress,
                                      uint32_t watchpointSize,
                                      uint32_t watchpointType);
static bool attemptToSetDWTComparatorMask(uint32_t comparatorAddress, uint32_t watchpointSize);
static uint32_t disableDWTWatchpoint(uint32_t watchpointAddress,
                                     uint32_t watchpointSize,
                                     uint32_t watchpointType);


__throws void Platform_SetHardwareWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type)
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

static uint32_t convertWatchpointTypeToCortexMType(PlatformWatchpointType type)
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

static bool isValidDWTComparatorSetting(uint32_t watchpointAddress,
                                        uint32_t watchpointSize,
                                        uint32_t watchpointType)
{
    return isValidDWTComparatorSize(watchpointSize) &&
           isValidDWTComparatorAddress(watchpointAddress, watchpointSize) &&
           isValidDWTComparatorType(watchpointType);
}

static bool isValidDWTComparatorSize(uint32_t watchpointSize)
{
    return isPowerOf2(watchpointSize);
}

static bool isPowerOf2(uint32_t value)
{
    return (value & (value - 1)) == 0;
}

static bool isValidDWTComparatorAddress(uint32_t watchpointAddress, uint32_t watchpointSize)
{
    return isAddressAlignedToSize(watchpointAddress, watchpointSize);
}

static bool isAddressAlignedToSize(uint32_t address, uint32_t size)
{
    uint32_t addressMask = ~(size - 1);
    return address == (address & addressMask);
}

static bool isValidDWTComparatorType(uint32_t watchpointType)
{
    return (watchpointType == DWT_COMP_FUNCTION_FUNCTION_DATA_READ) ||
           (watchpointType == DWT_COMP_FUNCTION_FUNCTION_DATA_WRITE) ||
           (watchpointType == DWT_COMP_FUNCTION_FUNCTION_DATA_READWRITE);
}

static uint32_t enableDWTWatchpoint(uint32_t watchpointAddress,
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

static uint32_t findDWTComparator(uint32_t watchpointAddress,
                                  uint32_t watchpointSize,
                                  uint32_t watchpointType)
{
    uint32_t currentComparatorAddress = DWT_COMP_ARRAY;
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

static bool doesDWTComparatorMatch(uint32_t comparatorAddress,
                                   uint32_t address,
                                   uint32_t size,
                                   uint32_t function)
{
    return doesDWTComparatorFunctionMatch(comparatorAddress, function) &&
           doesDWTComparatorAddressMatch(comparatorAddress, address) &&
           doesDWTComparatorMaskMatch(comparatorAddress, size);
}

static bool doesDWTComparatorFunctionMatch(uint32_t comparatorAddress, uint32_t function)
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

static uint32_t maskOffDWTFunctionBits(uint32_t functionValue)
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

static bool doesDWTComparatorAddressMatch(uint32_t comparatorAddress, uint32_t address)
{
    uint32_t compValue = 0;
    if (readTargetMemory(comparatorAddress + offsetof(DWT_COMP_Type, comp), &compValue, sizeof(compValue), SWD::TRANSFER_32BIT) != sizeof(compValue))
    {
        logError("Failed to read DWT comparator register.");
        return false;
    }
    return compValue == address;
}

static bool doesDWTComparatorMaskMatch(uint32_t comparatorAddress, uint32_t size)
{
    uint32_t maskValue = 0;
    if (readTargetMemory(comparatorAddress + offsetof(DWT_COMP_Type, mask), &maskValue, sizeof(maskValue), SWD::TRANSFER_32BIT) != sizeof(maskValue))
    {
        logError("Failed to read DWT mask register.");
        return false;
    }
    return maskValue == calculateLog2(size);
}

static uint32_t calculateLog2(uint32_t value)
{
    uint32_t log2 = 0;

    while (value > 1)
    {
        value >>= 1;
        log2++;
    }

    return log2;
}

static uint32_t findFreeDWTComparator()
{
    uint32_t currentComparatorAddress = DWT_COMP_ARRAY;
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

static bool isDWTComparatorFree(uint32_t comparatorAddress)
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

static bool attemptToSetDWTComparator(uint32_t comparatorAddress,
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

static bool attemptToSetDWTComparatorMask(uint32_t comparatorAddress, uint32_t watchpointSize)
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

__throws void Platform_ClearHardwareWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type)
{
    uint32_t nativeType = convertWatchpointTypeToCortexMType(type);

    if (!isValidDWTComparatorSetting(address, size, nativeType))
    {
        __throw(invalidArgumentException);
    }

    disableDWTWatchpoint(address, size, nativeType);
}

static uint32_t disableDWTWatchpoint(uint32_t watchpointAddress,
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



// *********************************************************************************************************************
// Routines called by the MRI core when dealing with exception/fault causes.
// *********************************************************************************************************************
// Looks like this could be 9 bits on ARMv7-M or 6 bits on ARMv6-M. User the smaller count here.
static const uint32_t IPSR_Mask = (1 << 6) - 1;
static const uint32_t CFSR_Address = 0xE000ED28;

static void displayHardFaultCauseToGdbConsole(void);
static void displayMemFaultCauseToGdbConsole(void);
static void displayBusFaultCauseToGdbConsole(void);
static void displayUsageFaultCauseToGdbConsole(void);
uint8_t Platform_DetermineCauseOfException(void)
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
        uint32_t exceptionNumber = Context_Get(&g_context, CPSR) & IPSR_Mask;
        switch(exceptionNumber)
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

    // Check the other bits in the DFSR if VCATCH wasn't set.
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
        if ((DHCSR_Value & DHCSR_C_STEP_Bit) && !g_wasStopFromGDB)
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

PlatformTrapReason Platform_GetTrapReason(void)
{
    return g_trapReason;
}

void Platform_DisplayFaultCauseToGdbConsole(void)
{
    // Nothing to do on ARMv6-M devices since they don't have fault status registers.
    if (g_swd.getCpuType() < SWD::CPU_CORTEX_M3)
    {
        return;
    }

    uint32_t exceptionNumber = Context_Get(&g_context, CPSR) & IPSR_Mask;
    switch (exceptionNumber)
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

static void displayHardFaultCauseToGdbConsole(void)
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

static void displayMemFaultCauseToGdbConsole(void)
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
        WriteHexValueToGdbConsole(Context_Get(&g_context, SP));
    }
    if (memManageFaultStatusRegister & unstackingErrorBit)
    {
        WriteStringToGdbConsole("\n    Unstacking Error w/ SP = ");
        WriteHexValueToGdbConsole(Context_Get(&g_context, SP));
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

static void displayBusFaultCauseToGdbConsole(void)
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
        WriteHexValueToGdbConsole(Context_Get(&g_context, SP));
    }
    if (busFaultStatusRegister & unstackingErrorBit)
    {
        WriteStringToGdbConsole("\n    Unstacking Error w/ SP = ");
        WriteHexValueToGdbConsole(Context_Get(&g_context, SP));
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

static void displayUsageFaultCauseToGdbConsole(void)
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
    if (g_fpu >= FPU_MAYBE)
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
    if (g_fpu >= FPU_MAYBE)
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
    return g_memoryLayoutSize;
}

const char* Platform_GetDeviceMemoryMapXml(void)
{
    return g_pMemoryLayoutXML;
}



// *********************************************************************************************************************
// Routines called by the MRI core to reset the device.
// *********************************************************************************************************************
// UNDONE: May want to override and do this earlier since the MRI core does it after trying to restart the device.
void Platform_ResetDevice(void)
{
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
static void enableResetVectorCatch();
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

    // "monitor help" can be handled by both the devcie specific code and this more generic code to allow listing of all
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
    if (g_pDevice == NULL || g_pDevice->handleMonitorCommand == NULL)
    {
        // No device specific command handler so just return false.
        return false;
    }

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

    return g_pDevice->handleMonitorCommand(g_pDeviceObject, &g_swd, &arguments[0], argCount);
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

    // Next character should be hex version of ' ', space character.
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
        enableResetVectorCatch();
        WriteStringToGdbConsole("Will reset and then halt on next continue.\r\n");
        PrepareStringResponse("OK");
        return HANDLER_RETURN_HANDLED;
    }

    PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
    return HANDLER_RETURN_HANDLED;
}

static void enableResetVectorCatch()
{
    if (!setOrClearBitsInDEMCR(DEMCR_VC_CORERESET_Bit, true))
    {
        logError("Failed to set reset vector catch bit in DEMCR register.");
        return;
    }
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

    if (!g_pDevice)
    {
        logError("mri-swd doesn't know how to FLASH program this device type.");
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
        return HANDLER_RETURN_HANDLED;
    }
    logInfoF("Erasing 0x%X (%u) bytes of FLASH at 0x%08X.", addressLength.length, addressLength.length, addressLength.address);
    if (!g_pDevice->flashBegin(g_pDeviceObject, &g_swd))
    {
        PrepareStringResponse(MRI_ERROR_INVALID_ARGUMENT);
        return HANDLER_RETURN_HANDLED;
    }

    // Return OK packet now before starting the erase so that GDB can start sending the next packet at the same time.
    sendOkResponseWithNewPacketBuffer();
    g_flashErrorDetected = false;

    if (!g_pDevice->flashErase(g_pDeviceObject, &g_swd, addressLength.address, addressLength.length))
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
    assert ( g_pDevice );

    // Return OK packet now before starting the write so that GDB can start sending the next packet at the same time.
    sendOkResponseWithNewPacketBuffer();

    uint32_t length = Buffer_BytesLeft(pBuffer);
    logInfoF("Writing 0x%lX (%lu) bytes to FLASH at 0x%08lX.", length, length, address);
    if (!g_pDevice->flashProgram(g_pDeviceObject, &g_swd, address, pBuffer->pCurrent, length))
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
    assert ( g_pDevice );

    logInfo("Completed FLASH update.");

    // First check to see if the previous write failed.
    if (g_flashErrorDetected)
    {
        PrepareStringResponse(MRI_ERROR_MEMORY_ACCESS_FAILURE);
        return HANDLER_RETURN_HANDLED;
    }

    if (!g_pDevice->flashEnd(g_pDeviceObject, &g_swd))
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
static bool disableSingleStepAndInterrupts();
static bool reenableInterrupts();
bool startCodeOnDevice(const CortexM_Registers* pRegistersIn)
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
        return false;
    }

    return true;
}

// UNDONE: Shouldn't need to call this anymore as single stepping is disabled on the way into the debugger now.
static bool disableSingleStepAndInterrupts()
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

static bool reenableInterrupts()
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


bool waitForCodeToHalt(CortexM_Registers* pRegistersOut, uint32_t timeout_ms)
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


bool runCodeOnDevice(CortexM_Registers* pRegistersInOut, uint32_t timeout_ms)
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

    parameters.parameter1 = Context_Get(&g_context, R0);
    parameters.parameter2 = Context_Get(&g_context, R1);
    parameters.parameter3 = Context_Get(&g_context, R2);
    parameters.parameter4 = Context_Get(&g_context, R3);

    return parameters;
}


void Platform_SetSemihostCallReturnAndErrnoValues(int returnValue, int errNo)
{
    Context_Set(&g_context, R0, returnValue);
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

    if (!readTargetMemory(address, buffer, length, SWD::TRANSFER_8BIT))
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
