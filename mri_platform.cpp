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
// Routines to expose the Cortex-M debug functionality via SWD to the mri debug core.
#define PLATFORM_MODULE "mri_platform.cpp"
#include "logging.h"
#include <string.h>
#include <stdio.h>
#include <pico/cyw43_arch.h>
#include "gdb_socket.h"
#include "uart_wifi_bridge.h"
#include "swd.h"
#include "mri_platform.h"
#include "config.h"
#include "version.h"
#include "devices/devices.h"
#include "cpu_core.h"
#include "ui.h"

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
    #include <core/fileio.h>
    #include <core/mbedsys.h>
    #include <semihost/newlib/newlib_stubs.h>
}



static SWD              g_swdBus;
static GDBSocket        g_gdbSocket;
static UartWiFiBridge   g_uartWifiBridge;
static CpuCores         g_cores;
static UI               g_ui(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT,
                             OLED_MOSI_PIN, OLED_CLK_PIN, OLED_DC_PIN, OLED_RESET_PIN, OLED_CS_PIN);

static bool       g_isSwdConnected = false;
static bool       g_isNetworkConnected = false;
static bool       g_haltOnAttach = false;
static bool       g_wasStopFromGDB = false;
static bool       g_isResetting = false;
static bool       g_haltOnReset = false;
static bool       g_isInDebugger = false;
static bool       g_isDetaching = false;

static uint32_t   g_originalPC;
static bool       g_wasMemoryExceptionEncountered = false;


// Forward Function Declarations.
static bool initSWD();
static bool waitForSwdAttach(uint32_t delayBetweenAttempts_ms);
static bool attemptSwdAttach();
static void handleSwdDisconnect();
static void triggerMriCoreToExit();
static bool initNetwork();
static void innerDebuggerLoop();
static bool checkForNetworkDown();
static void detachDebugger();


void mainDebuggerLoop()
{
    if (!g_ui.init())
    {
        logError("Failed to initialize the OLED.");
        return;
    }
    if (!g_swdBus.init(DEFAULT_SWD_CLOCK_RATE, SWCLK_PIN, SWDIO_PIN))
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
            g_uartWifiBridge.uninit();
            g_gdbSocket.uninit();
            cyw43_arch_deinit();
        }
    }
}

static bool initSWD()
{
    bool returnCode = false;

    UI::setTargetName("No Target");
    g_ui.setRunState("");
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

    bool result = g_cores.init(&g_swdBus, handleSwdDisconnect);
    logInfo("SWD initialization complete!");
    return result;
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


static bool initNetwork()
{
    g_ui.updateWiFiState(UI::WIFI_CONNECTING, NULL);
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
    g_ui.updateWiFiState(UI::WIFI_CONNECTED, ip4addr_ntoa(netif_ip4_addr(netif_list)));
    logInfo("Connected to Wi-Fi router.");

    if (!g_gdbSocket.init(GDB_SOCKET_PORT_NUMBER))
    {
        logError("Failed to initialize GDB socket.");
        return false;
    }
    if (!g_uartWifiBridge.init(UART_WIFI_BRIDGE_PORT_NUMBER, UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE))
    {
        logError("Failed to initialize UART<->WiFi Bridge.");
        return false;
    }

    return true;
}

static void innerDebuggerLoop()
{
    bool isFirstHalt = true;
    bool hasCpuHalted = false;
    while (g_isSwdConnected && g_isNetworkConnected)
    {
        // Check the WiFi link to see if it has gone down.
        if (checkForNetworkDown())
        {
            continue;
        }

        // See if the target should be halted because of attach or because GDB has sent a command via TCP/IP.
        if (g_haltOnAttach || (g_gdbSocket.isGdbConnected() && !g_gdbSocket.m_tcpToMriQueue.isEmpty()))
        {
            g_ui.setRunState("Halt!");
            logInfoF("%s has requested a CPU halt.", g_haltOnAttach ? "User" : "GDB");
            g_wasStopFromGDB = true;
            g_haltOnAttach = false;
            g_cores.requestCoresToHalt();
            if (isFirstHalt)
            {
                // Any active watchpoints or breakpoints would be from a previous GDB connection.
                g_cores.clearBreakpointsAndWatchpoints();
            }
            isFirstHalt = false;
        }

        // Query current state of CPU.
        if (!g_cores.refreshCoreStates(READ_DHCSR_TIMEOUT_MS))
        {
            logInfo("SWD target is no longer responding. Will attempt to reattach.");
            g_isSwdConnected = false;
            continue;
        }
        int resettingCore = g_cores.indexOfResettingCore();
        int haltingCore = g_cores.indexOfHaltedCore(false);
        if (resettingCore != CpuCores::CORE_NONE)
        {
            g_ui.setRunState("Reset");
            logInfo("External device RESET detected.");
            continue;
        }
        if (!hasCpuHalted && haltingCore != CpuCores::CORE_NONE)
        {
            g_ui.setRunState("Halted");
            logInfoF("Core%d has halted.", haltingCore);
            g_cores.requestCoresToHalt();
            hasCpuHalted = g_cores.waitForCoresToHalt(READ_DHCSR_TIMEOUT_MS);
            if (!hasCpuHalted)
            {
                logError("Timed out waiting for all cores to halt.");
            }
        }
        // UNDONE: Should check the sleep, lockup, retire bits in the DHCSR as well.

        if (g_gdbSocket.isGdbConnected() && hasCpuHalted)
        {
            hasCpuHalted = false;

            g_cores.enterMriCore(haltingCore);
            if (!g_isSwdConnected || !g_isNetworkConnected)
            {
                continue;
            }
            else if (g_isResetting)
            {
                g_ui.setRunState("Reset");
                logInfo("GDB has requested device RESET.");
                g_isResetting = false;
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
                    if (g_cores.isAnyCoreSingleStepping())
                    {
                        g_ui.setRunState("Step");
                    }
                    else
                    {
                        g_ui.setRunState("Running");
                    }
                    logInfoF("CPU execution has been resumed. %s",
                        g_cores.isAnyCoreSingleStepping() ? "Single stepping enabled." : "");
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

    // See if the halted core has been reset out underneath us while debugging it.
    if (g_cores.isHaltedCoreResetting())
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
static int isWordAligned(uintmri_t value);

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

uintmri_t Platform_ReadMemory(void* pvBuffer, uintmri_t address, uintmri_t readByteCount)
{
    if (isWordAligned(readByteCount) && isWordAligned(address))
    {
        return g_cores.readMemory(address, pvBuffer, readByteCount, SWD::TRANSFER_32BIT);
    }
    return g_cores.readMemory(address, pvBuffer, readByteCount, SWD::TRANSFER_8BIT);
}

static int isWordAligned(uintmri_t value)
{
    return (value & 3) == 0;
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
    if (g_cores.hasFPU() >= CpuCore::FPU_MAYBE)
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
    if (g_cores.hasFPU() >= CpuCore::FPU_MAYBE)
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
    g_cores.reset(g_haltOnReset, READ_DHCSR_TIMEOUT_MS);
    g_haltOnReset = false;
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
    // Default to not halting on reset.
    g_haltOnReset = false;
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
        g_haltOnReset = true;
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
    WriteStringToGdbConsole(" mri-core Version: " MRI_VERSION_STRING " [" MRI_BRANCH "]\r\n");
    PrepareStringResponse("OK");
    return HANDLER_RETURN_HANDLED;
}

static uint32_t handleMonitorHelpCommand()
{
    // showfault is implemented and handled in MRI core.
    // The rest are implemented in this module.
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




// *********************************************************************************************************************
// Routines called by the MRI core to interact with RTOS and/or hardware threads/cores.
// *********************************************************************************************************************
uintmri_t Platform_RtosGetHaltedThreadId(void)
{
    return g_cores.getHaltedThreadId();
}

uintmri_t Platform_RtosGetFirstThreadId(void)
{
    return g_cores.getFirstThreadId();
}

uintmri_t Platform_RtosGetNextThreadId(void)
{
    return g_cores.getNextThreadId();
}

const char* Platform_RtosGetExtraThreadInfo(uintmri_t threadId)
{
    return g_cores.getExtraThreadInfo(threadId);
}

MriContext* Platform_RtosGetThreadContext(uintmri_t threadId)
{
    return g_cores.getThreadContext(threadId);
}

int Platform_RtosIsThreadActive(uintmri_t threadId)
{
    return g_cores.isThreadActive(threadId);
}

int Platform_RtosIsSetThreadStateSupported(void)
{
    return g_cores.isSetThreadStateSupported();
}

void Platform_RtosSetThreadState(uintmri_t threadId, PlatformThreadState state)
{
    g_cores.setThreadState(threadId, state);
}

void Platform_RtosRestorePrevThreadState(void)
{
    g_cores.restorePrevThreadState();
}



// This situation is of no concern for SWD hardware debugging.
void Platform_HandleFaultFromHighPriorityCode(void)
{
}



// *********************************************************************************************************************
// Routines called by the MRI core to determine if the stop was caused by semihost request.
// *********************************************************************************************************************
static int isInstructionArmSemihostBreakpoint(uint16_t instruction);
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
    if (isInstructionArmSemihostBreakpoint(currentInstruction))
    {
        return isGdbTryingToBreakIn ? MRI_PLATFORM_INSTRUCTION_HARDCODED_BREAKPOINT :
                                      MRI_PLATFORM_INSTRUCTION_ARM_SEMIHOST_CALL;
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

static int isInstructionArmSemihostBreakpoint(uint16_t instruction)
{
    const uint16_t armSemihostBreakpointMachineCode = 0xbeab;

    return armSemihostBreakpointMachineCode == instruction;
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


uintmri_t Platform_GetNewlibSemihostOperation(void)
{
    uintmri_t opCode = 0;
    __try
    {
        opCode = getFirstHalfWordOfCurrentInstruction();
    }
    __catch
    {
        opCode = 0;
    }
    return opCode & 0xFF;
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

    return (instructionType == MRI_PLATFORM_INSTRUCTION_ARM_SEMIHOST_CALL ||
            instructionType == MRI_PLATFORM_INSTRUCTION_NEWLIB_SEMIHOST_CALL);
}


int Semihost_HandleSemihostRequest(void)
{
    PlatformInstructionType    instructionType = Platform_TypeOfCurrentInstruction();
    PlatformSemihostParameters parameters = Platform_GetSemihostCallParameters();

    if (instructionType == MRI_PLATFORM_INSTRUCTION_ARM_SEMIHOST_CALL)
        return Semihost_HandleArmSemihostRequest(&parameters);
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
// Can safely return an empty UUID for all boards that would be connected to MRI-SWD.
// *********************************************************************************************************************
const uint8_t* Platform_GetUid(void)
{
    return NULL;
}


size_t Platform_GetUidSize(void)
{
    return 0;
}
