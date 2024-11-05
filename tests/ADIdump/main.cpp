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
// Test program to connect to SWD target and dump a bunch of diagnostic information.
#define MAIN_MODULE "main.cpp"
#include "logging.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include "swd.h"


// SWD pin connections.
#define SWCLK_PIN 2
#define SWDIO_PIN 3

// Default SWD clock rate used when initially probing the device. It will be increased later for known devices that
// support higher clock rates.
#define DEFAULT_SWD_CLOCK_RATE 8000000

// The maximum length of commands accepted from the user.
#define MAX_COMMAND_LENGTH 255

// A printf() replacement that uses local indent variable to add indentation to the left of the line of text.
// It is used when walking the ROM table hierarchy to make nesting more noticeable.
#define PRINTF(FORMAT, ...) printf("%*s" FORMAT, (indent*4), "", ##__VA_ARGS__)

// Macro to remove AP_DP_RO/WO/RW bits from SWD register offsets so that existing CoreSight addresses can be reused.
#define CORESIGHT_REG_OFFSET(X) ((X) & ~(0x3 << 30))

// CoreSight register offsets.
static const uint32_t ITCTRL_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_ITCTRL);
static const uint32_t CLAIMSET_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_CLAIMSET);
static const uint32_t CLAIMCLR_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_CLAIMCLR);
static const uint32_t DEVAFF0_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_DEVAFF0);
static const uint32_t DEVAFF1_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_DEVAFF1);
static const uint32_t LAR_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_LAR);
static const uint32_t LSR_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_LSR);
static const uint32_t AUTHSTATUS_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_AUTHSTATUS);
static const uint32_t DEVARCH_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_DEVARCH);
static const uint32_t DEVID2_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_DEVID2);
static const uint32_t DEVID1_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_DEVID1);
static const uint32_t DEVID_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_DEVID);
static const uint32_t DEVTYPE_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_DEVTYPE);
static const uint32_t PIDR4_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_PIDR4);
static const uint32_t PIDR5_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_PIDR5);
static const uint32_t PIDR6_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_PIDR6);
static const uint32_t PIDR7_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_PIDR7);
static const uint32_t PIDR0_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_PIDR0);
static const uint32_t PIDR1_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_PIDR1);
static const uint32_t PIDR2_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_PIDR2);
static const uint32_t PIDR3_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_PIDR3);
static const uint32_t CIDR0_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_CIDR0);
static const uint32_t CIDR1_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_CIDR1);
static const uint32_t CIDR2_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_CIDR2);
static const uint32_t CIDR3_OFFSET = CORESIGHT_REG_OFFSET(SWD::APv2_CIDR3);

// Class 1 ROM Table Registers.
static const uint32_t MEMTYPE_OFFSET = 0xFCC;


// Structure to represent the user command, including it being broken down into tokens.
struct CommandTokens
{
    char*  tokens[16];
    size_t tokenCount;
    char   command[MAX_COMMAND_LENGTH + 1];
};

// Structure used to track whether AP is a MEM-AP or CoreSight ROM Table.
struct ComponentType
{
    ComponentType()
    {
        _class = 0;
        isMEMAP = false;
        isROMTable = false;
    }

    uint32_t _class;
    bool     isMEMAP;
    bool     isROMTable;
};

struct RomEntry
{
    int32_t offset;
    uint32_t powerDomainId;
    bool isPresent;
    bool isLastEntry;
    bool isPowerDomainValid;
};


static SWD           g_swdBus;
static bool          g_isSwdConnected = false;
static char          g_commandBuffer[MAX_COMMAND_LENGTH + 1];
static size_t        g_commandBufferIndex = 0;
static char          g_command[MAX_COMMAND_LENGTH + 1];
volatile static bool g_commandIsFull = false;
static bool          g_verbose = false;


class IRegisterReader
{
    public:
        virtual bool read(uint32_t address, uint32_t* pData) = 0;
        virtual void restore() = 0;

        static IRegisterReader* pReader;
};
IRegisterReader* IRegisterReader::pReader = NULL;

class ApRegisterReader : public IRegisterReader
{
    public:
        ApRegisterReader(uint32_t selectAP)
        {
            m_selectAP = selectAP;
            restore();
            m_pPrevReader = IRegisterReader::pReader;
            IRegisterReader::pReader = this;
        }

        ~ApRegisterReader()
        {
            IRegisterReader::pReader = m_pPrevReader;
            if (IRegisterReader::pReader != NULL)
            {
                IRegisterReader::pReader->restore();
            }
        }

        virtual bool read(uint32_t address, uint32_t* pData)
        {
            uint32_t dummy = 0;
            bool result = g_swdBus.readAP(SWD::AP_DP_RO | address, &dummy);
            if (!result)
            {
                return result;
            }

            result = g_swdBus.readDP(SWD::DP_RDBUFF, pData);
            return result;
        }

        virtual void restore()
        {
            g_swdBus.selectAP(m_selectAP);
        }

    protected:
        IRegisterReader* m_pPrevReader;
        uint32_t         m_selectAP;
};

class MemRegisterReader : public IRegisterReader
{
    public:
        MemRegisterReader(uint32_t baseAddress)
        {
            m_baseAddress = baseAddress;
            m_pPrevReader = IRegisterReader::pReader;
            IRegisterReader::pReader = this;
        }

        ~MemRegisterReader()
        {
            IRegisterReader::pReader = m_pPrevReader;
            if (IRegisterReader::pReader != NULL)
            {
                IRegisterReader::pReader->restore();
            }
        }

        virtual bool read(uint32_t address, uint32_t* pData)
        {
            return g_swdBus.readTargetMemory(m_baseAddress + address, pData, sizeof(*pData), SWD::TRANSFER_32BIT);
        }

        virtual void restore()
        {
        }

    protected:
        IRegisterReader* m_pPrevReader;
        uint32_t         m_baseAddress;
};

// Forward Function Declarations
static void charsAvailableCallback(void* pvParam);
static void waitForNextCommand();
static void handleNextCommand();
static bool parseTokens(CommandTokens* pCmdTokens);
static void handleConnectCommand(const CommandTokens* pCmdTokens);
static void checkParameterCount(const CommandTokens* pCmdTokens, size_t requiredParameterCount);
static bool attemptSwdAttach();
bool controlPower(bool systemPower, bool debugPower);
static void handleTargetSearchCommand(const CommandTokens* pCmdTokens);
static void handleApSearchCommand(const CommandTokens* pCmdTokens);
static bool readAP(uint32_t address, uint32_t* pData);
static void displayIDR(int ident, uint32_t idr, ComponentType& componentType);
static const char* decodeApClass(uint32_t _class);
static const char* decodeApClassType(uint32_t type, uint32_t _class);
static uint32_t dumpRegField(int indent, const char* pFieldName, uint32_t reg, uint32_t loBit, uint32_t hiBit, bool addNewLine);
static const char* decodeDesigner(uint32_t designer);
static void handleDumpCommand(const CommandTokens* pCmdTokens);
static uint32_t dumpDPIDR();
static void dumpDPIDR1();
static uint32_t dumpBASEPTRs();
static void dumpCTRL_STAT();
static void dumpDLCR();
static void dumpDLPIDR();
static void dumpAP(int indent, uint32_t dpVersion, uint32_t apAddress);
static void dumpTARGETID();
static void dumpIDR(int indent, uint32_t dpVersion, ComponentType& componentType);
static void dumpCoreSightComponent(int indent, uint32_t baseAddress, ComponentType& componentType, bool walkRomTable);
static void dumpCIDR(int indent, ComponentType& componentType);
static bool readRegister(uint32_t address, uint32_t* pData);
static void checkPreamble(const char* pName, uint32_t actual, uint32_t expected);
static void dumpPIDR(int ident);
static void dumpDEVARCH(int indent, ComponentType& componentType);
static const char* decodeArchitectureID(uint32_t archid);
static uint32_t dumpRegister(int indent, const char* pRegName, uint32_t regIndex);
static uint32_t dumpApRegister(int indent, const char* pRegName, uint32_t regIndex);
static void dumpAUTHSTATUS(int indent);
static void dumpAuthStatusBits(int indent, const char* pFieldName, uint32_t reg, uint32_t loBit, uint32_t hiBit);
static void dumpDEVTYPE(int ident);
static uint32_t dumpCFG(int indent, uint32_t dpVersion);
static const char* decodeMeciWidth(uint32_t meciWidth);
static const char* decodeTarInc(uint32_t tarInc);
static const char* decodeErr(uint32_t err);
static const char* decodeDarSize(uint32_t darsize);
static bool dumpCFG1(int indent, uint32_t dpVersion);
static const char* decodeTag0Gran(uint32_t tag0Gran);
static const char* decodeTag0Size(uint32_t tag0Size);
static void dumpCSW(int indent, uint32_t dpVersion, uint32_t cfg, bool isTaggingImplemented);
static const char* decodeRmeEn(uint32_t rmeen);
static const char* decodeMode(uint32_t mode);
static const char* decodeAddrInc(uint32_t addrInc);
static const char* decodeSize(uint32_t size);
static void dumpApRomTable(int indent, uint32_t dpVersion, uint32_t apAddress);
static RomEntry parseClass9Entry(uint32_t entry);
static void dumpMemRomTable(int indent, uint32_t baseAddress, uint32_t _class);
static void dumpClass1MemRomTable(int indent, uint32_t baseAddress);
static void dumpMEMTYPE(int indent);
static RomEntry parseClass1Entry(uint32_t entry);
static void dumpClass9MemRomTable(int indent, uint32_t baseAddress);
static void handleHelpCommand(const CommandTokens* pCmdTokens);
static void handleVerboseCommand(const CommandTokens* pCmdTokens);


int main()
{
    // Wait for terminal program to connect.
    stdio_init_all();
    while (!stdio_usb_connected())
    {
      sleep_ms(250);
    }
    stdio_set_chars_available_callback(charsAvailableCallback, NULL);

    if (!g_swdBus.init(DEFAULT_SWD_CLOCK_RATE, SWCLK_PIN, SWDIO_PIN))
    {
        logError("Failed to initialize the SWD port.");
        return -1;
    }

    // Help command shows the supported commands to the user.
    handleHelpCommand(NULL);

    // Enter main command handling loop.
    while (true)
    {
        // Prompt the user for the next command.
        printf("$ ");

        // Wait for next user command and then handle it.
        waitForNextCommand();
        handleNextCommand();
    }

    return 0;
}


static void charsAvailableCallback(void* pvParam)
{
    const int BELL = 7;
    const int BACKSPACE = '\b';
    const int DEL = 127;

    int ch = PICO_ERROR_TIMEOUT;
    while (g_commandBufferIndex < sizeof(g_commandBuffer)-1 && (ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT)
    {
        if (g_commandBufferIndex >= sizeof(g_commandBuffer))
        {
            // Eat the character and send a BEL character to terminal if command buffer is full.
            putchar_raw(BELL);
            continue;
        }

        switch (ch) {
            case '\n':
                break;
            case '\r':
                g_commandBuffer[g_commandBufferIndex++] = '\0';
                if (g_commandIsFull)
                {
                    // Eat the latest command if last command hasn't been processed yet.
                    putchar_raw(BELL);
                }
                else
                {
                    // Copy buffered input into g_command.
                    strcpy(g_command, g_commandBuffer);
                    g_commandIsFull = true;
                }
                // Reset to start buffering next command.
                g_commandBufferIndex = 0;
                putchar_raw('\r');
                putchar_raw('\n');
                break;
            case BACKSPACE:
            case DEL:
                if (g_commandBufferIndex == 0)
                {
                    // Don't backspace past the start of the command buffer.
                    putchar_raw(BELL);
                }
                else
                {
                    // Handle backspace by removing last character from command buffer and erasing last character from
                    // user terminal as well.
                    g_commandBufferIndex--;
                    putchar_raw(BACKSPACE);
                    putchar_raw(' ');
                    putchar_raw(BACKSPACE);
                }
                break;
            default:
                g_commandBuffer[g_commandBufferIndex++] = (char)ch;
                putchar_raw(ch);
                break;
        }
    }
}

static void waitForNextCommand()
{
    while (!g_commandIsFull)
    {
    }
}

// User commands supported by this program.
struct CommandHandler
{
    const char* pCommand;
    const char* pParameterDescription;
    const char* pDescription;
    void (*pCommandHandler)(const CommandTokens* pCmdTokens);
};

static const CommandHandler g_commandHandlers[] =
{
    { "connect", "[TARGET_ID | default]", "Connects to SWD target (the one specified by TARGET_ID or default otherwise).", handleConnectCommand },
    { "dump", "[AP_Selection]", "Dumps DP/AP registers and ROM Tables of current SWD target.", handleDumpCommand },
    { "ap_search", NULL, "Searches through all valid AP selection values.", handleApSearchCommand },
    { "target_search", NULL, "Searches through all valid SWD TARGETSEL values.", handleTargetSearchCommand },
    { "verbose", "on | off", "Only dump all CoreSight Component Registers when VERBOSE is enabled.", handleVerboseCommand },
    { "help", NULL, "Displays this help text.", handleHelpCommand },
};

static void handleNextCommand()
{
    CommandTokens cmdTokens;

    // Make a local copy of the command and free up g_command for the next one.
    assert ( sizeof(g_command) <= sizeof(cmdTokens.command) );
    strcpy(cmdTokens.command, g_command);
    g_commandIsFull = false;

    if (!parseTokens(&cmdTokens))
    {
        return;
    }

    for (size_t i = 0 ; i < count_of(g_commandHandlers) ; i++)
    {
        if (0 == strcasecmp(cmdTokens.tokens[0], g_commandHandlers[i].pCommand))
        {
            g_commandHandlers[i].pCommandHandler(&cmdTokens);
            return;
        }
    }
    handleHelpCommand(&cmdTokens);
}

static bool parseTokens(CommandTokens* pCmdTokens)
{
    pCmdTokens->tokenCount = 0;
    char* pCmd = &pCmdTokens->command[0];
    while (*pCmd != '\0' && pCmdTokens->tokenCount < count_of(pCmdTokens->tokens))
    {
        // Skip over whitespace.
        while (*pCmd != '\0' && isspace(*pCmd))
        {
            pCmd++;
        }

        // Record the start of this token.
        if (*pCmd != '\0')
        {
            pCmdTokens->tokens[pCmdTokens->tokenCount++] = pCmd;
        }

        // Skip over to next whitespace token.
        while (*pCmd != '\0' && !isspace(*pCmd))
        {
            pCmd++;
        }

        // NULL terminate the current token by overwriting the first whitespace character.
        if (*pCmd != '\0')
        {
            *pCmd = '\0';
            pCmd++;
        }
    }

    return pCmdTokens->tokenCount > 0;
}

static void handleConnectCommand(const CommandTokens* pCmdTokens)
{
    bool targetFound = false;

    g_swdBus.disableErrorLogging();
    if (pCmdTokens->tokenCount  > 1)
    {
        checkParameterCount(pCmdTokens, 1);

        if (0 == strcasecmp(pCmdTokens->tokens[1], "default"))
        {
            g_swdBus.switchJtagIntoDormantMode();
            g_swdBus.switchSwdOutOfDormantMode();
            targetFound = g_swdBus.sendLineReset();
        }
        else
        {
            uint32_t targetID = strtoul(pCmdTokens->tokens[1], NULL, 16);
            printf("Attempting to connect to target 0x%08lX\n", targetID);
            g_swdBus.switchJtagIntoDormantMode();
            g_swdBus.switchSwdOutOfDormantMode();
            targetFound = g_swdBus.selectSwdTarget((SWD::DPv2Targets)targetID);
        }
    }
    else
    {
        checkParameterCount(pCmdTokens, 0);
        targetFound = attemptSwdAttach();
    }
    g_swdBus.enableErrorLogging();

    if (targetFound)
    {
        printf("Successfully connected to SWD target.\n");
    }
    else
    {
        printf("Failed to connect to SWD target.\n");
    }
    g_isSwdConnected = targetFound;

    controlPower(true, true);
}

static void checkParameterCount(const CommandTokens* pCmdTokens, size_t requiredParameterCount)
{
    if (pCmdTokens->tokenCount > requiredParameterCount + 1)
    {
        printf("Ignoring extra parameters you provided on the command line.\n");
    }
}

static bool attemptSwdAttach()
{
    // Search through all known SWD DPv2 targets to see if any are found.
    // If that fails, try detecting SWD targets which don't go dormant, after making to switch SWJ-DP targets into SWD
    // mode.
    if (g_swdBus.searchForKnownSwdTarget())
    {
        // Have found one of the SWD DPv2 targets known by this debugger.
        printf("Found DPv2 SWD Target=0x%08X with DPIDR=0x%08lX\n", g_swdBus.getTarget(), g_swdBus.getDPIDR());
    }
    else if (g_swdBus.sendJtagToSwdSequence())
    {
        // Have found a non-dormant SWD target.
        printf("Found SWD Target with DPIDR=0x%08lX\n", g_swdBus.getDPIDR());
    }
    else
    {
        logError("No SWD Targets found.");
        return false;
    }
    return true;
}

bool controlPower(bool systemPower, bool debugPower)
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

    printf("Powering up debug access port...\n");
    absolute_time_t endTime = make_timeout_time_us(1000);
    do
    {
        uint32_t ctrlStat;

        bool result = g_swdBus.readDP(SWD::DP_CTRL_STAT, &ctrlStat);
        if (!result)
        {
            printf("Failed to read DP's CTRL/STAT register while enabling power.\n");
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
            result = g_swdBus.writeDP(SWD::DP_CTRL_STAT, ctrlStat);
            if (!result)
            {
                printf("Failed to write DP's CTRL/STAT register while enabling power.\n");
                return false;
            }

            powerUpRequestSent = true;
        }
    } while (absolute_time_diff_us(get_absolute_time(), endTime) > 0);
    if (!poweredUp)
    {
        printf("TIMEOUT!\n");
        return false;
    }
    return true;
}

static void handleTargetSearchCommand(const CommandTokens* pCmdTokens)
{
    checkParameterCount(pCmdTokens, 0);

    g_swdBus.disableErrorLogging();
        printf("Searching through all possible SWD TARGETSEL values.\n");
        printf("This will take around half an hour to complete...\n");
        g_swdBus.switchJtagIntoDormantMode();
        g_swdBus.switchSwdOutOfDormantMode();

        SWD::DPv2Targets target;
        bool result = g_swdBus.findFirstSwdTarget(&target);
        while (result == true)
        {
            uint32_t TARGETSEL = (uint32_t)target;
            printf("  TARGETSEL = 0x%08lX\n", TARGETSEL);
            uint32_t designer = dumpRegField(0, "TDESIGNER", TARGETSEL, 1, 11, false);
                printf(" (%s)\n", decodeDesigner(designer));
            dumpRegField(0, "TPARTNO", TARGETSEL, 12, 27, true);
            dumpRegField(0, "TINSTANCE", TARGETSEL, 28, 31, true);

            result = g_swdBus.findNextSwdTarget(&target);
        }
    g_swdBus.enableErrorLogging();
}

static void handleApSearchCommand(const CommandTokens* pCmdTokens)
{
    checkParameterCount(pCmdTokens, 0);

    uint32_t maxApCount = 256;
    uint32_t dpidr = g_swdBus.getDPIDR();
    uint32_t dpVersion = (dpidr >> 12) & 0xF;
    if (dpVersion >= 3)
    {
        uint32_t DPIDR1 = 0;
        bool result = g_swdBus.readDP(SWD::DP_DPIDR1, &DPIDR1);
        if (!result)
        {
            printf("Failed to read DP's DPIDR1 register to obtain ASIZE.\n");
            return;
        }
        uint32_t asize = DPIDR1 & 0x7F;
        maxApCount = 1 << (asize - 12);
    }

    g_swdBus.disableErrorLogging();
        printf("Searching through all possible AP selection values.\n");
        for (uint32_t i = 0 ; i < maxApCount ; i++)
        {
            ApRegisterReader reader(i);

            uint32_t IDR = SWD::APv1_IDR;
            if (dpVersion >= 3)
            {
                IDR = SWD::APv2_IDR;
            }

            uint32_t idrVal = 0;
            bool result = readAP(IDR, &idrVal);
            if (!result)
            {
                printf("Failed to read AP's IDR register for AP address %lu.\n", i);
                continue;
            }

            if (idrVal != 0)
            {
                printf("Found an AP at address %lu.\n", i);
                ComponentType componentType;
                displayIDR(0, idrVal, componentType);
            }
        }
    g_swdBus.enableErrorLogging();
}

static bool readAP(uint32_t address, uint32_t* pData)
{
    assert ( (address & (0x3 << 30)) );

    uint32_t dummy = 0;
    bool result = g_swdBus.readAP(SWD::AP_DP_RO | address, &dummy);
    if (!result)
    {
        return result;
    }

    return g_swdBus.readDP(SWD::DP_RDBUFF, pData);
}

static void displayIDR(int indent, uint32_t idrVal, ComponentType& componentType)
{
    PRINTF("  IDR = 0x%08lX\n", idrVal);
    if (idrVal == 0)
    {
        return;
    }

    dumpRegField(indent, "REVISION", idrVal, 28, 31, true);
    uint32_t designer = dumpRegField(indent, "DESIGNER", idrVal, 17, 27, false);
        printf(" (%s)\n", decodeDesigner(designer));
    uint32_t _class = dumpRegField(indent, "CLASS", idrVal, 13, 16, false);
        printf(" (%s)\n", decodeApClass(_class));
    dumpRegField(indent, "VARIANT", idrVal, 4, 7, true);
    uint32_t type = dumpRegField(indent, "TYPE", idrVal, 0, 3, false);
        printf(" (%s)\n", decodeApClassType(type, _class));

    componentType.isMEMAP |= (_class == 0x8);
}

static const char* decodeApClass(uint32_t _class)
{
    switch (_class)
    {
        case 0x0:
            return "No defined class";
        case 0x8:
            return "MEM-AP";
        default:
            return "Unknown";
    }
}

static const char* decodeApClassType(uint32_t type, uint32_t _class)
{
    const struct TYPE {
        const char* pName;
        uint32_t type;
        uint32_t _class;
    } knownClassTypes[] = {
        { "JTAG connection", 0x0, 0x0 },
        { "AMBA AHB3 bus", 0x1, 0x8 },
        { "AMBA APB2 or APB3 bus", 0x2, 0x8 },
        { "AMBA AXI3 or AXI4 bus, with optional ACE-Lite support", 0x4, 0x8 },
        { "AMBA AHB5 bus", 0x5, 0x8 },
        { "AMBA APB4 and APB5 bus", 0x6, 0x8 },
        { "AMBA AXI5 bus", 0x7, 0x8 },
        { "AMBA AHB5 with enhanced HPROT", 0x8, 0x8 },
    };

    for (size_t i = 0 ; i < count_of(knownClassTypes) ; i++) {
        const TYPE* pType = &knownClassTypes[i];
        if (pType->type == type && pType->_class == _class) {
            return pType->pName;
        }
    }
    return "Reserved";
}

static uint32_t dumpRegField(int indent, const char* pFieldName, uint32_t reg, uint32_t loBit, uint32_t hiBit, bool addNewLine)
{
    uint32_t bitCount = hiBit - loBit + 1;
    uint32_t bitMask = (1 << bitCount) - 1;
    uint32_t value = (reg >> loBit) & bitMask;
    PRINTF("    %s = 0x%lX%s", pFieldName, value, addNewLine ? "\n" : "");

    return value;
}

static const char* decodeDesigner(uint32_t designer)
{
    const struct ID {
        const char* pName;
        uint32_t contCode;
        uint32_t idCode;
    } knownDesigners[] = {
        { "ARM Ltd.", 4, 59 },
        { "Raspberry Pi Trading Ltd.", 9, 19 },
    };

    uint32_t continuationCode = (designer >> 7) & 0xF;
    uint32_t identificationCode = designer & 0x7F;
    for (size_t i = 0 ; i < count_of(knownDesigners) ; i++) {
        const ID* pID = &knownDesigners[i];
        if (pID->contCode == continuationCode && pID->idCode == identificationCode) {
            return pID->pName;
        }
    }

    static char buffer[16];
    snprintf(buffer, sizeof(buffer), "%ld.%ld", continuationCode, identificationCode);
    return buffer;
}

static void handleDumpCommand(const CommandTokens* pCmdTokens)
{
    if (!g_isSwdConnected)
    {
        printf("Use 'connect' command first.\n");
        return;
    }

    bool useDpBasePtr = true;
    uint32_t apAddress = 0;
    if (pCmdTokens->tokenCount  > 1)
    {
        checkParameterCount(pCmdTokens, 1);

        apAddress = strtoul(pCmdTokens->tokens[1], NULL, 16);
        useDpBasePtr = false;
    }

    printf("Dumping SWD target DP registers.\n");
    uint32_t dpVersion = dumpDPIDR();
    if (dpVersion >= 3)
    {
        dumpDPIDR1();
        uint32_t baseptr = dumpBASEPTRs();

        const uint32_t BASEPTR_VALID_Bit = 0x1;
        if (useDpBasePtr && (baseptr & BASEPTR_VALID_Bit))
        {
            apAddress = baseptr >> 12;
        }
    }
    dumpCTRL_STAT();
    dumpDLCR();
    dumpDLPIDR();
    dumpTARGETID();

    dumpAP(0, dpVersion, apAddress);
}

static uint32_t dumpDPIDR()
{
    uint32_t DPIDR = 0;
    bool result = g_swdBus.readDP(SWD::DP_DPIDR, &DPIDR);
    if (!result)
    {
        printf("Failed to read DP's DPIDR register.\n");
        return 0;
    }
    printf("  DPIDR = 0x%08lX\n", DPIDR);
    dumpRegField(0, "REVISION", DPIDR, 28, 31, true);
    dumpRegField(0, "PARTNO", DPIDR, 20, 27, true);
    uint32_t minDP = dumpRegField(0,"MIN", DPIDR, 16, 16, false);
        printf(" (MINDP %simplemented)\n", minDP ? "" : "not ");
    uint32_t dpVersion = dumpRegField(0, "VERSION", DPIDR, 12, 15, false);
        printf(" (DPv%ld)\n", dpVersion);
    uint32_t designer = dumpRegField(0, "DESIGNER", DPIDR, 1, 11, false);
        printf(" (%s)\n", decodeDesigner(designer));

    return dpVersion;
}

static void dumpDPIDR1()
{
    uint32_t DPIDR1 = 0;
    bool result = g_swdBus.readDP(SWD::DP_DPIDR1, &DPIDR1);
    if (!result)
    {
        printf("Failed to read DP's DPIDR1 register.\n");
        return;
    }
    printf("  DPIDR1 = 0x%08lX\n", DPIDR1);
    dumpRegField(0, "ERRMODE", DPIDR1, 7, 7, true);
    uint32_t asize = dumpRegField(0, "ASIZE", DPIDR1, 0, 6, false);
        printf(" (%lu-bit address)\n", asize);
}

static uint32_t dumpBASEPTRs()
{
    uint32_t BASEPTR0 = 0;
    uint32_t BASEPTR1 = 0;
    bool result = g_swdBus.readDP(SWD::DP_BASEPTR0, &BASEPTR0);
    if (!result)
    {
        printf("Failed to read DP's BASEPTR0 register.\n");
        return 0x0;
    }
    result = g_swdBus.readDP(SWD::DP_BASEPTR1, &BASEPTR1);
    if (!result)
    {
        printf("Failed to read DP's BASEPTR1 register.\n");
        return 0x0;
    }
    printf("  BASEPTR1:BASEPTR0 = 0x%08lX:0x%08lX\n", BASEPTR1, BASEPTR0);
    uint32_t valid = dumpRegField(0, "VALID", BASEPTR0, 0, 0, true);
    if (valid)
    {
        printf("    PTR = 0x%08lX%08lX\n", BASEPTR1, BASEPTR0 & 0xFFFFF000);
    }
    return BASEPTR0;
}

static void dumpCTRL_STAT()
{
    uint32_t CTRL_STAT = 0;
    bool result = g_swdBus.readDP(SWD::DP_CTRL_STAT, &CTRL_STAT);
    if (!result)
    {
        printf("Failed to read DP's CTRL/STAT register.\n");
        return;
    }

    printf("  CTRL/STAT = 0x%08lX\n", CTRL_STAT);
    dumpRegField(0, "CSYSPWRUPACK", CTRL_STAT, 31, 31, true);
    dumpRegField(0, "CSYSPWRUPREQ", CTRL_STAT, 30, 30, true);
    dumpRegField(0, "CDBGPWRUPACK", CTRL_STAT, 29, 29, true);
    dumpRegField(0, "CDBGPWRUPREQ", CTRL_STAT, 28, 28, true);
    dumpRegField(0, "CDBGRSTACK", CTRL_STAT, 27, 27, true);
    dumpRegField(0, "CDBGRSTREQ", CTRL_STAT, 26, 26, true);
    dumpRegField(0, "ERRMODE", CTRL_STAT, 24, 24, true);
    dumpRegField(0, "TRNCNT", CTRL_STAT, 12, 23, true);
    uint32_t maskLane = dumpRegField(0, "MASKLANE", CTRL_STAT, 8, 11, false);
        printf(" (0b%c%c%c%c)\n",
            (maskLane & (1 << 3)) ? '1' : '0',
            (maskLane & (1 << 2)) ? '1' : '0',
            (maskLane & (1 << 1)) ? '1' : '0',
            (maskLane & (1 << 0)) ? '1' : '0');
    dumpRegField(0, "WDATAERR", CTRL_STAT, 7, 7, true);
    dumpRegField(0, "READOK", CTRL_STAT, 6, 6, true);
    dumpRegField(0, "STICKYERR", CTRL_STAT, 5, 5, true);
    dumpRegField(0, "STICKYCMP", CTRL_STAT, 4, 4, true);
    uint32_t trnMode = dumpRegField(0, "TRNMODE", CTRL_STAT, 2, 3, false);
        switch (trnMode)
        {
            case 0:
                printf(" (Normal operation)\n");
                break;
            case 1:
                printf(" (Pushed-verify mode)\n");
                break;
            case 2:
                printf(" (Pushed-compare mode)\n");
                break;
            case 3:
            default:
                printf(" (Reserved)\n");
                break;
        }
    dumpRegField(0, "STICKYORUN", CTRL_STAT, 1, 1, true);
    dumpRegField(0, "ORUNDETECT", CTRL_STAT, 0, 0, true);
}

static void dumpDLCR()
{
    uint32_t DLCR = 0;
    bool result = g_swdBus.readDP(SWD::DP_DLCR, &DLCR);
    if (!result)
    {
        printf("Failed to read DP's DLCR register.\n");
        return;
    }

    printf("  DLCR = 0x%08lX\n", DLCR);
    uint32_t turnAround = dumpRegField(0, "TURNROUND", DLCR, 8, 9, false);
        printf(" (%lu data period%s)\n", turnAround + 1, turnAround > 0 ? "s" : "");
}

static void dumpDLPIDR()
{
    uint32_t DLPIDR = 0;
    bool result = g_swdBus.readDP(SWD::DP_DLPIDR, &DLPIDR);
    if (!result)
    {
        printf("Failed to read DP's DLPIDR register.\n");
        return;
    }

    printf("  DLPIDR = 0x%08lX\n", DLPIDR);
    dumpRegField(0, "TINSTANCE", DLPIDR, 28, 31, true);
    dumpRegField(0, "PROTVSN", DLPIDR, 0, 3, true);
}

static void dumpAP(int indent, uint32_t dpVersion, uint32_t apAddress)
{
    ComponentType componentType;

    PRINTF("Dumping AP registers at address 0x%lX.\n", apAddress);
    ApRegisterReader reader(apAddress);
    dumpIDR(indent, dpVersion, componentType);
    if (dpVersion >= 3)
    {
        dumpCoreSightComponent(indent, 0x00000000, componentType, false);
    }

    if (componentType.isMEMAP)
    {
        PRINTF("Dumping MEM-AP registers at address 0x%lX.\n", apAddress);
        uint32_t cfg = dumpCFG(indent, dpVersion);
        bool isTaggingSupported = dumpCFG1(indent, dpVersion);
        dumpCSW(indent, dpVersion, cfg, isTaggingSupported);
        uint32_t base = dumpApRegister(indent, "BASE", (dpVersion >= 3) ? SWD::APv2_BASE : SWD::APv1_BASE);

        const uint32_t BASE_PRESENT_Bit = 1 << 0;
        const uint32_t BASE_ADDRESS_Mask = 0xFFFFF000;
        if (base & BASE_PRESENT_Bit)
        {
            uint32_t baseAddress = base & BASE_ADDRESS_Mask;

            indent++;
            ComponentType componentType;
            MemRegisterReader reader(baseAddress);
            dumpCoreSightComponent(indent, baseAddress, componentType, true);
        }
    }
    else if (componentType.isROMTable)
    {
        dumpApRomTable(indent, dpVersion, apAddress);
    }
}

static void dumpTARGETID()
{
    uint32_t TARGETID = 0;
    bool result = g_swdBus.readDP(SWD::DP_TARGETID, &TARGETID);
    if (!result)
    {
        printf("Failed to read DP's TARGETID register.\n");
        return;
    }

    printf("  TARGETID = 0x%08lX\n", TARGETID);
    dumpRegField(0, "TREVISION", TARGETID, 28, 31, true);
    dumpRegField(0, "TPARTNO", TARGETID, 12, 27, true);
    uint32_t designer = dumpRegField(0, "TDESIGNER", TARGETID, 1, 11, false);
        printf(" (%s)\n", decodeDesigner(designer));
}

static void dumpIDR(int indent, uint32_t dpVersion, ComponentType& ComponentType)
{
    uint32_t IDR = SWD::APv1_IDR;
    if (dpVersion >= 3)
    {
        IDR = SWD::APv2_IDR;
    }

    uint32_t idrVal = 0;
    bool result = readAP(IDR, &idrVal);
    if (!result)
    {
        printf("Failed to read AP's IDR register.\n");
    }
    displayIDR(indent, idrVal, ComponentType);
}

static void dumpCoreSightComponent(int indent, uint32_t baseAddress, ComponentType& componentType, bool walkRomTable)
{
    if (walkRomTable)
    {
        PRINTF("Dumping CoreSight component at address 0x%08lX\n", baseAddress);
    }

    dumpCIDR(indent, componentType);
    dumpPIDR(indent);
    if (componentType._class == 9)
    {
        dumpDEVARCH(indent, componentType);
        dumpDEVTYPE(indent);
        if (g_verbose)
        {
            dumpAUTHSTATUS(indent);
            dumpRegister(indent, "ITCTRL", ITCTRL_OFFSET);
            dumpRegister(indent, "CLAIMSET", CLAIMSET_OFFSET);
            dumpRegister(indent, "CLAIMCLR", CLAIMCLR_OFFSET);
            dumpRegister(indent, "DEVAFF0", DEVAFF0_OFFSET);
            dumpRegister(indent, "DEVAFF1", DEVAFF1_OFFSET);
            dumpRegister(indent, "LSR", LSR_OFFSET);
            dumpRegister(indent, "DEVID", DEVID_OFFSET);
            dumpRegister(indent, "DEVID1", DEVID1_OFFSET);
            dumpRegister(indent, "DEVID2", DEVID2_OFFSET);
        }
    }

    if (walkRomTable && componentType.isROMTable)
    {
        dumpMemRomTable(indent, baseAddress, componentType._class);
    }
}

static void dumpCIDR(int indent, ComponentType& componentType)
{
    uint32_t CIDR0 = 0;
    uint32_t CIDR1 = 0;
    uint32_t CIDR2 = 0;
    uint32_t CIDR3 = 0;
    bool result = true;
    result &= readRegister(CIDR0_OFFSET, &CIDR0);
    result &= readRegister(CIDR1_OFFSET, &CIDR1);
    result &= readRegister(CIDR2_OFFSET, &CIDR2);
    result &= readRegister(CIDR3_OFFSET, &CIDR3);
    if (!result)
    {
        printf("Failed to read AP's CIDR registers.\n");
        return;
    }
    PRINTF("  CIDR0 = 0x%08lX\n", CIDR0);
    PRINTF("  CIDR1 = 0x%08lX\n", CIDR1);
    PRINTF("  CIDR2 = 0x%08lX\n", CIDR2);
    PRINTF("  CIDR3 = 0x%08lX\n", CIDR3);

    uint32_t PRMBL_0 = CIDR0 & 0xFF;
    uint32_t PRMBL_1 = CIDR1 & 0xF;
    uint32_t PRMBL_2 = CIDR2 & 0xFF;
    uint32_t PRMBL_3 = CIDR3 & 0xFF;
    checkPreamble("CIDR0_RES0", CIDR0 & 0xFFFFFF00, 0);
    checkPreamble("CIDR1_RES0", CIDR1 & 0xFFFFFF00, 0);
    checkPreamble("CIDR2_RES0", CIDR2 & 0xFFFFFF00, 0);
    checkPreamble("CIDR3_RES0", CIDR3 & 0xFFFFFF00, 0);
    checkPreamble("PRMBL_0", PRMBL_0, 0x0D);
    checkPreamble("PRMBL_1", PRMBL_1, 0x0);
    checkPreamble("PRMBL_2", PRMBL_2, 0x05);
    checkPreamble("PRMBL_3", PRMBL_3, 0xB1);
    componentType._class = dumpRegField(indent, "CLASS", CIDR1, 4, 7, true);
    if (componentType._class == 1)
    {
        componentType.isROMTable |= true;
    }
}

static bool readRegister(uint32_t address, uint32_t* pData)
{
    assert ( IRegisterReader::pReader != NULL );
    assert ( (address & (0x3 << 30)) == 0 );
    return IRegisterReader::pReader->read(address, pData);
}

static void checkPreamble(const char* pName, uint32_t actual, uint32_t expected)
{
    if (actual != expected)
    {
        printf("    %s isn't the expected value. Actual = 0x%lX  Expected = 0x%lX\n", pName, actual, expected);
    }
}

static void dumpPIDR(int indent)
{
    uint32_t PIDR4 = 0;
    uint32_t PIDR5 = 0;
    uint32_t PIDR6 = 0;
    uint32_t PIDR7 = 0;
    uint32_t PIDR0 = 0;
    uint32_t PIDR1 = 0;
    uint32_t PIDR2 = 0;
    uint32_t PIDR3 = 0;
    bool result = true;
    result &= readRegister(PIDR4_OFFSET, &PIDR4);
    result &= readRegister(PIDR5_OFFSET, &PIDR5);
    result &= readRegister(PIDR6_OFFSET, &PIDR6);
    result &= readRegister(PIDR7_OFFSET, &PIDR7);
    result &= readRegister(PIDR0_OFFSET, &PIDR0);
    result &= readRegister(PIDR1_OFFSET, &PIDR1);
    result &= readRegister(PIDR2_OFFSET, &PIDR2);
    result &= readRegister(PIDR3_OFFSET, &PIDR3);
    if (!result)
    {
        printf("Failed to read AP's PIDR registers.\n");
        return;
    }
    PRINTF("  PIDR4 = 0x%08lX\n", PIDR4);
    PRINTF("  PIDR5 = 0x%08lX\n", PIDR5);
    PRINTF("  PIDR6 = 0x%08lX\n", PIDR6);
    PRINTF("  PIDR7 = 0x%08lX\n", PIDR7);
    PRINTF("  PIDR0 = 0x%08lX\n", PIDR0);
    PRINTF("  PIDR1 = 0x%08lX\n", PIDR1);
    PRINTF("  PIDR2 = 0x%08lX\n", PIDR2);
    PRINTF("  PIDR3 = 0x%08lX\n", PIDR3);

    uint32_t DES_2 = PIDR4 & 0xF;
    uint32_t PART_0 = PIDR0 & 0xFF;
    uint32_t DES_0 = (PIDR1 >> 4) & 0xF;
    uint32_t PART_1 = PIDR1 & 0xF;
    uint32_t DES_1 = PIDR2 & 0x7;
    uint32_t designer = (DES_2 << 7) | (DES_1 << 4) | DES_0;
    uint32_t partno = (PART_1 << 8) | PART_0;
    checkPreamble("PIDR4_RES0", PIDR4 & 0xFFFFFF00, 0);
    checkPreamble("PIDR5_RES0", PIDR5, 0);
    checkPreamble("PIDR6_RES0", PIDR6, 0);
    checkPreamble("PIDR7_RES0", PIDR7, 0);
    checkPreamble("PIDR0_RES0", PIDR0 & 0xFFFFFF00, 0);
    checkPreamble("PIDR1_RES0", PIDR1 & 0xFFFFFF00, 0);
    checkPreamble("PIDR2_RES0", PIDR2 & 0xFFFFFF00, 0);
    checkPreamble("PIDR3_RES0", PIDR3 & 0xFFFFFF00, 0);
    dumpRegField(indent, "REVISION", PIDR2, 4, 7, true);
    dumpRegField(indent, "REVAND", PIDR3, 4, 7, true);
    dumpRegField(indent, "CMOD", PIDR3, 0, 3, true);
    dumpRegField(indent, "JEDEC", PIDR2, 3, 3, true);
    PRINTF("    Designer = 0x%lX (%s)\n", designer, decodeDesigner(designer));
    PRINTF("    PartNo = 0x%lX\n", partno);
    uint32_t SIZE = dumpRegField(indent, "SIZE", PIDR4, 4, 7, false);
        printf(" (%u)\n", 4096 * (1 << SIZE));
}

static void dumpDEVARCH(int indent, ComponentType& componentType)
{
    uint32_t DEVARCH = 0;
    bool result = readRegister(DEVARCH_OFFSET, &DEVARCH);
    if (!result)
    {
        printf("Failed to read AP's DEVARCH register.\n");
    }

    PRINTF("  DEVARCH = 0x%08lX\n", DEVARCH);
    uint32_t architect = dumpRegField(indent, "ARCHITECT", DEVARCH, 21, 31, false);
        printf(" (%s)\n", decodeDesigner(architect));
    dumpRegField(indent, "PRESENT", DEVARCH, 20, 20, true);
    dumpRegField(indent, "REVISION", DEVARCH, 16, 19, true);
    uint32_t archid = dumpRegField(indent, "ARCHID", DEVARCH, 0, 15, false);
        printf(" (%s)\n", decodeArchitectureID(archid));

    componentType.isMEMAP |= (archid == 0x0A17);
    componentType.isROMTable |= (archid == 0x0AF7);
}

static const char* decodeArchitectureID(uint32_t archid)
{
    const struct ARCHID {
        const char* pName;
        uint32_t archid;
    } knownArchitectures[] = {
        { "RAS architecture",  0x0A00 },
        { "Instrumentation Trace Macrocell (ITM) architecture", 0x1A01 },
        { "DWT architecture", 0x1A02 },
        { "Flash Patch and Breakpoint unit (FPB) architecture", 0x1A03 },
        { "Processor debug architecture (ARMv8-M)", 0x2A04 },
        { "Processor debug architecture (ARMv8-R)", 0x6A05 },
        { "PC sample-based profiling", 0x0A10 },
        { "ETM architecture.", 0x4A13 },
        { "CTI architecture", 0x1A14 },
        { "Processor debug architecture (v8.0-A)", 0x6A15 },
        { "Processor debug architecture (v8.1-A)", 0x7A15 },
        { "Processor debug architecture (v8.2-A)", 0x8A15 },
        { "Processor Performance Monitor (PMU) architecture", 0x2A16 },
        { "Memory Access Port v2 architecture", 0x0A17},
        { "JTAG Access Port v2 architecture", 0x0A27 },
        { "Basic trace router", 0x0A31 },
        { "Power requester", 0x0A34 },
        { "Unknown Access Port v2 architecture", 0x0A47 },
        { "HSSTP architecture", 0x0A50 },
        { "STM architecture", 0x0A63 },
        { "CoreSight ELA architecture", 0x0A75 },
        { "CoreSight ROM architecture", 0x0AF7 },
    };

    for (size_t i = 0 ; i < count_of(knownArchitectures) ; i++) {
        const ARCHID* pID = &knownArchitectures[i];
        if (pID->archid == archid) {
            return pID->pName;
        }
    }

    return "Unknown";
}

static void dumpAUTHSTATUS(int indent)
{
    uint32_t AUTHSTATUS = 0;
    bool result = readRegister(AUTHSTATUS_OFFSET, &AUTHSTATUS);
    if (!result)
    {
        printf("Failed to read AP's AUTHSTATUS register.\n");
        return;
    }

    PRINTF("  AUTHSTATUS = 0x%08lX\n", AUTHSTATUS);
    dumpAuthStatusBits(indent, "RTNID (Root Non-Invasive Debug)", AUTHSTATUS, 26, 27);
    dumpAuthStatusBits(indent, "RTID (Root Invasive Debug)", AUTHSTATUS, 24, 25);
    dumpAuthStatusBits(indent, "SUNID (Secure Unprivileged Non-Invasive Debug)", AUTHSTATUS, 22, 23);
    dumpAuthStatusBits(indent, "SUID (Secure Unprivileged Invasive Debug)", AUTHSTATUS, 20, 21);
    dumpAuthStatusBits(indent, "NSUNID (Non-Secure Unprivileged Non-Invasive Debug)", AUTHSTATUS, 18, 19);
    dumpAuthStatusBits(indent, "NSUID (Non-Secure Unprivileged Invasive Debug)", AUTHSTATUS, 16, 17);
    dumpAuthStatusBits(indent, "RLNID (Realm Non-Invasive Debug)", AUTHSTATUS, 14, 15);
    dumpAuthStatusBits(indent, "RLID (Realm Invasive Debug)", AUTHSTATUS, 12, 13);
    dumpAuthStatusBits(indent, "HNID (Hypervisor Non-Invasive Debug)", AUTHSTATUS, 10, 11);
    dumpAuthStatusBits(indent, "HID (Hypervisor Invasive Debug)", AUTHSTATUS, 8, 9);
    dumpAuthStatusBits(indent, "SNID (Secure Non-Invasive Debug)", AUTHSTATUS, 6, 7);
    dumpAuthStatusBits(indent, "SID (Secure Invasive Debug)", AUTHSTATUS, 4, 5);
    dumpAuthStatusBits(indent, "NSNID (Non-Secure Non-Invasive Debug)", AUTHSTATUS, 2, 3);
    dumpAuthStatusBits(indent, "NSID (Non-Secure Invasive Debug)", AUTHSTATUS, 0, 1);
}

static void dumpAuthStatusBits(int indent, const char* pFieldName, uint32_t reg, uint32_t loBit, uint32_t hiBit)
{
    uint32_t value = dumpRegField(indent, pFieldName, reg, loBit, hiBit, false);
    switch (value)
    {
        case 0x0:
            printf(" (Not Implemented)\n");
            break;
        case 0x2:
            printf(" (Implemented but Disabled)\n");
            break;
        case 0x3:
            printf(" (Implemented and Enabled)\n");
            break;
        default:
            printf("\n");
            break;
    }
}

static void dumpDEVTYPE(int indent)
{
    uint32_t DEVTYPE = 0;
    bool result = readRegister(DEVTYPE_OFFSET, &DEVTYPE);
    if (!result)
    {
        printf("Failed to read AP's DEVTYPE register.\n");
        return;
    }
    PRINTF("  DEVTYPE = 0x%08lX\n", DEVTYPE);
    uint32_t MAJOR = dumpRegField(indent, "MAJOR", DEVTYPE, 0, 3, true);
    uint32_t SUB = dumpRegField(indent, "SUB", DEVTYPE, 4, 7, true);

    const struct TYPE {
        const char* pName;
        uint32_t major;
        uint32_t sub;
    } knownTypes[] = {
        { "Miscellaneous - Other, undefined", 0x0, 0x0 },
        { "Miscellaneous - Validation component", 0x0, 0x4 },
        { "Trace Sink - Trace port, for example TPIU", 0x1, 0x1 },
        { "Trace Sink - Buffer, for example ETB", 0x1, 0x2 },
        { "Trace Sink - Basic trace router", 0x1, 0x3 },
        { "Trace Link - Other", 0x2, 0x0 },
        { "Trace Link - Trace funnel, Router", 0x2, 0x1 },
        { "Trace Link - Filter", 0x2, 0x2 },
        { "Trace Link - FIFO, Large Buffer", 0x2, 0x3 },
        { "Trace Source - Other", 0x3, 0x0 },
        { "Trace Source - Associated with a processor core", 0x3, 0x1 },
        { "Trace Source - Associated with a DSP", 0x3, 0x2 },
        { "Trace Source - Associated with a Data Engine or coprocessor", 0x3, 0x3 },
        { "Trace Source - Associated with a Bus, stimulus-derived from bus activity", 0x3, 0x4 },
        { "Trace Source - Associated with software, stimulus-derived from software activity", 0x3, 0x6 },
        { "Debug Control - Other", 0x4, 0x0 },
        { "Debug Control - Trigger Matrix, for example ECT", 0x4, 0x1 },
        { "Debug Control - Debug Authentication Module", 0x4, 0x2 },
        { "Debug Control - Power requestor", 0x4, 0x3 },
        { "Debug Logic - Other", 0x5, 0x0 },
        { "Debug Logic - Processor core", 0x5, 0x1 },
        { "Debug Logic - DSP", 0x5, 0x2 },
        { "Debug Logic - Data Engine or coprocessor", 0x5, 0x3 },
        { "Debug Logic - Bus, stimulus-derived from bus activity", 0x5, 0x4 },
        { "Debug Logic - Memory, tightly coupled device such as Built In Self-Test (BIST)", 0x5, 0x5 },
        { "Performance Monitor - Other", 0x6, 0x0 },
        { "Performance Monitor - Associated with a processor", 0x6, 0x1 },
        { "Performance Monitor - Associated with a DSP", 0x6, 0x2 },
        { "Performance Monitor - Associated with a Data Engine or coprocessor", 0x6, 0x3 },
        { "Performance Monitor - Associated with a bus, stimulus-derived from bus activity", 0x6, 0x4 },
        { "Performance Monitor - Associated with a Memory Management Unit that conforms to the Arm System MMU Architecture", 0x6, 0x5 },
    };

    for (size_t i = 0 ; i < count_of(knownTypes) ; i++) {
        const TYPE* pType = &knownTypes[i];
        if (pType->major == MAJOR && pType->sub == SUB) {
            PRINTF("      %s\n", pType->pName);
            return;
        }
    }
    PRINTF("    Reserved\n");
}

static uint32_t dumpRegister(int indent, const char* pRegName, uint32_t regIndex)
{
    uint32_t regValue = 0;
    bool result = readRegister(regIndex, &regValue);
    if (!result)
    {
        printf("Failed to read AP's %s register.\n", pRegName);
        return 0;
    }
    PRINTF("  %s = 0x%08lX\n", pRegName, regValue);

    return regValue;
}

static uint32_t dumpApRegister(int indent, const char* pRegName, uint32_t regIndex)
{
    uint32_t regValue = 0;
    bool result = readAP(regIndex, &regValue);
    if (!result)
    {
        printf("Failed to read AP's %s register.\n", pRegName);
        return 0;
    }
    PRINTF("  %s = 0x%08lX\n", pRegName, regValue);

    return regValue;
}

static uint32_t dumpCFG(int indent, uint32_t dpVersion)
{
    uint32_t value = 0;
    uint32_t CFG = SWD::APv1_CFG;
    if (dpVersion >= 3)
    {
        CFG = SWD::APv2_CFG;
    }

    bool result = readAP(CFG, &value);
    if (!result)
    {
        printf("Failed to read MEM-AP's CFG register.\n");
        return 0x0;
    }
    PRINTF("  CFG = 0x%08lX\n", value);

    if (dpVersion >= 3)
    {
        uint32_t meciWidth = dumpRegField(indent, "MECIDWIDTH", value, 20, 25, false);
            printf(" (%s)\n", decodeMeciWidth(meciWidth));
        uint32_t tarInc = dumpRegField(indent, "TARINC", value, 16, 19, false);
            printf(" (%s)\n", decodeTarInc(tarInc));
        uint32_t err = dumpRegField(indent, "ERR", value, 8, 11, false);
            printf(" (%s)\n", decodeErr(err));
        uint32_t darSize = dumpRegField(indent, "DARSIZE", value, 4, 7, false);
            printf(" (%s)\n", decodeDarSize(darSize));
        dumpRegField(indent, "RME", value, 3, 3, true);
    }
    dumpRegField(indent, "LargeData", value, 2, 2, true);
    dumpRegField(indent, "LongAddress", value, 1, 1, true);
    dumpRegField(indent, "BigEndian", value, 0, 0, true);

    return value;
}

static const char* decodeMeciWidth(uint32_t meciWidth)
{
    static char buffer[64];
    if (meciWidth == 0x0)
    {
        return "MEC extension is not implemented";
    }
    else
    {
        snprintf(buffer, sizeof(buffer), "MEC extension is implemented with width=%lu\n", meciWidth);
        return buffer;
    }
}

static const char* decodeTarInc(uint32_t tarInc)
{
    static char buffer[32];
    if (tarInc == 0)
    {
        return ">= 10 bits";
    }
    else
    {
        snprintf(buffer, sizeof(buffer), "%lu bits", 9 + tarInc);
        return buffer;
    }
}

static const char* decodeErr(uint32_t err)
{
    switch (err)
    {
        case 0x0:
            return "CSW.ERRNPASS and CSW.ERRSTOP are not implemented";
        case 0x1:
            return "CSW.ERRNPASS and CSW.ERRSTOP are implemented";
        default:
            return "Reserved";
    }
}

static const char* decodeDarSize(uint32_t darsize)
{
    switch (darsize)
    {
        case 0x0:
            return "DAR0-DAR255 are not implemented";
        case 0xA:
            return "DAR0-DAR255 are implemented";
        default:
            return "Reserved";
    }
}

static bool dumpCFG1(int indent, uint32_t dpVersion)
{
    uint32_t value = 0;
    uint32_t CFG1 = SWD::APv1_CFG1;
    if (dpVersion >= 3)
    {
        CFG1 = SWD::APv2_CFG1;
    }

    bool result = readAP(CFG1, &value);
    if (!result)
    {
        printf("Failed to read MEM-AP's CFG1 register.\n");
        return 0x0;
    }
    PRINTF("  CFG1 = 0x%08lX\n", value);

    uint32_t tag0Gran = dumpRegField(indent, "TAG0GRAN", value, 4, 8, false);
        printf(" (%s)\n", decodeTag0Gran(tag0Gran));
    uint32_t tag0Size = dumpRegField(indent, "TAG0SIZE", value, 0, 3, false);
        printf(" (%s)\n", decodeTag0Size(tag0Size));

    return tag0Size == 0x4;
}

static const char* decodeTag0Gran(uint32_t tag0Gran)
{
    switch (tag0Gran)
    {
        case 0x4:
            return "Memory tagging granule is 16 bytes";
        default:
            return "Reserved";
    }
}

static const char* decodeTag0Size(uint32_t tag0Size)
{
    switch (tag0Size)
    {
        case 0x0:
            return "Memory Tagging Extension not implemented";
        case 0x4:
            return "Memory Tagging Extension implemented. Tag size is 4-bits";
        default:
            return "Reserved";
    }
}

static void dumpCSW(int indent, uint32_t dpVersion, uint32_t cfg, bool isTaggingImplemented)
{
    uint32_t value = 0;
    uint32_t CSW = SWD::APv1_CSW;
    if (dpVersion >= 3)
    {
        CSW = SWD::APv2_CSW;
    }

    bool result = readAP(CSW, &value);
    if (!result)
    {
        printf("Failed to read MEM-AP's CSW register.\n");
        return;
    }
    PRINTF("  CSW = 0x%08lX\n", value);

    uint32_t cfgRME = (cfg >> 3) & 0x1;
    uint32_t cfgERR = (cfg >> 8) & 0xF;
    dumpRegField(indent, "DbgSwEnable", value, 31, 31, true);
    dumpRegField(indent, "Prot", value, 24, 30, true);
    dumpRegField(indent, "SDeviceEn", value, 23, 23, true);
    if (cfgRME == 0x1)
    {
        uint32_t rmeen = dumpRegField(indent, "RMEEN", value, 21, 22, false);
            printf(" (%s)\n", decodeRmeEn(rmeen));
    }
    if (cfgERR == 0x1)
    {
        dumpRegField(indent, "ERRSTOP", value, 17, 17, true);
        dumpRegField(indent, "ERRNPASS", value, 16, 16, true);
    }
    if (isTaggingImplemented)
    {
        dumpRegField(indent, "MTE", value, 15, 15, true);
        dumpRegField(indent, "Type", value, 12, 14, true);
    }
    else
    {
        dumpRegField(indent, "Type", value, 12, 15, true);
    }
    uint32_t mode = dumpRegField(indent, "Mode", value, 8, 11, false);
        printf(" (%s)\n", decodeMode(mode));
    dumpRegField(indent, "TrInProg", value, 7, 7, true);
    dumpRegField(indent, "DeviceEn", value, 6, 6, true);
    uint32_t addrInc = dumpRegField(indent, "AddrInc", value, 4, 5, false);
        printf(" (%s)\n", decodeAddrInc(addrInc));
    uint32_t size = dumpRegField(indent, "Size", value, 0, 2, false);
        printf(" (%s)\n", decodeSize(size));
}

static const char* decodeRmeEn(uint32_t rmeen)
{
    switch (rmeen)
    {
        case 0x0:
            return "Realm and Root accesses are disabled";
        case 0x1:
            return "Realm access is enabled anbd Root access is disabled";
        case 0x3:
            return "Realm and Root access is enabled";
        default:
            return "Reserved";
    }
}

static const char* decodeMode(uint32_t mode)
{
    switch (mode)
    {
        case 0x0:
            return "Basic mode";
        case 0x1:
            return "Barrier support enabled";
        default:
            return "Reserved";
    }
}

static const char* decodeAddrInc(uint32_t addrInc)
{
    switch (addrInc)
    {
        case 0x0:
            return "Address auto-increment disabled";
        case 0x1:
            return "Address increment-single enabled";
        case 0x2:
            return "Address increment-packed enabled";
        default:
            return "Reserved";
    }
}

static const char* decodeSize(uint32_t size)
{
    switch (size)
    {
        case 0x0:
            return "8-bits - Byte";
        case 0x1:
            return "16-bits - Halfword";
        case 0x2:
            return "32-bits - Word";
        case 0x3:
            return "64-bits - Doubleword";
        case 0x4:
            return "128-bits";
        case 0x5:
            return "256-bits";
        default:
            return "Reserved";
    }
}

static void dumpApRomTable(int indent, uint32_t dpVersion, uint32_t apAddress)
{
    printf("Dump AP CoreSight ROM Table at address 0x%lX.\n", apAddress);

    // UNDONE: Could dump some of the ROM Table specific registers in the future.

    uint32_t DEVID = 0;
    bool result = readAP(SWD::APv2_DEVID, &DEVID);
    if (!result)
    {
        printf("Failed to read AP ROM Table's DEVID register.\n");
    }

    // UNDONE: uint32_t PRR = (DEVID >> 5) & 0x1;
    uint32_t FORMAT = DEVID & 0xF;
    if (FORMAT != 0x0)
    {
        printf("Don't support Class 9 ROM Table DEVID.FORMAT other than 32-bit.\n");
        return;
    }

    // Walk the table entries.
    uint32_t offset;
    for (offset = 0 ; offset < 0x800 ; offset += sizeof(uint32_t))
    {
        uint32_t entry;
        result = readAP(SWD::AP_DP_RO | offset, &entry);
        if (!result)
        {
            printf("Failed to read AP Class 9 ROM Table Entry at offset 0x%lX\n", offset);
            return;
        }

        RomEntry romEntry = parseClass9Entry(entry);
        if (romEntry.isLastEntry)
        {
            break;
        }
        if (!romEntry.isPresent)
        {
            continue;
        }

        printf("  Offset: %ld", romEntry.offset);
        if (romEntry.isPowerDomainValid)
        {
            printf("  PowerDomain: %lu", romEntry.powerDomainId);
        }
        printf("\n");

        dumpAP(indent+1, dpVersion, apAddress + romEntry.offset);
    }
}

static RomEntry parseClass9Entry(uint32_t entry)
{
    RomEntry romEntry = {
        .offset = 0,
        .powerDomainId = 0,
        .isPresent = false,
        .isLastEntry = false,
        .isPowerDomainValid = false };

    romEntry.offset = ((int32_t)entry >> 12);
    romEntry.powerDomainId = (entry >> 4) & 0x1F;
    romEntry.isPowerDomainValid = !!((entry >> 2) & 0x1);
    uint32_t PRESENT = entry & 0x3;
    switch (PRESENT)
    {
        case 0x0:
            romEntry.isLastEntry = true;
            romEntry.isPresent = false;
            break;
        default:
        case 0x2:
            romEntry.isPresent = false;
            romEntry.isLastEntry = false;
            break;
        case 0x3:
            romEntry.isPresent = true;
            romEntry.isLastEntry = false;
            break;
    }

    return romEntry;
}

static void dumpMemRomTable(int indent, uint32_t baseAddress, uint32_t _class)
{
    PRINTF("Dumping ROM Table address 0x%08lX\n", baseAddress);
    switch (_class)
    {
        case 1:
            dumpClass1MemRomTable(indent, baseAddress);
            break;
        case 9:
            dumpClass9MemRomTable(indent, baseAddress);
            break;
        default:
            assert ( _class == 1 || _class == 9 );
    }
}

static void dumpClass1MemRomTable(int indent, uint32_t baseAddress)
{
    dumpMEMTYPE(indent);

    // Walk the table entries.
    uint32_t offset;
    for (offset = 0 ; offset < 0xF00 ; offset += sizeof(uint32_t))
    {
        uint32_t entry;
        bool result = readRegister(offset, &entry);
        if (!result)
        {
            printf("Failed to read AP Class 1 ROM Table Entry at offset 0x%lX\n", offset);
            return;
        }

        RomEntry romEntry = parseClass1Entry(entry);
        if (romEntry.isLastEntry)
        {
            break;
        }
        if (!romEntry.isPresent)
        {
            continue;
        }

        PRINTF("  Offset: %ld", romEntry.offset);
        if (romEntry.isPowerDomainValid)
        {
            printf("  PowerDomain: %lu", romEntry.powerDomainId);
        }
        printf("\n");

        {
            ComponentType componentType;
            uint32_t componentAddress = baseAddress + (romEntry.offset << 12);
            MemRegisterReader reader(componentAddress);
            dumpCoreSightComponent(indent+1, componentAddress, componentType, true);
        }
    }
}

static void dumpMEMTYPE(int indent)
{
    uint32_t value;
    bool result = readRegister(MEMTYPE_OFFSET, &value);
    if (!result)
    {
        printf("Failed to read MEMTYPE register.\n");
        return;
    }
    PRINTF("  MEMTYPE = 0x%08lX\n", value);

    dumpRegField(indent, "SYSMEM", value, 0, 0, true);
}

static RomEntry parseClass1Entry(uint32_t entry)
{
    RomEntry romEntry;

    romEntry.offset = ((int32_t)entry >> 12);
    romEntry.powerDomainId = (entry >> 4) & 0x1F;
    romEntry.isPowerDomainValid = !!((entry >> 2) & 0x1);
    romEntry.isPresent = !!(entry & 0x1);
    romEntry.isLastEntry = (entry == 0x00000000);

    return romEntry;
}

static void dumpClass9MemRomTable(int indent, uint32_t baseAddress)
{
    uint32_t DEVID = 0;
    bool result = readRegister(DEVID_OFFSET, &DEVID);
    if (!result)
    {
        printf("Failed to read AP ROM Table's DEVID register.\n");
    }

    // UNDONE: uint32_t PRR = (DEVID >> 5) & 0x1;
    uint32_t FORMAT = DEVID & 0xF;
    if (FORMAT != 0x0)
    {
        printf("Don't support Class 9 ROM Table DEVID.FORMAT other than 32-bit.\n");
        return;
    }

    // Walk the table entries.
    uint32_t offset;
    for (offset = 0 ; offset < 0x800 ; offset += sizeof(uint32_t))
    {
        uint32_t entry;
        result = readRegister(offset, &entry);
        if (!result)
        {
            printf("Failed to read AP Class 9 ROM Table Entry at offset 0x%lX\n", offset);
            return;
        }

        RomEntry romEntry = parseClass9Entry(entry);
        if (romEntry.isLastEntry)
        {
            break;
        }
        if (!romEntry.isPresent)
        {
            continue;
        }

        PRINTF("  Offset: %ld", romEntry.offset);
        if (romEntry.isPowerDomainValid)
        {
            printf("  PowerDomain: %lu", romEntry.powerDomainId);
        }
        printf("\n");

        {
            ComponentType componentType;
            uint32_t componentAddress = baseAddress + (romEntry.offset << 12);
            MemRegisterReader reader(componentAddress);
            dumpCoreSightComponent(indent+1, componentAddress, componentType, true);
        }
    }
}

static void handleHelpCommand(const CommandTokens* pCmdTokens)
{
    printf("Supported commands:\n");
    for (size_t i = 0 ; i < count_of(g_commandHandlers) ; i++)
    {
        if (g_commandHandlers[i].pParameterDescription != NULL)
        {
            printf("  %s %s - %s\n",
                g_commandHandlers[i].pCommand,
                g_commandHandlers[i].pParameterDescription,
                g_commandHandlers[i].pDescription);
        }
        else
        {
            printf("  %s - %s\n",
                g_commandHandlers[i].pCommand,
                g_commandHandlers[i].pDescription);
        }
    }
}

static void handleVerboseCommand(const CommandTokens* pCmdTokens)
{
    if (pCmdTokens->tokenCount < 2)
    {
        printf("Must specify 'on' or 'off' as arguments to verbose command.\n");
        return;
    }
    checkParameterCount(pCmdTokens, 1);

    if (0 == strcasecmp(pCmdTokens->tokens[1], "on"))
    {
        printf("Enabling VERBOSE output.\n");
        g_verbose = true;
    }
    else if (0 == strcasecmp(pCmdTokens->tokens[1], "off"))
    {
        printf("Disabling VERBOSE output.\n");
        g_verbose = false;
    }
    else
    {
        printf("Must specify 'on' or 'off' as arguments to verbose command.\n");
    }
}
