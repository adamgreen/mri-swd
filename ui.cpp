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
// UI class used to present mri-swd state to the user via a 128x32 OLED screen (SS1306).
// Currently supports Adafruit's Monochrome 0.91" 128x32 I2C OLED Display (https://www.adafruit.com/product/661)
#define UI_MODULE "ui.cpp"
#include <string.h>
#include "logging.h"
#include "ui.h"

// Initialize the singleton.
UI* UI::s_pThis = NULL;

// WiFi Logo with multiple frames for animating when attempting to connect to a WiFi router.
const uint8_t UI::s_wifiLogo[4][24] = {
    {
        // Full logo in 0th element.
        0b00001111, 0b11111000,
        0b00011000, 0b00001100,
        0b01100000, 0b00000011,
        0b01000111, 0b11110001,
        0b00001100, 0b00011000,
        0b00010000, 0b00000100,
        0b00000011, 0b11100000,
        0b00000110, 0b00110000,
        0b00000100, 0b00010000,
        0b00000001, 0b11000000,
        0b00000001, 0b11000000,
        0b00000001, 0b11000000
    },
    {
        // Just the dot and the following elements will animate the bands outwards from this dot.
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000001, 0b11000000,
        0b00000001, 0b11000000,
        0b00000001, 0b11000000
    },
    {
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000011, 0b11100000,
        0b00000110, 0b00110000,
        0b00000100, 0b00010000,
        0b00000001, 0b11000000,
        0b00000001, 0b11000000,
        0b00000001, 0b11000000
    },
    {
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000000, 0b00000000,
        0b00000111, 0b11110000,
        0b00001100, 0b00011000,
        0b00010000, 0b00000100,
        0b00000011, 0b11100000,
        0b00000110, 0b00110000,
        0b00000100, 0b00010000,
        0b00000001, 0b11000000,
        0b00000001, 0b11000000,
        0b00000001, 0b11000000
    },
};


UI::UI(uint8_t width, uint8_t height,
       int8_t mosiPin, int8_t sclkPin, int8_t dcPin, int8_t rstPin, int8_t csPin)
: m_oled(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, mosiPin, sclkPin, dcPin, rstPin, csPin)
{
    // This should be the only singleton object.
    assert ( s_pThis == NULL );
    s_pThis = this;
}


bool UI::init()
{
    // Setup an async context and workers to perform display updates when needed.
    // All SPI based OLED updates occur from these async contexts.
    if (!async_context_threadsafe_background_init_with_defaults(&m_asyncContext))
    {
        logError("Failed to initialize async context.");
        return false;
    }
    memset(&m_asyncWorker, 0, sizeof(m_asyncWorker));
    memset(&m_asyncActivityTimer, 0, sizeof(m_asyncActivityTimer));
    memset(&m_asyncAnimationTimer, 0, sizeof(m_asyncAnimationTimer));
    m_asyncWorker.do_work = handleDisplayUpdate;
    m_asyncActivityTimer.do_work = handleActivityTimeout;
    m_asyncAnimationTimer.do_work = handleAnimationTimeout;
    async_context_add_when_pending_worker(&m_asyncContext.core, &m_asyncWorker);

    m_lastHeartBeatTime = get_absolute_time();

    bool result = m_oled.begin();
    if (!result)
    {
        logError("Failed to initialize OLED.");
        return false;
    }
    scheduleDisplayUpdate();

    return result;
}


void UI::updateWiFiState(WiFiState wifiState, const char* pConnectedIpAddress)
{
    m_wifiState = wifiState;
    switch (wifiState)
    {
        case WIFI_CONNECTING:
            m_animationCount = 1;
            async_context_add_at_time_worker_in_ms(&m_asyncContext.core, &m_asyncAnimationTimer, ANIMATION_INTERVAL_MS);
            updateIpAddress("");
            break;
        case WIFI_CONNECTED:
            m_animationCount = 0;
            updateIpAddress(pConnectedIpAddress);
            break;
        default:
            assert ( false );
            break;
    }
}

void UI::updateIpAddress(const char* pIpAddress)
{
    strlcpy(m_ipAddress, pIpAddress, sizeof(m_ipAddress));
    scheduleDisplayUpdate();
}

void UI::scheduleDisplayUpdate()
{
    // Schedule async display update.
    async_context_set_work_pending(&m_asyncContext.core, &m_asyncWorker);
}

void UI::setGdbConnectedState(bool isConnected)
{
    assert ( s_pThis != NULL );
    if (isConnected == s_pThis->m_isGdbConnected)
    {
        return;
    }
    s_pThis->m_isGdbConnected = isConnected;
    s_pThis->scheduleDisplayUpdate();
}

void UI::setUartConnectedState(bool isConnected)
{
    assert ( s_pThis != NULL );
    if (isConnected == s_pThis->m_isUartConnected)
    {
        return;
    }
    s_pThis->m_isUartConnected = isConnected;
    s_pThis->scheduleDisplayUpdate();
}

void UI::transmittingToGdb()
{
    s_pThis->m_packetsToGdb++;
    s_pThis->m_isTransmittingToGdb = true;
    s_pThis->scheduleDisplayUpdate();
    async_context_add_at_time_worker_in_ms(&s_pThis->m_asyncContext.core, &s_pThis->m_asyncActivityTimer, ACTIVITY_TIMEOUT_MS);
}

void UI::receivingFromGdb()
{
    s_pThis->m_packetsFromGdb++;
    s_pThis->m_isReceivingFromGdb = true;
    s_pThis->scheduleDisplayUpdate();
    async_context_add_at_time_worker_in_ms(&s_pThis->m_asyncContext.core, &s_pThis->m_asyncActivityTimer, ACTIVITY_TIMEOUT_MS);
}

void UI::transmittingToUart()
{
    s_pThis->m_packetsToUart++;
    s_pThis->m_isTransmittingToUart = true;
    s_pThis->scheduleDisplayUpdate();
    async_context_add_at_time_worker_in_ms(&s_pThis->m_asyncContext.core, &s_pThis->m_asyncActivityTimer, ACTIVITY_TIMEOUT_MS);
}

void UI::receivingFromUart()
{
    s_pThis->m_packetsFromUart++;
    s_pThis->m_isReceivingFromUart = true;
    s_pThis->scheduleDisplayUpdate();
    async_context_add_at_time_worker_in_ms(&s_pThis->m_asyncContext.core, &s_pThis->m_asyncActivityTimer, ACTIVITY_TIMEOUT_MS);
}

void UI::setTargetName(const char* pTargetName)
{
    strlcpy(s_pThis->m_targetName, pTargetName, sizeof(m_targetName));
    s_pThis->scheduleDisplayUpdate();

}

void UI::setRunState(const char* pRunState)
{
    // Do nothing if the UI is already displaying this state.
    if (strncmp(m_runState, pRunState, sizeof(m_runState)-1) == 0)
    {
        return;
    }

    strlcpy(m_runState, pRunState, sizeof(m_runState));
    scheduleDisplayUpdate();
}

void UI::beatHeart()
{
    absolute_time_t currentTime = get_absolute_time();
    if (absolute_time_diff_us(m_lastHeartBeatTime, currentTime) > UI::HEART_BEAT_INTERVAL_US)
    {
        m_heartBeat = !m_heartBeat;
        m_lastHeartBeatTime = currentTime;
        scheduleDisplayUpdate();
    }
}


void UI::handleDisplayUpdate(async_context_t *async_context, async_when_pending_worker_t *worker)
{
    assert ( s_pThis != NULL );
    s_pThis->update();
}

void UI::update()
{
    const int16_t charWidth = 6;
    const int16_t charHeight = 8;
    const int16_t charsPerRow = OLED_SCREEN_WIDTH / charWidth;

    // Clear the display buffer since we are going to redraw everything in it next.
    m_oled.clearDisplay();

    // All text will be drawn at a scale of 1 in white.
    m_oled.setTextSize(1);
    m_oled.setTextColor(SSD1306_WHITE);

    // Display GDB port connection status in upper left hand corner.
    if (m_isGdbConnected)
    {
        m_oled.setCursor(0, charHeight/2);
        m_oled.printf("GDB");
    }
    // Display UART port connection status in upper right hand corner.
    if (m_isUartConnected)
    {
        m_oled.setCursor(charWidth*(charsPerRow-4), charHeight/2);
        m_oled.printf("UART");
    }

    // Display GDB port activity icons to the right of GDB designator.
    if (m_isTransmittingToGdb)
    {
        m_oled.setCursor(charWidth*3, 0);
        m_oled.printf("\x11");
    }
    if (m_isReceivingFromGdb)
    {
        m_oled.setCursor(charWidth*3, charHeight*1);
        m_oled.printf("\x10");
    }
    // Display UART port activity icons to the left of UART designator.
    if (m_isTransmittingToUart)
    {
        m_oled.setCursor(charWidth*(charsPerRow-5), 0);
        m_oled.printf("\x10");
    }
    if (m_isReceivingFromUart)
    {
        m_oled.setCursor(charWidth*(charsPerRow-5), charHeight*1);
        m_oled.printf("\x11");
    }

    // Display the IP address towards the lower center of the screen.
    int16_t centeredStart = OLED_SCREEN_WIDTH / 2 - (strlen(m_ipAddress) * charWidth) / 2;
    m_oled.setCursor(centeredStart, 2*charHeight-2);
    m_oled.printf("%s", m_ipAddress);

    // Display target name in lower left hand corner.
    m_oled.setCursor(0, charHeight*3);
    m_oled.printf("%s", m_targetName);

    // Display target run state in lower right hand corner.
    m_oled.setCursor(charWidth*(charsPerRow-7), charHeight*3);
    m_oled.printf("%s", m_runState);

    // Draw the WiFi Logo.
    drawWiFiLogo(OLED_SCREEN_WIDTH/2 - LOGO_WIDTH/2, 0);

    // Draw the heart beat icon if needed.
    if (m_heartBeat)
    {
        m_oled.setCursor(OLED_SCREEN_WIDTH - charWidth, 2*charHeight);
        m_oled.printf("\x03");
    }

    // Push display buffer out to OLED over SPI.
    m_oled.display();
}

void UI::drawWiFiLogo(uint16_t x, uint16_t y)
{
    m_oled.drawBitmap(x, y, s_wifiLogo[m_animationCount], LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
}


void UI::handleActivityTimeout(async_context_t *async_context, async_work_on_timeout *activityTimeout)
{
    assert ( s_pThis != NULL );
    s_pThis->activityTimeout();
}

void UI::activityTimeout()
{
    // Check to see if the UART or GDB has transmitted or received data since last async timer fired.
    m_isTransmittingToGdb = m_prevPacketsToGdb != m_packetsToGdb;
    m_isReceivingFromGdb = m_prevPacketsFromGdb != m_packetsFromGdb;
    m_isTransmittingToUart = m_prevPacketsToUart != m_packetsToUart;
    m_isReceivingFromUart = m_prevPacketsFromUart != m_packetsFromUart;
    m_prevPacketsToGdb = m_packetsToGdb;
    m_prevPacketsFromGdb = m_packetsFromGdb;
    m_prevPacketsToUart = m_packetsToUart;
    m_prevPacketsFromUart = m_packetsFromUart;
    if (m_isTransmittingToGdb || m_isReceivingFromGdb || m_isTransmittingToUart || m_isReceivingFromUart)
    {
        // Check again later asynchronously if there is still activity on GDB or UART.
        async_context_add_at_time_worker_in_ms(&m_asyncContext.core, &m_asyncActivityTimer, ACTIVITY_TIMEOUT_MS);
    }
    update();
}

void UI::handleAnimationTimeout(async_context_t *async_context, async_work_on_timeout *activityTimeout)
{
    assert ( s_pThis != NULL );
    s_pThis->animationTimeout();
}

void UI::animationTimeout()
{
    if (m_wifiState == UI::WIFI_CONNECTING)
    {
        // Animate to the next frame of the WiFi logo animation if mri-swd is currently trying to connect to a router.
        m_animationCount = (m_animationCount + 1) % LOGO_ANIMATION_FRAMES;
    }

    update();

    if (m_wifiState == UI::WIFI_CONNECTING)
    {
        // When connecting, schedule to play next frame in the future.
        async_context_add_at_time_worker_in_ms(&m_asyncContext.core, &m_asyncAnimationTimer, ANIMATION_INTERVAL_MS);
    }
}
