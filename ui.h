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
#ifndef UI_H_
#define UI_H_
#include <Adafruit_SSD1306/Adafruit_SSD1306.h>
#include <pico/async_context_threadsafe_background.h>


class UI
{
public:
    UI(uint8_t width, uint8_t height,
       int8_t mosiPin, int8_t sclkPin, int8_t dcPin, int8_t rstPin, int8_t csPin);

    bool init();

    enum WiFiState
    {
        WIFI_CONNECTING,
        WIFI_CONNECTED
    };

    // Methods used by mri-swd code to reports its state to this UI class so that it can be reflected on the OLED.
    void updateWiFiState(WiFiState wifiState, const char* pConnectedIpAddress);
    void setRunState(const char* pRunState);
    static void setGdbConnectedState(bool isConnected);
    static void setUartConnectedState(bool isConnected);
    static void transmittingToGdb();
    static void receivingFromGdb();
    static void transmittingToUart();
    static void receivingFromUart();
    static void setTargetName(const char* pTargetName);

protected:
    // The GDB and UART transmit and receive activity icons are lit for a minimum of this timeout amount.
    static const uint32_t ACTIVITY_TIMEOUT_MS = 250;
    // The WiFi log animation frames are each displayed for this amount of time.
    static const uint32_t ANIMATION_INTERVAL_MS = 250;
    // Dimensions of the WiFi Logo.
    static const uint16_t LOGO_WIDTH = 16;
    static const uint16_t LOGO_HEIGHT = 12;
    // The number of frames in the WiFi Logo animation.
    static const size_t   LOGO_ANIMATION_FRAMES = 4;

    void updateIpAddress(const char* pIpAddress);
    void scheduleDisplayUpdate();

    static void handleDisplayUpdate(async_context_t *async_context, async_when_pending_worker_t *worker);
    static void handleActivityTimeout(async_context_t *async_context, async_work_on_timeout *activityTimeout);
    static void handleAnimationTimeout(async_context_t *async_context, async_work_on_timeout *activityTimeout);

    void update();
    void drawWiFiLogo(uint16_t x, uint16_t y);

    void activityTimeout();
    void animationTimeout();


    // UI Singleton.
    static UI*           s_pThis;

    // WiFi Logo Animation Frames.
    static const uint8_t s_wifiLogo[4][24];

    // Number of packets that have been received and transmitted from/to GDB and UART.
    uint32_t         m_packetsToGdb = 0;
    uint32_t         m_packetsFromGdb = 0;
    uint32_t         m_packetsToUart = 0;
    uint32_t         m_packetsFromUart = 0;

    // Packet counters from last execution of activityTimeout() method.
    uint32_t         m_prevPacketsToGdb = 0;
    uint32_t         m_prevPacketsFromGdb = 0;
    uint32_t         m_prevPacketsToUart = 0;
    uint32_t         m_prevPacketsFromUart = 0;

    // Counter used for running the WiFi logo animation during connection attempts.
    uint32_t         m_animationCount = 0;

    // Object which knows how to draw on the attached OLED.
    Adafruit_SSD1306 m_oled;

    // Is the device in the process of connecting to WiFi or is it already connected?
    WiFiState        m_wifiState = WIFI_CONNECTING;

    // State of the GDB and UART TCP/IP port connections.
    bool m_isGdbConnected = false;
    bool m_isUartConnected = false;

    // Current transmit/receive state of the GDB and UART ports.
    bool m_isTransmittingToGdb = false;
    bool m_isReceivingFromGdb = false;
    bool m_isTransmittingToUart = false;
    bool m_isReceivingFromUart = false;

    // An async context is notified when there is new state to be displayed on the OLED.
    // All SPI operations to the OLED occur from one of these async workers.
    async_context_threadsafe_background_t m_asyncContext;
    async_when_pending_worker_t           m_asyncWorker;
    async_at_time_worker_t                m_asyncActivityTimer;
    async_at_time_worker_t                m_asyncAnimationTimer;

    // IP address of format: ddd.ddd.ddd.ddd\0
    char m_ipAddress[3+1+3+1+3+1+3+1] = { '\0' };

    // Name of target device type.
    char m_targetName[13+1] = { '\0' };

    // Run state ("Running" or "Halted") as text.
    char m_runState[7+1] = { '\0' };
};

#endif // UI_H_
