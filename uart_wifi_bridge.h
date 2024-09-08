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
// Module which uses LWIP for implementing the UART WiFi Bridge.
#ifndef UART_WIFI_BRIDGE_H_
#define UART_WIFI_BRIDGE_H_

#include <hardware/dma.h>
#include <hardware/uart.h>
#include <pico/async_context_threadsafe_background.h>
#include "socket_server.h"
#include "circular_queue.h"


class UartWiFiBridge : public SocketServer
{
    public:
        UartWiFiBridge();

        bool init(uint16_t port, uint32_t uartTxPin, uint32_t uartRxPin, uint32_t uartBaudRate);
        void uninit();

    protected:
        virtual uint32_t writeReceivedData(const struct pbuf *pBuf);
        virtual bool shouldSendNow(const void* pBuffer, uint16_t bufferLength);
        virtual void updateConnectionState(bool isConnected);
        virtual void discardingBytesDueToUnconnectedSocket(uint16_t bytesDiscarded);

        uart_inst_t* txPinToUartInstance(uint32_t txPin);
        uart_inst_t* rxPinToUartInstance(uint32_t rxPin);

        static void staticUartISR()
        {
            assert ( s_pThis != NULL );
            s_pThis->uartISR();
        }
        void uartISR();

        static void staticSendQueuedDataToSocket(async_context_t *async_context, async_when_pending_worker_t *worker)
        {
            assert ( s_pThis != NULL );
            s_pThis->sendQueuedDataToSocket();
        }
        void sendQueuedDataToSocket();

        static void staticDmaISR()
        {
            assert ( s_pThis != NULL );
            s_pThis->dmaISR();
        }
        void dmaISR();
        void sendQueuedDataToUart();


        static UartWiFiBridge* s_pThis;

        uart_inst_t*       m_pUartInstance = NULL;
        int                m_uartIRQ = 0;
        int                m_dmaIRQ = 0;
        int                m_dmaChannel = 0;
        dma_channel_config m_dmaConfig;
        CircularQueue<256> m_tcpToUartQueue;
        CircularQueue<256> m_uartToTcpQueue;

        // An async context is notified when there is new data to be sent out over lwIP at a lower priority.
        async_context_threadsafe_background_t m_asyncContext;
        async_when_pending_worker_t           m_asyncWorker;

        volatile bool      m_isDmaTransmitting = false;
};

#endif // UART_WIFI_BRIDGE_H_
