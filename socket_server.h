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
// Base class for implementing TCP/IP Servers.
#ifndef SOCKET_SERVER_H_
#define SOCKET_SERVER_H_

#include <lwip/tcp.h>
#include "config.h"


class SocketServer
{
    public:
        SocketServer();

        bool init(const char* pServerName, uint16_t port);
        void uninit();

        err_t send(const void* pBuffer, uint16_t bufferLength);
        uint32_t bytesInFlight() { return m_bytesInFlight; }
        err_t closeClient();
        bool isConnected()
        {
            return m_pClientPCB != NULL;
        }

    protected:
        // Derived classes must implement these virtual methods.
        virtual uint32_t writeReceivedData(const struct pbuf *pBuf) = 0;
        virtual bool shouldSendNow(const void* pBuffer, uint16_t bufferLength) = 0;
        virtual void updateConnectionState(bool isConnected) = 0;

        err_t closeServer();
        err_t onAccept(tcp_pcb* pClientPCB, err_t error);
        err_t onRecv(tcp_pcb* pPCB, pbuf* pBuf, err_t error);
        err_t onSent(struct tcp_pcb* pPCB, u16_t length);
        void onError(err_t error);

        static err_t staticAccept(void* pvThis, tcp_pcb* pClientPCB, err_t error)
        {
            return ((SocketServer*)pvThis)->onAccept(pClientPCB, error);
        }
        static void staticError(void* pvThis, err_t error)
        {
            ((SocketServer*)pvThis)->onError(error);
        }
        static err_t staticRecv(void* pvThis, tcp_pcb* pPCB, pbuf* pBuf, err_t error)
        {
            return ((SocketServer*)pvThis)->onRecv(pPCB, pBuf, error);
        }
        static err_t staticSent(void* pvThis, struct tcp_pcb* pPCB, u16_t length)
        {
            return ((SocketServer*)pvThis)->onSent(pPCB, length);
        }

        const char* m_pServerName = NULL;
        tcp_pcb*    m_pListenPCB = NULL;
        tcp_pcb*    m_pClientPCB = NULL;
        volatile uint32_t m_bytesInFlight = 0;
};

#endif // SOCKET_SERVER_H_
