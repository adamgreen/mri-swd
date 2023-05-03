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
// Module which uses LWIP for implementing the GDB communication layer.
#ifndef GDB_SOCKET_H_
#define GDB_SOCKET_H_

#include <lwip/tcp.h>
#include "circular_queue.h"

// The default port number to which GDB can connect.
#define MRI_SWD_BASE_PORT 4242

class GDBSocket
{
    public:
        GDBSocket();

        bool init(uint16_t port = MRI_SWD_BASE_PORT);
        err_t send(const void* pBuffer, uint16_t bufferLength);
        uint32_t bytesInFlight() { return m_bytesInFlight; }
        err_t close();
        err_t closeClient();

        CircularQueue<256> m_tcpToMriQueue;
// UNDONE:        CircularQueue<256> m_mriToTcpQueue;

    protected:
        err_t onAccept(tcp_pcb* pClientPCB, err_t error);
        err_t onRecv(tcp_pcb* pPCB, pbuf* pBuf, err_t error);
        err_t onSent(struct tcp_pcb* pPCB, u16_t length);
        void onError(err_t error);

        static err_t staticAccept(void* pvThis, tcp_pcb* pClientPCB, err_t error)
        {
            return ((GDBSocket*)pvThis)->onAccept(pClientPCB, error);
        }
        static void staticError(void* pvThis, err_t error)
        {
            ((GDBSocket*)pvThis)->onError(error);
        }
        static err_t staticRecv(void* pvThis, tcp_pcb* pPCB, pbuf* pBuf, err_t error)
        {
            return ((GDBSocket*)pvThis)->onRecv(pPCB, pBuf, error);
        }
        static err_t staticSent(void* pvThis, struct tcp_pcb* pPCB, u16_t length)
        {
            return ((GDBSocket*)pvThis)->onSent(pPCB, length);
        }

        tcp_pcb* m_pListenPCB = NULL;
        tcp_pcb* m_pClientPCB = NULL;
        volatile uint32_t m_bytesInFlight = 0;
};

#endif // GDB_SOCKET_H_
