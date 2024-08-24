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
// Module which uses LWIP for implementing the GDB communication layer.
#ifndef GDB_SOCKET_H_
#define GDB_SOCKET_H_

#include <lwip/tcp.h>
#include "socket_server.h"
#include "circular_queue.h"


class GDBSocket : public SocketServer
{
    public:
        GDBSocket();

        bool init(uint16_t port);
        void uninit();

        bool isGdbConnected()
        {
            return SocketServer::isConnected();
        }

        CircularQueue<16*1024> m_tcpToMriQueue;

    protected:
        virtual uint32_t writeReceivedData(const struct pbuf *pBuf);
        virtual bool shouldSendNow(const void* pBuffer, uint16_t bufferLength);
};

#endif // GDB_SOCKET_H_
