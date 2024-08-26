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
#define GDB_SOCKET_MODULE "gdb_socket.cpp"
#include "logging.h"
#include "gdb_socket.h"
#include "ui.h"


GDBSocket::GDBSocket()
{
    m_bytesInFlight = 0;
}

bool GDBSocket::init(uint16_t port)
{
    return SocketServer::init("GDB", port);
}

void GDBSocket::uninit()
{
    SocketServer::uninit();
}

uint32_t GDBSocket::writeReceivedData(const struct pbuf *pBuf)
{
    // Update UI to signal activity to user.
    UI::receivingFromGdb();

    // Place the received packet data into the circular queue.
    return m_tcpToMriQueue.write(pBuf);
}

bool GDBSocket::shouldSendNow(const void* pBuffer, uint16_t bufferLength)
{
    // Update UI to signal activity to user.
    UI::transmittingToGdb();

    // Minimum packet is $#00
    const uint16_t minPacketLength = 4;
    bool isPacketData = bufferLength >= minPacketLength;
    return isPacketData;
}

void GDBSocket::updateConnectionState(bool isConnected)
{
    UI::setGdbConnectedState(isConnected);
}
