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
#define SOCKET_SERVER_MODULE "socket_server.cpp"
#include "logging.h"
#include <pico/cyw43_arch.h>
#include "atomic.h"
#include "socket_server.h"


SocketServer::SocketServer()
{
    m_bytesInFlight = 0;
}

bool SocketServer::init(const char* pServerName, uint16_t port)
{
    assert ( !m_pListenPCB && !m_pClientPCB );
    m_bytesInFlight = 0;
    m_pServerName = pServerName;

    logInfoF("Starting %s server at %s on port %u", m_pServerName, ip4addr_ntoa(netif_ip4_addr(netif_list)), port);
    tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb)
    {
        logError("Failed to call tcp_new_ip_type(IPADDR_TYPE_ANY)");
        return false;
    }

    err_t error = tcp_bind(pcb, NULL, port);
    if (error)
    {
        logErrorF("Failed to call tcp_bind(pcb, NULL, %u)", port);
        return false;
    }

    m_pListenPCB = tcp_listen_with_backlog(pcb, 1);
    if (!m_pListenPCB)
    {
        logError("Failed to call tcp_listen_with_backlog(pcb, 1)");
        if (pcb)
        {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(m_pListenPCB, this);
    tcp_accept(m_pListenPCB, staticAccept);

    return true;
}

void SocketServer::uninit()
{
    logInfoF("Shutting down %s server at %s", m_pServerName, ip4addr_ntoa(netif_ip4_addr(netif_list)));
    closeClient();
    closeServer();
}

err_t SocketServer::onAccept(tcp_pcb* pClientPCB, err_t error)
{
    if (error != ERR_OK || pClientPCB == NULL)
    {
        logErrorF("for %s was called with error=%d & pClientPCB=0x%08X", m_pServerName, error, pClientPCB);
        return ERR_VAL;
    }
    if (m_pClientPCB != NULL)
    {
        logErrorF("Can't accept a second connection when %s client is already connected.", m_pServerName);
        return ERR_MEM;
    }
    logInfoF("%s connected.", m_pServerName);

    m_pClientPCB = pClientPCB;
    tcp_arg(pClientPCB, this);
    tcp_sent(pClientPCB, staticSent);
    tcp_recv(pClientPCB, staticRecv);
    tcp_err(pClientPCB, staticError);

    return ERR_OK;
}

err_t SocketServer::closeClient()
{
    err_t error = ERR_OK;
    if (m_pClientPCB)
    {
        tcp_arg(m_pClientPCB, NULL);
        tcp_sent(m_pClientPCB, NULL);
        tcp_recv(m_pClientPCB, NULL);
        tcp_err(m_pClientPCB, NULL);
        error = tcp_close(m_pClientPCB);
        if (error != ERR_OK)
        {
            logErrorF("Failed to call tcp_close(m_pClientPCB) for %s. Aborting.", m_pServerName);
            tcp_abort(m_pClientPCB);
            error = ERR_ABRT;
        }
        m_pClientPCB = NULL;
    }
    m_bytesInFlight = 0;
    return error;
}

err_t SocketServer::closeServer()
{
    err_t error = ERR_OK;
    if (m_pListenPCB)
    {
        tcp_arg(m_pListenPCB, NULL);
        tcp_accept(m_pListenPCB, NULL);
        error = tcp_close(m_pListenPCB);
        if (error != ERR_OK)
        {
            logErrorF("Failed to call tcp_close(m_pListenPCB) for %s. Aborting.", m_pServerName);
            tcp_abort(m_pListenPCB);
            error = ERR_ABRT;
        }
        m_pListenPCB = NULL;
    }
    return error;
}

void SocketServer::onError(err_t error)
{
    logErrorF("Client for %s has hit error %d", m_pServerName, error);
    if (error != ERR_ABRT)
    {
        closeClient();
    }
}

err_t SocketServer::onRecv(tcp_pcb* pPCB, pbuf* pBuf, err_t error)
{
    if (!pBuf)
    {
        logInfoF("%s has disconnected.", m_pServerName);
        return closeClient();
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    if (pBuf->tot_len > 0)
    {
        logDebugF("Received %d bytes from %s w/ error %d", pBuf->tot_len, m_pServerName, error);

        // Process the received packet data.
        uint32_t bytesWritten = writeReceivedData(pBuf);
        if (bytesWritten != pBuf->tot_len)
        {
            logErrorF("%s truncated TCP/IP received packet by %lu bytes.", m_pServerName, pBuf->tot_len - bytesWritten);
        }
        tcp_recved(pPCB, pBuf->tot_len);
    }
    pbuf_free(pBuf);
    return ERR_OK;
}

err_t SocketServer::send(const void* pBuffer, uint16_t bufferLength)
{
    bool sendNow = shouldSendNow(pBuffer, bufferLength);

    logDebugF("Writing %d bytes to %s client", bufferLength, m_pServerName);
    while (m_pClientPCB && bufferLength > 0)
    {
        cyw43_arch_lwip_begin();
            err_t error = ERR_OK;
            uint32_t maxBytesToSend = tcp_sndbuf(m_pClientPCB);
            logDebugF("tcp_sndbuf() for %s returned %lu", m_pServerName, maxBytesToSend);
            if (maxBytesToSend > 0)
            {
                bool outputNow = false;
                uint32_t bytesToSend = bufferLength;
                if (bytesToSend > maxBytesToSend)
                {
                    // Network stack is full so request data to be sent now to free up space rather than nagle.
                    bytesToSend = maxBytesToSend;
                    outputNow = true;
                }

                error = tcp_write(m_pClientPCB, pBuffer, bytesToSend, TCP_WRITE_FLAG_COPY);
                if (error == ERR_OK)
                {
                    atomic_u32_add(&m_bytesInFlight, bytesToSend);
                    bufferLength -= bytesToSend;
                    pBuffer = (const uint8_t*)pBuffer + bytesToSend;
                    if (outputNow || (bufferLength == 0 && sendNow))
                    {
                        // Send data now if it is long enough to be a packet for which GDB is waiting.
                        // UNDONE: Just ignoring errors for now.
                        tcp_output(m_pClientPCB);
                    }
                }
            }
        cyw43_arch_lwip_end();

        if (error != ERR_OK)
        {
            logErrorF("Failed call to tcp_write(m_pClientPCB, 0x%08lX, %u, TCP_WRITE_FLAG_COPY) for %s with error %d",
                pBuffer, bufferLength, m_pServerName, error);
            return error;
        }
    }
    if (!m_pClientPCB)
    {
        logErrorF("%s socket has been closed so %d bytes have been discarded.", m_pServerName, bufferLength);
    }

    return ERR_OK;
}

err_t SocketServer::onSent(struct tcp_pcb* pPCB, u16_t length)
{
    logDebugF("tcp_server_sent %u bytes to %s", length, m_pServerName);
    atomic_u32_sub(&m_bytesInFlight, length);
    return ERR_OK;
}
