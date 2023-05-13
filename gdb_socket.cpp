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
#define GDB_SOCKET_MODULE "gdb_socket.cpp"
#include "logging.h"
#include <pico/cyw43_arch.h>
#include "gdb_socket.h"


// UNDONE: Get rid of this later.
#define BUF_SIZE 2048


GDBSocket::GDBSocket()
{
    m_bytesInFlight = 0;
}

bool GDBSocket::init(uint16_t port)
{
    logInfoF("Starting server at %s on port %u", ip4addr_ntoa(netif_ip4_addr(netif_list)), port);

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

err_t GDBSocket::onAccept(tcp_pcb* pClientPCB, err_t error)
{
    if (error != ERR_OK || pClientPCB == NULL)
    {
        logErrorF("was called with error=%d & pClientPCB=0x%08X", error, pClientPCB);
        return ERR_VAL;
    }
    logInfo("GDB connected.");

    m_pClientPCB = pClientPCB;
    tcp_arg(pClientPCB, this);
    tcp_sent(pClientPCB, staticSent);
    tcp_recv(pClientPCB, staticRecv);
    tcp_err(pClientPCB, staticError);

    return ERR_OK;
}

err_t GDBSocket::closeClient()
{
    err_t error = ERR_OK;
    if (m_pClientPCB != NULL)
    {
        tcp_arg(m_pClientPCB, NULL);
        tcp_poll(m_pClientPCB, NULL, 0);
        tcp_sent(m_pClientPCB, NULL);
        tcp_recv(m_pClientPCB, NULL);
        tcp_err(m_pClientPCB, NULL);
        error = tcp_close(m_pClientPCB);
        if (error != ERR_OK)
        {
            logError("Failed to call tcp_close(m_pClientPCB)");
            tcp_abort(m_pClientPCB);
            error = ERR_ABRT;
        }
        m_pClientPCB = NULL;
    }

    return error;
}

err_t GDBSocket::close()
{
    err_t error = closeClient();
    if (m_pListenPCB != NULL)
    {
        tcp_arg(m_pListenPCB, NULL);
        tcp_close(m_pListenPCB);
        m_pListenPCB = NULL;
    }
    return error;
}

void GDBSocket::onError(err_t error)
{
    logErrorF("Client socket has hit error %d", error);
    if (error != ERR_ABRT)
    {
        closeClient();
    }
}

err_t GDBSocket::onRecv(tcp_pcb* pPCB, pbuf* pBuf, err_t error)
{
    if (!pBuf)
    {
        logInfo("GDB has disconnected.");
        return closeClient();
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    if (pBuf->tot_len > 0)
    {
        logDebugF("Received %d bytes w/ error %d", pBuf->tot_len, error);

        // Receive the buffer
        // UNDONE: Find a better way to do this.
        uint8_t buffer[BUF_SIZE];
        uint16_t length = pbuf_copy_partial(pBuf, buffer, pBuf->tot_len > sizeof(buffer) ? sizeof(buffer) : pBuf->tot_len, 0);
        if (length > 0)
        {
            // UNDONE: What should I do if this fails to queue up all of the data?
            m_tcpToMriQueue.write(buffer, length);
        }
        tcp_recved(pPCB, pBuf->tot_len);
    }
    pbuf_free(pBuf);
    return ERR_OK;
}

err_t GDBSocket::send(const void* pBuffer, uint16_t bufferLength)
{
    // Minimum packet is $#00
    const uint16_t minPacketLength = 4;
    bool isPacketData = bufferLength >= minPacketLength;

    logDebugF("Writing %d bytes to client", bufferLength);
    while (bufferLength > 0)
    {
        cyw43_arch_lwip_begin();
            err_t error = ERR_OK;
            uint32_t maxBytesToSend = tcp_sndbuf(m_pClientPCB);
            logDebugF("tcp_sndbuf() returned %lu", maxBytesToSend);
            if (maxBytesToSend > 0)
            {
                bool outputNow = false;
                uint32_t bytesToSend = bufferLength;
                if (bytesToSend > maxBytesToSend)
                {
                    bytesToSend = maxBytesToSend;
                    outputNow = true;
                }

                error = tcp_write(m_pClientPCB, pBuffer, bytesToSend, TCP_WRITE_FLAG_COPY);
                if (error == ERR_OK)
                {
                    atomic_u32_add(&m_bytesInFlight, bytesToSend);
                    bufferLength -= bytesToSend;
                    pBuffer = (const uint8_t*)pBuffer + bytesToSend;
                    if (outputNow || (bufferLength == 0 && isPacketData))
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
            logErrorF("Failed call to tcp_write(m_pClientPCB, 0x%08lX, %u, TCP_WRITE_FLAG_COPY) with error %d", pBuffer, bufferLength, error);
            return error;
        }
    }

    return ERR_OK;
}

err_t GDBSocket::onSent(struct tcp_pcb* pPCB, u16_t length)
{
    logDebugF("tcp_server_sent %u bytes", length);
    atomic_u32_sub(&m_bytesInFlight, length);
    return ERR_OK;
}
