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
#include <pico/cyw43_arch.h>
#include "gdb_socket.h"


// UNDONE: Get rid of this later.
#define BUF_SIZE 2048


// Macros used to enable/disable logging of SWD error conditions.
#define logFailure(X) errorf("%s:%u %s failed calling " X "\n", __FILE__, __LINE__, __FUNCTION__)
static int (*errorf)(const char* format, ...) = printf;
static int (*logf)(const char* format, ...) = printf;


GDBSocket::GDBSocket()
{
    m_bytesInFlight = 0;
}

bool GDBSocket::init(uint16_t port)
{
    logf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), port);

    tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb)
    {
        logFailure("tcp_new_ip_type(IPADDR_TYPE_ANY)");
        return false;
    }

    err_t error = tcp_bind(pcb, NULL, port);
    if (error)
    {
        logFailure("tcp_bind()");
        return false;
    }

    m_pListenPCB = tcp_listen_with_backlog(pcb, 1);
    if (!m_pListenPCB)
    {
        logFailure("tcp_listen_with_backlog()");
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
        errorf("%s:%u %s was called with error=%d & pClientPCB=0x%08X\n",
            __FILE__, __LINE__, __FUNCTION__, error, pClientPCB);
// UNDONE:        handleError(error);
        return ERR_VAL;
    }
    logf("Client connected.\n");

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
            logFailure("tcp_close(m_pClientPCB)");
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
    logf("Client socket has hit error %d\n", error);
    if (error != ERR_ABRT)
    {
        closeClient();
    }
}

err_t GDBSocket::onRecv(tcp_pcb* pPCB, pbuf* pBuf, err_t error)
{
    if (!pBuf)
    {
        logf("Remote client has disconnected.\n");
        return closeClient();
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    if (pBuf->tot_len > 0)
    {
        logf("Received %d bytes w/ error %d\n", pBuf->tot_len, error);

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
    // UNDONE: Should call tcp_sndbuf() to figure out maximum data to send.
    logf("Writing %d bytes to client\n", bufferLength);
    cyw43_arch_lwip_begin();
        err_t error = tcp_write(m_pClientPCB, pBuffer, bufferLength, TCP_WRITE_FLAG_COPY);
    cyw43_arch_lwip_end();
    if (error != ERR_OK)
    {
        logFailure("Failed to write data\n");
        // UNDONE: Might not want to call close on this error path.
        __breakpoint();
        return closeClient();
    }
    atomic_u32_add(&m_bytesInFlight, bufferLength);

    return ERR_OK;
}

err_t GDBSocket::onSent(struct tcp_pcb* pPCB, u16_t length)
{
    logf("tcp_server_sent %u\n", length);
    atomic_u32_sub(&m_bytesInFlight, length);
    return ERR_OK;
}
