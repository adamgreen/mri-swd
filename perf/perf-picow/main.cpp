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
#include <hardware/sync.h>
#include <pico/stdlib.h>
#include <pico/cyw43_arch.h>
#include <lwip/tcp.h>


static tcp_pcb* volatile g_pClientPCB = NULL;
static volatile uint32_t g_count = 0;
static volatile uint32_t g_currRequest = 0;


// Function Prototypes.
static bool connectToWiFiRouter();
static err_t onAccept(void* pvThis, tcp_pcb* pClientPCB, err_t error);
static void onError(void* pvThis, err_t error);
static err_t closeClient();
static err_t onRecv(void* pvThis, tcp_pcb* pPCB, pbuf* pBuf, err_t error);
static err_t onSent(void* pvThis, struct tcp_pcb* pPCB, u16_t length);


int main()
{
    uint32_t currRequest = g_currRequest;

    stdio_init_all();

    // UNDONE: Giving time for serial terminal to reconnect.
    sleep_ms(2000);

    printf("Starting up...\n");

    // Fill the buffer.
    uint32_t buffer[14336/sizeof(uint32_t)];
    for (size_t i = 0 ; i < count_of(buffer) ; i++)
    {
        buffer[i] = 0xBAADFEED;
    }


    printf("Connecting to Wi-Fi router...\n");
    bool result = connectToWiFiRouter();
    if (!result)
    {
        printf("Failed to connect to Wi-Fi router.\n");
        __breakpoint();
        return 1;
    }
    printf("Connected.\n");


    // Initialize the TCP/IP stack.
    const uint16_t port = 4242;
    printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), port);

    tcp_pcb* pBindPCB = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pBindPCB)
    {
        printf("Failed to call tcp_new_ip_type(IPADDR_TYPE_ANY)\n");
        __breakpoint();
        return 1;
    }

    err_t error = tcp_bind(pBindPCB, NULL, port);
    if (error)
    {
        printf("Failed to call tcp_bind(pcb, NULL, %u)\n", port);
        __breakpoint();
        return 1;
    }

    tcp_pcb* pListenPCB = tcp_listen_with_backlog(pBindPCB, 1);
    if (!pListenPCB)
    {
        printf("Failed to call tcp_listen_with_backlog(pcb, 1)\n");
        if (pBindPCB)
        {
            tcp_close(pBindPCB);
        }
        __breakpoint();
        return 1;
    }

    tcp_arg(pListenPCB, NULL);
    tcp_accept(pListenPCB, onAccept);


    while (true)
    {
        printf("Waiting for client to connect...\n");
        while (g_pClientPCB == NULL)
        {
        }
        printf("Client is now connected.\n");

        printf("Waiting for client to send test length...\n");
        while (g_currRequest == currRequest)
        {
        }
        currRequest = g_currRequest;
        printf("Client has sent request for %lu bytes.\n", g_count);

        printf("Writing %lu bytes to client\n", g_count);
        uint32_t bytesLeft = g_count;
        uint32_t bytesQueued = 0;
        while (bytesLeft > 0 && g_pClientPCB != NULL)
        {
            cyw43_arch_lwip_begin();
                err_t error = ERR_OK;
                bool writeSuccessful = false;
                uint32_t bytesToSend = sizeof(buffer);
                uint32_t maxBytesToSend = tcp_sndbuf(g_pClientPCB);
                if (maxBytesToSend > sizeof(buffer))
                {
                    if (bytesToSend > bytesLeft)
                    {
                        bytesToSend = bytesLeft;
                    }

                    error = tcp_write((tcp_pcb*)g_pClientPCB, buffer, bytesToSend, TCP_WRITE_FLAG_COPY);
                    if (error == ERR_OK)
                    {
                        bytesLeft -= bytesToSend;
                        bytesQueued += bytesToSend;
                        writeSuccessful = true;
                    }
                }
                else
                {
                    cyw43_arch_poll();
                }
                if (bytesLeft == 0 || (bytesQueued > 0 && !writeSuccessful))
                {
                    // Keep queueing up data until we can fit no more and then trigger lwip to send it.
                    tcp_output((tcp_pcb*)g_pClientPCB);
                    bytesQueued = 0;
                }
            cyw43_arch_lwip_end();

            if (error != ERR_OK)
            {
                printf("tcp_write() failed.\n");
                __breakpoint();
            }
        }

        printf("Finished sending %lu bytes of test data.\n", g_count-bytesLeft);
        printf("Closing client connection.\n");
        closeClient();
    }

    cyw43_arch_deinit();
    return 0;
}

static bool connectToWiFiRouter()
{
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_USA))
    {
        printf("Wi-Fi chip failed to initialize.\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("Failed to connect to Wi-Fi.\n");
        return false;
    }

    return true;
}

static err_t onAccept(void* pvThis, tcp_pcb* pClientPCB, err_t error)
{
    if (error != ERR_OK || pClientPCB == NULL)
    {
        printf("onAccept() was called with error=%d & pClientPCB=0x%08lX\n", error, (uint32_t)pClientPCB);
        return ERR_VAL;
    }

    g_pClientPCB = pClientPCB;
    tcp_arg(pClientPCB, NULL);
    tcp_sent(pClientPCB, onSent);
    tcp_recv(pClientPCB, onRecv);
    tcp_err(pClientPCB, onError);

    return ERR_OK;
}

static void onError(void* pvThis, err_t error)
{
    printf("Client socket has hit error %d\n", error);
    if (error != ERR_ABRT)
    {
        closeClient();
    }
}

static err_t closeClient()
{
    err_t error = ERR_OK;
    uint32_t state = save_and_disable_interrupts();
    if (g_pClientPCB != NULL)
    {
        tcp_arg((tcp_pcb*)g_pClientPCB, NULL);
        tcp_sent((tcp_pcb*)g_pClientPCB, NULL);
        tcp_recv((tcp_pcb*)g_pClientPCB, NULL);
        tcp_err((tcp_pcb*)g_pClientPCB, NULL);
        error = tcp_close((tcp_pcb*)g_pClientPCB);
        if (error != ERR_OK)
        {
            printf("Failed to call tcp_close(g_pClientPCB)\n");
            tcp_abort((tcp_pcb*)g_pClientPCB);
            error = ERR_ABRT;
        }
        g_pClientPCB = NULL;
    }
    restore_interrupts(state);

    return error;
}

static err_t onRecv(void* pvThis, tcp_pcb* pPCB, pbuf* pBuf, err_t error)
{
    if (!pBuf)
    {
        printf("Client has disconnected.\n");
        return closeClient();
    }

    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    if (pBuf->tot_len > 0)
    {
        printf("Received %d bytes w/ error %d\n", pBuf->tot_len, error);

        // Receive the buffer
        uint16_t length = pbuf_copy_partial(pBuf, (void*)&g_count, pBuf->tot_len > sizeof(g_count) ? sizeof(g_count) : pBuf->tot_len, 0);
        if (length > 0)
        {
            g_currRequest++;
        }
        tcp_recved(pPCB, pBuf->tot_len);
    }
    pbuf_free(pBuf);

    return ERR_OK;
}

static err_t onSent(void* pvThis, struct tcp_pcb* pPCB, u16_t length)
{
    return ERR_OK;
}
