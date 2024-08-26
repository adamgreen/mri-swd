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
#define UART_WIFI_BRIDGE_MODULE "uart_wifi_bridge.cpp"
#include "logging.h"
#include <pico/stdlib.h>
#include "uart_wifi_bridge.h"
#include "ui.h"

// The address of the singleton to which UART and DMA ISR requests should be made.
UartWiFiBridge* UartWiFiBridge::s_pThis = NULL;



UartWiFiBridge::UartWiFiBridge()
{
    m_bytesInFlight = 0;
}

bool UartWiFiBridge::init(uint16_t port, uint32_t uartTxPin, uint32_t uartRxPin, uint32_t uartBaudRate)
{
    uart_inst_t* pTxUartInstance = txPinToUartInstance(uartTxPin);
    uart_inst_t* pRxUartInstance = rxPinToUartInstance(uartRxPin);

    // Both pins must use the same UART instance.
    assert ( pTxUartInstance != NULL && pRxUartInstance != NULL );
    assert ( pTxUartInstance == pRxUartInstance );
    m_pUartInstance = pTxUartInstance;

    // Select correct interrupt for the UART we are using.
    int m_uartIRQ = (m_pUartInstance == uart0) ? UART0_IRQ : UART1_IRQ;

    // Switch the GPIO pins over to the UART.
    gpio_set_function(uartTxPin, GPIO_FUNC_UART);
    gpio_set_function(uartRxPin, GPIO_FUNC_UART);

    // Initialize the UART instance.
    uart_init(m_pUartInstance, uartBaudRate);

    // Remember this object as the UartWiFiBridge singleton before enabling any interrupts.
    s_pThis = this;

    // Set up and enable the interrupt handlers for UART Rx and Tx.
    // Must run the UART interrupt at the lowest priority, the same as the network stack so that one can't interrupt the
    // other.
    irq_set_exclusive_handler(m_uartIRQ, staticUartISR);
    irq_set_priority(m_uartIRQ, PICO_LOWEST_IRQ_PRIORITY);
    irq_set_enabled(m_uartIRQ, true);
    uart_set_irq_enables(m_pUartInstance, true, false);

    // Increase the Rx FIFO threshold to the maximum level instead of the minimum.
    // This allows sending bigger TCP/IP packets to the PC.
    hw_write_masked(&uart_get_hw(m_pUartInstance)->ifls,
        4 << UART_UARTIFLS_RXIFLSEL_LSB,
        UART_UARTIFLS_RXIFLSEL_BITS);

    // Setup for future DMA transfers used to transmit data via UART.
    m_dmaChannel = dma_claim_unused_channel(true);
    m_dmaConfig = dma_channel_get_default_config(m_dmaChannel);
    uint32_t dreq = (m_pUartInstance == uart0) ? DREQ_UART0_TX : DREQ_UART1_TX;
    channel_config_set_dreq(&m_dmaConfig, dreq);
    channel_config_set_transfer_data_size(&m_dmaConfig, DMA_SIZE_8);

    // Setup the DMA interrupt on DMA_IRQ_0.
    m_dmaIRQ = DMA_IRQ_0;
    irq_set_exclusive_handler(m_dmaIRQ, staticDmaISR);
    irq_set_enabled(m_dmaIRQ, true);
    dma_channel_set_irq0_enabled(m_dmaChannel, true);

    bool result = SocketServer::init("UART<->WiFi", port);
    if (!result)
    {
        uninit();
        return false;
    }
    return result;
}

uart_inst_t* UartWiFiBridge::txPinToUartInstance(uint32_t txPin)
{
    uart_inst_t* const mapPinToUart[30] = {
        uart0, NULL, NULL, NULL,
        uart1, NULL, NULL, NULL,
        uart1, NULL, NULL, NULL,
        uart0, NULL, NULL, NULL,
        uart0, NULL, NULL, NULL,
        uart1, NULL, NULL, NULL,
        uart1, NULL, NULL, NULL,
        uart0, NULL };
    if (txPin >= count_of(mapPinToUart))
    {
        return NULL;
    }
    return mapPinToUart[txPin];
}

uart_inst_t* UartWiFiBridge::rxPinToUartInstance(uint32_t rxPin)
{
    // The UART Rx pins are 1 higher than the corresponding Tx pins.
    // A pin of 0 will underflow to 0xFFFFFFFF which will still cause txPinToUartInstance() to return NULL as expected.
    return txPinToUartInstance(rxPin - 1);
}

void UartWiFiBridge::uartISR()
{
    const size_t UART_FIFO_SIZE = 32;
    uint8_t buffer[UART_FIFO_SIZE];

    // Take data in the UART's Rx FIFO and send it out over the network.
    size_t i;
    for (i = 0 ; i < count_of(buffer) && uart_is_readable(m_pUartInstance) ; i++)
    {
        buffer[i] = uart_get_hw(m_pUartInstance)->dr;
    }
    if (i > 0)
    {
        SocketServer::send(buffer, i);
    }
}

void UartWiFiBridge::dmaISR()
{
    // Mark the chunk just sent as completed and free it up in the circular queue.
    m_isDmaTransmitting = false;
    m_tcpToUartQueue.commitPeek();
    dma_channel_acknowledge_irq0(m_dmaChannel);

    // Fetch next chunk of data from the circular queue and start another DMA transfer to UART.
    sendQueuedDataToUart();
}

void UartWiFiBridge::sendQueuedDataToUart()
{
    // If there is already a DMA transaction in process then this method will be called once it completes and there is
    // nothing to do at this point.
    if (m_isDmaTransmitting)
    {
        return;
    }

    // Fetch next chunk of data from the circular queue.
    uint8_t* pChunk = NULL;
    uint32_t chunkSize = 0;
    m_tcpToUartQueue.peekForDMA(&pChunk, &chunkSize);
    if (chunkSize == 0)
    {
        // No more data to send so just return immediately.
        return;
    }

    // Queue up the next UART transmit using the DMA.
    m_isDmaTransmitting = true;
    dma_channel_configure(
            m_dmaChannel,
            &m_dmaConfig,
            &uart_get_hw(m_pUartInstance)->dr,
            pChunk,
            chunkSize,
            true);
}

void UartWiFiBridge::uninit()
{
    SocketServer::uninit();
    if (m_dmaIRQ != 0)
    {
        irq_set_enabled(m_dmaIRQ, false);
        irq_set_enabled(m_uartIRQ, false);
        dma_channel_cleanup(m_dmaChannel);
        uart_deinit(m_pUartInstance);

        m_pUartInstance = NULL;
        m_uartIRQ = 0;
        m_dmaIRQ = 0;
        m_dmaChannel = 0;
        m_isDmaTransmitting = false;
    }
    s_pThis = NULL;
}


uint32_t UartWiFiBridge::writeReceivedData(const struct pbuf *pBuf)
{
    // Update UI to signal activity to user.
    UI::transmittingToUart();

    uint32_t bytesWritten = m_tcpToUartQueue.write(pBuf);
    irq_set_enabled(m_dmaIRQ, false);
        sendQueuedDataToUart();
    irq_set_enabled(m_dmaIRQ, true);
    return bytesWritten;
}

bool UartWiFiBridge::shouldSendNow(const void* pBuffer, uint16_t bufferLength)
{
    // Update UI to signal activity to user.
    UI::receivingFromUart();

    // Don't want to call tcp_output from UART ISR.
    return false;
}

void UartWiFiBridge::updateConnectionState(bool isConnected)
{
    UI::setUartConnectedState(isConnected);
}
