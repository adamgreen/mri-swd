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
/* Circular queue used to communicate between BLE stack and MRI. */
#ifndef CICRCULAR_QUEUE_H_
#define CICRCULAR_QUEUE_H_

#include <stdint.h>
#include <lwip/pbuf.h>
#include "atomic.h"


template<uint32_t QUEUE_SIZE>
class CircularQueue
{
    protected:
        uint32_t m_write;
        uint32_t m_read;
        uint32_t m_peek;
        uint32_t m_peekSize;
        volatile uint32_t m_count;
        volatile uint8_t  m_queue[QUEUE_SIZE];

    public:
        CircularQueue()
        {
            assert ( (sizeof(m_queue) & (sizeof(m_queue)-1)) == 0 );
            init();
        }

        void init()
        {
            m_write = 0;
            m_read = 0;
            m_peek = 0;
            m_peekSize = 0;
            m_count = 0;
        }

        uint32_t write(const uint8_t* pData, uint32_t dataSize)
        {
            assert ( m_count <= sizeof(m_queue) );

            uint32_t bytesLeft = sizeof(m_queue) - m_count;
            uint32_t bytesToWrite = (bytesLeft < dataSize) ? bytesLeft : dataSize;
            for (uint32_t i = 0 ; i < bytesToWrite ; i++)
            {
                m_queue[m_write] = *pData++;
                m_write = incrementIndex(m_write);
            }
            atomic_u32_add(&m_count, bytesToWrite);
            return bytesToWrite;
        }

        uint32_t write(const struct pbuf *pbuf)
        {
            uint32_t bytesWritten = 0;
            for ( ; pbuf != NULL ; pbuf = pbuf->next )
            {
                bytesWritten += write((uint8_t*)pbuf->payload, pbuf->len);
            }
            return bytesWritten;
        }

        uint32_t bytesToRead()
        {
            return m_count;
        }

        uint32_t isFull()
        {
            return bytesToRead() == QUEUE_SIZE;
        }

        uint32_t isEmpty()
        {
            return bytesToRead() == 0;
        }

        uint32_t read(uint8_t* pData, uint32_t dataSize)
        {
            assert ( m_count <= sizeof(m_queue) );

            uint32_t bytesToRead = (dataSize > m_count) ? m_count : dataSize;
            for (uint32_t i = 0 ; i < bytesToRead ; i++)
            {
                *pData++ = m_queue[m_read];
                m_read = incrementIndex(m_read);
            }
            atomic_u32_sub(&m_count, bytesToRead);
            return bytesToRead;
        }

        uint32_t peek(uint8_t* pData, uint32_t dataSize)
        {
            assert ( m_count <= sizeof(m_queue) );
            assert ( m_peekSize == 0 );

            m_peek = m_read;
            uint32_t bytesToRead = (dataSize > m_count) ? m_count : dataSize;
            for (uint32_t i = 0 ; i < bytesToRead ; i++)
            {
                *pData++ = m_queue[m_peek];
                m_peek = incrementIndex(m_peek);
            }
            m_peekSize = bytesToRead;
            return bytesToRead;
        }

        void commitPeek()
        {
            assert ( m_peekSize > 0 );

            m_read = m_peek;
            atomic_u32_sub(&m_count, m_peekSize);
            m_peekSize = 0;
        }

        void rollbackPeek()
        {
            assert ( m_peekSize > 0 );

            m_peekSize = 0;
        }

    protected:
        static uint32_t incrementIndex(uint32_t index)
        {
            return (index + 1) & (QUEUE_SIZE-1);
        }
};

#endif // CICRCULAR_QUEUE_H_
