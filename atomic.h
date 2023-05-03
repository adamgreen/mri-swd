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
// Atomic add/subtract function.
#ifndef ATOMIC_H_
#define ATOMIC_H_

#include <stdint.h>
#include <hardware/sync.h>


static inline uint32_t atomic_u32_add(volatile uint32_t* pData, uint32_t increment)
{
    uint32_t state = save_and_disable_interrupts();
        uint32_t orig = *pData;
        uint32_t next = orig + increment;
        *pData = next;
    restore_interrupts(state);
    return next;
}

static inline uint32_t atomic_u32_sub(volatile uint32_t* pData, uint32_t decrement)
{
    uint32_t state = save_and_disable_interrupts();
        uint32_t orig = *pData;
        uint32_t next = orig - decrement;
        *pData = next;
    restore_interrupts(state);
    return next;
}

#endif // ATOMIC_H_
