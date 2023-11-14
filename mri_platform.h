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
// Routines to expose the Cortex-M debug functionality via SWD to the mri debug core.
#ifndef MRI_PLATFORM_H_
#define MRI_PLATFORM_H_

struct CortexM_Registers
{
    union
    {
        struct
        {
            uint32_t R0;
            uint32_t R1;
            uint32_t R2;
            uint32_t R3;
            uint32_t R4;
            uint32_t R5;
            uint32_t R6;
            uint32_t R7;
            uint32_t R8;
            uint32_t R9;
            uint32_t R10;
            uint32_t R11;
            uint32_t R12;
            uint32_t SP;
            uint32_t LR;
            uint32_t PC;
        };
        uint32_t registers[16];
    };
};

void mainDebuggerLoop();

bool runCodeOnDevice(CortexM_Registers* pRegistersInOut, uint32_t timeout_ms);
bool startCodeOnDevice(const CortexM_Registers* pRegistersIn);
bool waitForCodeToHalt(CortexM_Registers* pRegistersOut, uint32_t timeout_ms);

#endif /* MRI_PLATFORM_H_ */
