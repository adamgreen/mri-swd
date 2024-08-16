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
/* Routines used to handle the situation where semi-host requests are made when the mri-swd debugger isn't attached:
   - Just hang and wait for GDB to attach on any semi-host request.
   - Ignore semi-host writes to stdout when GDB isn't attached but hang on any other semi-host requests.

   NOTE: These routines are currently written to handle the situation on ARMv6-M where BKPT instructions are
         escalated to Hard Faults when there is no SWD debugger attached. A slightly different approach should be
         taken for newer Cortex-M architectures.
*/
#include <stdint.h>
#include "newlib_stubs.h"
#include "semihost_arm.h"


// Machine code for the BKPT instruction.
#define BKPT_INSTR_MACHINE_CODE 0xBE00

// The Posix fileno used for stdout.
#define STDOUT_FILENO 1



// This structure contains the integer registers that are automatically stacked by Cortex-M processor when it enters
// an exception handler.
typedef struct
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
} ExceptionStackedRegisters;



// Hard Fault handler which keeps retrying BKPT instructions (like those used for semi-hosting) until debugger attaches.
void mriswd_semihost_stdout_wait_for_debugger_attach_handler(ExceptionStackedRegisters* pException)
{
    // Load the 16-bit instruction pointed to by PC.
    uint16_t faultingInsruction = *(uint16_t*)pException->pc;
    // Check to see if this instruction was a BKPT or not.
    if ((faultingInsruction & BKPT_INSTR_MACHINE_CODE) == BKPT_INSTR_MACHINE_CODE)
    {
        // Just return to the faulting BKPT instruction to let it fault again until the debugger is attached.
        return;
    }
    // Not a BKPT so infinite loop here.
    while (1)
    {
    }
}


// Hard fault handler which ignores semi-host write requests to stdout and infinite loops on all other hard faults
// until debugger is attached.
void mriswd_semihost_stdout_ignore_until_debugger_attach_handler(ExceptionStackedRegisters* pException)
{
    // Load the 16-bit instruction pointed to by PC.
    uint16_t faultingInsruction = *(uint16_t*)pException->pc;

    // Check to see if this instruction was a BKPT or not.
    if ((faultingInsruction & BKPT_INSTR_MACHINE_CODE) == BKPT_INSTR_MACHINE_CODE)
    {
        switch (faultingInsruction)
        {
            case BKPT_INSTR_MACHINE_CODE + MRI_ARM_SEMIHOST_BKPT_NO:
                {
                    typedef struct
                    {
                        uint32_t        fileDescriptor;
                        uint32_t        bufferAddress;
                        int32_t         bufferSize;
                    } TransferParameters;
                    TransferParameters* pParams = (TransferParameters*)pException->r1;

                    // Is a write semi-host request to stdout being made?
                    if (pException->r0 == MRI_ARM_SEMIHOST_WRITE && pParams->fileDescriptor == STDOUT_FILENO)
                    {
                        // Set the return value (R0) to 0 to fake that all bytes have been written.
                        pException->r0 = 0;
                        // Advance the PC past the BKPT instruction and allow the rest of the _write() call to execute.
                        pException->pc += 2;
                        return;
                    }
                    // Just return to the faulting semi-host BKPT instruction to let it fault again
                    // until the debugger is attached.
                    return;
                }
                break;
            case BKPT_INSTR_MACHINE_CODE + MRI_NEWLIB_SEMIHOST_WRITE:
                {
                    // Is the file number of the _write() call set to stdout?
                    if (pException->r0 == STDOUT_FILENO)
                    {
                        // Set the return value (R0) to the write length (R2) to fake that all bytes have been written.
                        pException->r0 = pException->r2;
                        // Advance the PC past the BKPT instruction and allow the rest of the _write() call to execute.
                        pException->pc += 2;
                        return;
                    }
                    // Just return to the faulting semi-host BKPT instruction to let it fault again
                    // until the debugger is attached.
                    return;
                }
                break;
            default:
                // Just return to the faulting BKPT instruction to let it fault again until the debugger is attached.
                return;
        }
    }

    // Not a BKPT so infinite loop here.
    while (1)
    {
    }
}
