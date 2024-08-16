/* Copyright (C) 2024  Adam Green (https://github.com/adamgreen)

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
/* Hard fault handlers which always infinite loop and optionally ignore semi-host writes to stdout
   until a debugger is attached.
*/
    .text
    .syntax unified
    .thumb


    // Bit in the LR register which indicates that the registers were pushed to PSP on exception entry.
    #define LR_PSP (1 << 2)



    .type mriswd_hardfault_trampoline, %function
    .section .text.mriswd_hardfault_trampoline
    .thumb_func
    /* This assembly language routine loads R0 with the address of the exception record and then calls the function
       pointed to by R3.
    */
mriswd_hardfault_trampoline:
    // Assume they were stacked to MSP and switch to PSP if we find out otherwise.
    mrs     r0, msp
    // Push LR (and R0 for 8-byte alignment) onto the stack.
    push    {r0, lr}
    // Were the volatile registers stacked to PSP or MSP?
    mov     r1, lr
    movs    r2, #LR_PSP
    tst     r1, r2
    beq     1$
    mrs     r0, psp
1$: // R0 now points to where the volatile registers were stacked on fault entry.
    // Call the C function pointed to by R3, with the first parameter set to the address of the exception record.
    blx     r3
    // Return from exception handler.
    pop     {r0, pc}
    // Let assembler know that we have hit the end of the function.
    .pool
    .size   mriswd_hardfault_trampoline, .-mriswd_hardfault_trampoline


    .global mriswd_semihost_stdout_wait_for_debugger_attach
    .type mriswd_semihost_stdout_wait_for_debugger_attach, %function
    .section .text.mriswd_semihost_stdout_wait_for_debugger_attach
    .thumb_func
mriswd_semihost_stdout_wait_for_debugger_attach:
    ldr     r3, =mriswd_semihost_stdout_wait_for_debugger_attach_handler
    b       mriswd_hardfault_trampoline
    // Let assembler know that we have hit the end of the function.
    .pool
    .size   mriswd_semihost_stdout_wait_for_debugger_attach, .-mriswd_semihost_stdout_wait_for_debugger_attach


    .global mriswd_semihost_stdout_ignore_until_debugger_attach
    .type mriswd_semihost_stdout_ignore_until_debugger_attach, %function
    .section .text.mriswd_semihost_stdout_ignore_until_debugger_attach
    .thumb_func
mriswd_semihost_stdout_ignore_until_debugger_attach:
    ldr     r3, =mriswd_semihost_stdout_ignore_until_debugger_attach_handler
    b       mriswd_hardfault_trampoline
    // Let assembler know that we have hit the end of the function.
    .pool
    .size   mriswd_semihost_stdout_ignore_until_debugger_attach, .-mriswd_semihost_stdout_ignore_until_debugger_attach


    .end