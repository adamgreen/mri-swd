/* Copyright (C) 2020  Adam Green (https://github.com/adamgreen)

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
/* Implementation of code to place known values in registers to verify context reading code. */
    .text
    .syntax unified

    .global testContextWithCrash
    .type testContextWithCrash, %function
    .thumb_func
    /* extern "C" void testContextWithCrash(void);
       Sets all of the registers to a known value to make sure that context is being read correctly.
    */
testContextWithCrash:
    // Save all non-volatile registers on the stack.
    push    {r4-r7, lr}
    mov     r0, r8
    mov     r1, r9
    mov     r2, r10
    mov     r3, r11
    mov     r4, r12
    push    {r0-r4}
    // Load known values into R0-R12
    movs    r0, #8
    movs    r1, #9
    movs    r2, #10
    movs    r3, #11
    movs    r4, #12
    mov     r8, r0
    mov     r9, r1
    mov     r10, r2
    mov     r11, r3
    mov     r12, r4
    movs    r0, #0
    movs    r1, #1
    movs    r2, #2
    movs    r3, #3
    movs    r4, #4
    movs    r5, #5
    movs    r6, #6
    movs    r7, #7
    // Crash on read from invalid memory.
    // "set var $pc=$pc+2" in GDB to resume.
    ldr     r0, =0xFFFFFFF0
    ldr     r0, [r0]
    // Restore non-volatile registers and return to caller.
    pop     {r0-r4}
    mov     r8, r0
    mov     r9, r1
    mov     r10, r2
    mov     r11, r3
    mov     r12, r4
    pop     {r0-r7, pc}
    // Let assembler know that we have hit the end of the function.
    .pool
    .size   testContextWithCrash, .-testContextWithCrash


/* Implementation of code to place known values in registers to verify context reading code. */
    .text
    .syntax unified

    .global testContextWithHardcodedBreakpoint
    .type testContextWithHardcodedBreakpoint, %function
    .thumb_func
    /* extern "C" void testContextWithHardcodedBreakpoint(void);
       Sets all of the registers to a known value to make sure that context is being read correctly.
    */
testContextWithHardcodedBreakpoint:
    // Save all non-volatile registers on the stack.
    push    {r4-r7, lr}
    mov     r0, r8
    mov     r1, r9
    mov     r2, r10
    mov     r3, r11
    mov     r4, r12
    push    {r0-r4}
    // Load known values into R0-R12
    movs    r0, #8
    movs    r1, #9
    movs    r2, #10
    movs    r3, #11
    movs    r4, #12
    mov     r8, r0
    mov     r9, r1
    mov     r10, r2
    mov     r11, r3
    mov     r12, r4
    movs    r0, #0
    movs    r1, #1
    movs    r2, #2
    movs    r3, #3
    movs    r4, #4
    movs    r5, #5
    movs    r6, #6
    movs    r7, #7
    // Hardcoded breakpoint.
    bkpt    #0
    // Restore non-volatile registers and return to caller.
    pop     {r0-r4}
    mov     r8, r0
    mov     r9, r1
    mov     r10, r2
    mov     r11, r3
    mov     r12, r4
    pop     {r4-r7, pc}
    // Let assembler know that we have hit the end of the function.
    .pool
    .size   testContextWithHardcodedBreakpoint, .-testContextWithHardcodedBreakpoint


    .text
    .syntax unified

    .global testStackingHandlerException
    .type testStackingHandlerException, %function
    .thumb_func
    /* extern "C" void testStackingHandlerException(void);
       Sets MSP to an invalid value which will cause a stacking exception hard fault.
    */
testStackingHandlerException:
    ldr     r0, =0xFFFFFFF0
    msr     msp, r0
    ldr     r0, [r0]
    bx      lr
    // Let assembler know that we have hit the end of the function.
    .pool
    .size   testStackingHandlerException, .-testStackingHandlerException


    .end