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
   - Ignore/drop semi-host writes to stdout when GDB isn't attached but hang on any other semi-host requests.

   NOTE: These routines are currently written to handle the situation on ARMv6-M where BKPT instructions are
         escalated to Hard Faults when there is no SWD debugger attached. A slightly different approach should be
         taken for newer Cortex-M architectures.
*/
#ifndef MRISWD_SEMIHOST_
#define MRISWD_SEMIHOST_


/* This routine will cause an infinite loop to occur when any semi-host request is made without the debugger attached.

   Use the API appropriate for your platform to replace the default Hard Fault handler with this assembly language
   routine.
   Examples:
       Using the Pico C SDK:
           exception_set_exclusive_handler(HARDFAULT_EXCEPTION, mriswd_semihost_stdout_wait_for_debugger_attach);
*/
void mriswd_semihost_stdout_wait_for_debugger_attach(void);


/* This routine will just ignore/drop semi-host writes to stdout when no debugger is attached but it will still enter
   and infinite loop when any other semi-host request is made without the debugger attached.

   Use the API appropriate for your platform to replace the default Hard Fault handler with this assembly language
   routine.
   Examples:
       Using the Pico C SDK:
           exception_set_exclusive_handler(HARDFAULT_EXCEPTION, mriswd_semihost_stdout_ignore_until_debugger_attach);
*/
void mriswd_semihost_stdout_ignore_until_debugger_attach(void);


#endif /* MRISWD_SEMIHOST_*/
