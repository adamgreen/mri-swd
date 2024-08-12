# mri-swd Test Pass
This README documents the manual steps that I conduct in GDB when running a test pass on the [mri-swd](https://github.com/user/mri-swd) against a [Pico target](https://www.raspberrypi.com/products/raspberry-pi-pico/). This repository also contains the source code for the [test program](main.c) that I run on the target Pico board while running through the steps in this test pass.


## Build and Deploy Test Program
* Build the test program for the Pico by typing the following in a Terminal window: ```make clean all```
* Connect to **mri-swd** with GDB by using the following command (note that the IP address for the **mri-swd** probe changes over time): ```./debug 10.0.0.201```
* Upload the new code to the Pico with this GDB command: ```load```
* Start the test code running on the Pico with this GDB command: ```continue```
* Once the initial menu of test options is displayed in GDB, press **CTRL+C** to halt the target.

The test code uses the semi-hosting support built into **mri-swd** to redirect its standard output (stdout) and standard input (stdin) to/from the GDB terminal so all interactions with the test can occur from within GDB itself. No Terminal program is required.

```console
user@Mac TestPass % make clean all
Removing build output for clean build...
Building...
PICO_SDK_PATH is /Users/user/depots/mri-swd/pico-sdk
Defaulting PICO_PLATFORM to rp2040 since not specified.
Defaulting PICO platform compiler to pico_arm_gcc since not specified.
-- Defaulting build type to 'Release' since not specified.
PICO compiler is pico_arm_gcc
-- The C compiler identification is GNU 13.3.1
-- The CXX compiler identification is GNU 13.3.1
-- The ASM compiler identification is GNU
-- Found assembler: /Users/user/depots/gcc-arm-none-eabi/install-native/bin/arm-none-eabi-gcc
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: /Users/user/depots/gcc-arm-none-eabi/install-native/bin/arm-none-eabi-gcc - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: /Users/user/depots/gcc-arm-none-eabi/install-native/bin/arm-none-eabi-g++ - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
Build type is Debug
Using regular optimized debug build (set PICO_DEOPTIMIZED_DEBUG=1 to de-optimize)
PICO target board is pico.
Using board configuration from /Users/user/depots/mri-swd/pico-sdk/src/boards/include/boards/pico.h
-- Found Python3: /opt/homebrew/Frameworks/Python.framework/Versions/3.10/bin/python3.10 (found version "3.10.7") found components: Interpreter
TinyUSB available at /Users/user/depots/mri-swd/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040; enabling build support for USB.
Compiling TinyUSB with CFG_TUSB_DEBUG=1
BTstack available at /Users/user/depots/mri-swd/pico-sdk/lib/btstack
cyw43-driver available at /Users/user/depots/mri-swd/pico-sdk/lib/cyw43-driver
Pico W Bluetooth build support available.
lwIP available at /Users/user/depots/mri-swd/pico-sdk/lib/lwip
mbedtls available at /Users/user/depots/mri-swd/pico-sdk/lib/mbedtls
-- Configuring done (0.6s)
-- Generating done (0.1s)
-- Build files have been written to: /Users/user/depots/mri-swd/tests/TestPass/build
[  1%] Building ASM object pico-sdk/src/rp2_common/boot_stage2/CMakeFiles/bs2_default.dir/compile_time_choice.S.obj
[  2%] Linking ASM executable bs2_default.elf
[  2%] Built target bs2_default
[  4%] Generating bs2_default.bin
[  5%] Generating bs2_default_padded_checksummed.S
[  5%] Built target bs2_default_padded_checksummed_asm
[  7%] Creating directories for 'ELF2UF2Build'
[  8%] No download step for 'ELF2UF2Build'
[  9%] No update step for 'ELF2UF2Build'
[ 11%] No patch step for 'ELF2UF2Build'
[ 12%] Performing configure step for 'ELF2UF2Build'
-- The C compiler identification is AppleClang 15.0.0.15000309
-- The CXX compiler identification is AppleClang 15.0.0.15000309
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: /Library/Developer/CommandLineTools/usr/bin/cc - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: /Library/Developer/CommandLineTools/usr/bin/c++ - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Configuring done (0.3s)
-- Generating done (0.0s)
-- Build files have been written to: /Users/user/depots/mri-swd/tests/TestPass/build/elf2uf2
[ 14%] Performing build step for 'ELF2UF2Build'
[ 50%] Building CXX object CMakeFiles/elf2uf2.dir/main.cpp.o
[100%] Linking CXX executable elf2uf2
[100%] Built target elf2uf2
[ 15%] No install step for 'ELF2UF2Build'
[ 16%] Completed 'ELF2UF2Build'
[ 16%] Built target ELF2UF2Build
[ 18%] Building C object CMakeFiles/mri-swd-tests.dir/main.c.obj
[ 19%] Building ASM object CMakeFiles/mri-swd-tests.dir/tests.S.obj
[ 21%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/shared/newlib_retarget.c.obj
[ 22%] Building ASM object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/shared/newlib_stubs.S.obj
[ 23%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_stdlib/stdlib.c.obj
[ 25%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_gpio/gpio.c.obj
[ 26%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_platform/platform.c.obj
[ 28%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_claim/claim.c.obj
[ 29%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_sync/sync.c.obj
[ 30%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_irq/irq.c.obj
[ 32%] Building ASM object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_irq/irq_handler_chain.S.obj
[ 33%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/common/pico_sync/sem.c.obj
[ 35%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/common/pico_sync/lock_core.c.obj
[ 36%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/common/pico_sync/mutex.c.obj
[ 38%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/common/pico_sync/critical_section.c.obj
[ 39%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/time.c.obj
[ 40%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/timeout_helper.c.obj
[ 42%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_timer/timer.c.obj
[ 43%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/common/pico_util/datetime.c.obj
[ 45%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/common/pico_util/pheap.c.obj
[ 46%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/common/pico_util/queue.c.obj
[ 47%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_uart/uart.c.obj
[ 49%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_clocks/clocks.c.obj
[ 50%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_pll/pll.c.obj
[ 52%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_vreg/vreg.c.obj
[ 53%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_watchdog/watchdog.c.obj
[ 54%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_xosc/xosc.c.obj
[ 56%] Building ASM object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_divider/divider.S.obj
[ 57%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_runtime/runtime.c.obj
[ 59%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_printf/printf.c.obj
[ 60%] Building ASM object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_bit_ops/bit_ops_aeabi.S.obj
[ 61%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_bootrom/bootrom.c.obj
[ 63%] Building ASM object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_divider/divider.S.obj
[ 64%] Building ASM object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_double/double_aeabi.S.obj
[ 66%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_double/double_init_rom.c.obj
[ 67%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_double/double_math.c.obj
[ 69%] Building ASM object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_double/double_v1_rom_shim.S.obj
[ 70%] Building ASM object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_int64_ops/pico_int64_ops_aeabi.S.obj
[ 71%] Building ASM object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_float/float_aeabi.S.obj
[ 73%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_float/float_init_rom.c.obj
[ 74%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_float/float_math.c.obj
[ 76%] Building ASM object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_float/float_v1_rom_shim.S.obj
[ 77%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_malloc/pico_malloc.c.obj
[ 78%] Building ASM object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_mem_ops/mem_ops_aeabi.S.obj
[ 80%] Building ASM object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_standard_link/crt0.S.obj
[ 81%] Building CXX object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_standard_link/new_delete.cpp.obj
[ 83%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_standard_link/binary_info.c.obj
[ 84%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_stdio/stdio.c.obj
[ 85%] Building C object CMakeFiles/mri-swd-tests.dir/Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_multicore/multicore.c.obj
[ 87%] Linking CXX executable mri-swd-tests.elf
[ 87%] Built target mri-swd-tests
[ 88%] Sizing  mri-swd-tests
   text    data     bss     dec     hex filename
 104008       0    3604  107612   1a45c /Users/user/depots/mri-swd/tests/TestPass/build/mri-swd-tests.elf
[ 88%] Built target size_elf
[ 90%] Creating directories for 'PioasmBuild'
[ 91%] No download step for 'PioasmBuild'
[ 92%] No update step for 'PioasmBuild'
[ 94%] No patch step for 'PioasmBuild'
[ 95%] Performing configure step for 'PioasmBuild'
loading initial cache file /Users/user/depots/mri-swd/tests/TestPass/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp/PioasmBuild-cache-Debug.cmake
-- The CXX compiler identification is AppleClang 15.0.0.15000309
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: /Library/Developer/CommandLineTools/usr/bin/c++ - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Configuring done (0.1s)
-- Generating done (0.0s)
-- Build files have been written to: /Users/user/depots/mri-swd/tests/TestPass/build/pioasm
[ 97%] Performing build step for 'PioasmBuild'
[ 10%] Building CXX object CMakeFiles/pioasm.dir/main.cpp.o
[ 20%] Building CXX object CMakeFiles/pioasm.dir/pio_assembler.cpp.o
[ 30%] Building CXX object CMakeFiles/pioasm.dir/pio_disassembler.cpp.o
[ 40%] Building CXX object CMakeFiles/pioasm.dir/gen/lexer.cpp.o
[ 50%] Building CXX object CMakeFiles/pioasm.dir/gen/parser.cpp.o
[ 60%] Building CXX object CMakeFiles/pioasm.dir/c_sdk_output.cpp.o
[ 70%] Building CXX object CMakeFiles/pioasm.dir/python_output.cpp.o
[ 80%] Building CXX object CMakeFiles/pioasm.dir/hex_output.cpp.o
[ 90%] Building CXX object CMakeFiles/pioasm.dir/ada_output.cpp.o
[100%] Linking CXX executable pioasm
[100%] Built target pioasm
[ 98%] No install step for 'PioasmBuild'
[100%] Completed 'PioasmBuild'
[100%] Built target PioasmBuild
user@Mac TestPass % ./debug 10.0.0.201
GNU gdb (Arm GNU Toolchain 13.3.Rel1 (Build arm-13.24)) 14.2.90.20240526-git
Copyright (C) 2023 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "--host=aarch64-apple-darwin20.6.0 --target=arm-none-eabi".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<https://bugs.linaro.org/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from build/mri-swd-tests.elf...
Remote debugging using 10.0.0.201:2331
<signal handler called>
(gdb) load
Loading section .boot2, size 0x100 lma 0x10000000
Loading section .text, size 0xf5d8 lma 0x10000100
Loading section .rodata, size 0x9600 lma 0x1000f6d8
Loading section .binary_info, size 0x1c lma 0x10018cd8
Loading section .data, size 0x954 lma 0x10018cf4
Start address 0x100001e8, load size 104008
Transfer rate: 109 KB/sec, 9455 bytes/write.
(gdb) continue
Continuing.

1) Set registers to known values and crash.
2) Set registers to known values and stop at hardcoded bkpt.
3) Call breakOnMe() to increment g_global.
4) Log to stdout from both cores at the same time.
5) Log to stdout from main() and an ISR at the same time.
6) Run Semi-Hosting tests.
7) Trigger stacking exception.
Selection: ^C
Thread 1 received signal SIGINT, Interrupt.
mriNewlib_SemihostRead () at /Users/user/depots/mri-swd/shared/newlib_stubs.S:41
41          bkpt    MRI_NEWLIB_SEMIHOST_READ
(gdb)
```


## Verify Correct XML for Registers & Memory
**mri-swd** should expose XML dictating the register and memory layout of the device on which it is running.

There should be the expected r0-r12, sp, lr, pc, xpsr, msp, psp, primask, faultfmask, and control registers as seen below:
```console
(gdb) info all-reg
r0             0x0                 0
r1             0x20000f78          536874872
r2             0x400               1024
r3             0x0                 0
r4             0x200008b0          536873136
r5             0x20000d0c          536874252
r6             0x1000df81          268492673
r7             0x20041fb8          537141176
r8             0xffffffff          -1
r9             0x200008b0          536873136
r10            0x20041fb8          537141176
r11            0xffffffff          -1
r12            0x1000              4096
sp             0x20041f58          0x20041f58
lr             0x10000df7          268439031
pc             0x10000e58          0x10000e58 <mriNewlib_SemihostRead>
xpsr           0x41000000          1090519040
msp            0x20041f58          537141080
psp            0xfffffffc          -4
primask        0x0                 0
basepri        0x0                 0
faultmask      0x0                 0
control        0x0                 0
```

The various BootROM, FLASH, and RAM locations should also be exposed as seen below:
```console
(gdb) info mem
Using memory regions provided by the target.
Num Enb Low Addr   High Addr  Attrs
0   y   0x00000000 0x00004000 ro nocache
1   y   0x10000000 0x11000000 flash blocksize 0x1000 nocache
2   y   0x20000000 0x20042000 rw nocache
```


## Verify Initial Threads
Since the RP2040 has 2 cores, it should show 2 threads:
* Core0: In main(), waiting for input from the user over stdin.
* Core1: In the Boot ROM code at an address from the first 0x4000 bytes of memory since this core in initially halted by the second stage bootloader.
```
(gdb) info thread
  Id   Target Id         Frame
* 1    Thread 1 (Core0)  mriNewlib_SemihostRead () at /Users/user/depots/mri-swd/shared/newlib_stubs.S:41
  2    Thread 2 (Core1)  0x00000178 in ?? ()
(gdb) thread apply all bt

Thread 2 (Thread 2 (Core1)):
#0  0x00000178 in ?? ()
#1  0x0000014e in ?? ()
Backtrace stopped: previous frame identical to this frame (corrupt stack?)

Thread 1 (Thread 1 (Core0)):
#0  mriNewlib_SemihostRead () at /Users/user/depots/mri-swd/shared/newlib_stubs.S:41
#1  0x10000df6 in _read (file=<optimized out>, ptr=<optimized out>, len=<optimized out>) at /Users/user/depots/mri-swd/shared/newlib_retarget.c:157
#2  0x1000e334 in _read_r ()
#3  0x1000df8c in __sread ()
#4  0x100086a8 in __srefill_r ()
#5  0x10008c82 in _fgets_r ()
#6  0x10008d2e in fgets ()
#7  0x10000a76 in main () at /Users/user/depots/mri-swd/tests/TestPass/main.c:50
```


## Verify GDB Generated Memory Faults are Handled Properly
Try to read and write from an invalid address, like **0xFFFFFFF0**, to verify that **mri-swd** catches the fact that a memory access initiated by GDB has failed:
```console
(gdb) x/1wx 0xfffffff0
0xfffffff0:     Cannot access memory at address 0xfffffff0
(gdb) set var *(uint32_t*)0xfffffff0=0
Cannot access memory at address 0xfffffff0
```


## Test Generated Hard Fault
Reset the target (via the ```monitor reset``` and ```continue``` commands) and then select the following test to run: ```1) Set registers to known values and crash.```

```console
(gdb) monitor reset
Will reset on next continue.
(gdb) continue
Continuing.

1) Set registers to known values and crash.
2) Set registers to known values and stop at hardcoded bkpt.
3) Call breakOnMe() to increment g_global.
4) Log to stdout from both cores at the same time.
5) Log to stdout from main() and an ISR at the same time.
6) Run Semi-Hosting tests.
7) Trigger stacking exception.
Selection: 1
```

This test should generate a Hard Fault caused by a precise data access to address 0xFFFFFFF0 as seen below:
```
Thread 1 received signal SIGSEGV, Segmentation fault.
isr_hardfault () at /Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_standard_link/crt0.S:98
98      decl_isr_bkpt isr_hardfault
```

The registers should have known values at this time, most ascending as seen below:
```console
(gdb) info all-reg
r0             0xfffffff0          -16
r1             0x1                 1
r2             0x2                 2
r3             0x3                 3
r4             0x4                 4
r5             0x5                 5
r6             0x6                 6
r7             0x7                 7
r8             0x8                 8
r9             0x9                 9
r10            0xa                 10
r11            0xb                 11
r12            0xc                 12
sp             0x20041f70          0x20041f70
lr             0xfffffff9          -7
pc             0x100001c4          0x100001c4 <isr_hardfault>
xpsr           0x1000003           16777219
msp            0x20041f70          537141104
psp            0xfffffffc          -4
primask        0x0                 0
basepri        0x0                 0
faultmask      0x0                 0
control        0x0                 0
```

Dumping the callstack, with the ```bt``` GDB command, should show something like this:
```
(gdb) bt
#0  isr_hardfault () at /Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_standard_link/crt0.S:98
#1  <signal handler called>
#2  testContextWithCrash () at /Users/user/depots/mri-swd/tests/TestPass/tests.S:56
#3  0x100009ee in main () at /Users/user/depots/mri-swd/tests/TestPass/main.c:55
```

We can now reset the target again and let the test continue executing:
```
(gdb) monitor reset
Will reset on next continue.
(gdb) continue
Continuing.

1) Set registers to known values and crash.
2) Set registers to known values and stop at hardcoded bkpt.
3) Call breakOnMe() to increment g_global.
4) Log to stdout from both cores at the same time.
5) Log to stdout from main() and an ISR at the same time.
6) Run Semi-Hosting tests.
7) Trigger stacking exception.
Selection:
```


## Test Generated Breakpoint
Use GDB to select the ```2) Set registers to known values and stop at hardcoded bkpt.``` test.

This test should generate a debug trap caused by a hardcoded breakpoint in the test routine as seen below:
```
Selection: 2

Thread 1 received signal SIGTRAP, Trace/breakpoint trap.
testContextWithHardcodedBreakpoint () at /Users/user/depots/mri-swd/tests/TestPass/tests.S:109
109         bkpt    #0
```

The registers should have known values at this time, most ascending as seen in the GDB session below:
```
(gdb) info all-reg
r0             0x0                 0
r1             0x1                 1
r2             0x2                 2
r3             0x3                 3
r4             0x4                 4
r5             0x5                 5
r6             0x6                 6
r7             0x7                 7
r8             0x8                 8
r9             0x9                 9
r10            0xa                 10
r11            0xb                 11
r12            0xc                 12
sp             0x20041f90          0x20041f90
lr             0x10000a93          268438163
pc             0x10000ba6          0x10000ba6 <testContextWithHardcodedBreakpoint+49>
xpsr           0x1000000           16777216
msp            0x20041f90          537141136
psp            0xfffffffc          -4
primask        0x0                 0
basepri        0x0                 0
faultmask      0x0                 0
control        0x0                 0
(gdb) bt
#0  testContextWithHardcodedBreakpoint () at /Users/user/depots/mri-swd/tests/TestPass/tests.S:109
#1  0x10000a92 in main () at /Users/user/depots/mri-swd/tests/TestPass/main.c:58
```

Single step over the breakpoint exception as this should just cause **mri-swd** to soft step over the instruction and immediately trap on the next instruction from which we can continue execution:
```
(gdb) si
111         pop     {r0-r4}
(gdb) c
Continuing.

1) Set registers to known values and crash.
2) Set registers to known values and stop at hardcoded bkpt.
3) Call breakOnMe() to increment g_global.
4) Log to stdout from both cores at the same time.
5) Log to stdout from main() and an ISR at the same time.
6) Run Semi-Hosting tests.
7) Trigger stacking exception.
Selection:
```

Use GDB to select ```2) Set registers to known values and stop at hardcoded bkpt``` again and then just issue a ```continue``` to let the test program resume execution. **mri-swd** will first single step over the hardcoded breakpoint and then resume execution.
```
Selection: 2

Thread 1 received signal SIGTRAP, Trace/breakpoint trap.
testContextWithHardcodedBreakpoint () at /Users/user/depots/mri-swd/tests/TestPass/tests.S:109
109         bkpt    #0
(gdb) c
Continuing.

1) Set registers to known values and crash.
2) Set registers to known values and stop at hardcoded bkpt.
3) Call breakOnMe() to increment g_global.
4) Log to stdout from both cores at the same time.
5) Log to stdout from main() and an ISR at the same time.
6) Run Semi-Hosting tests.
7) Trigger stacking exception.
Selection:
```


## Test Handler and Thread Mode Breakpoints
**mri-swd** can set breakpoints in both handler and thread mode code.

* Press **CTRL+C** at the ```Selection: ``` prompt to halt execution so that we can set a breakpoint.
  * This will halt in ```mriNewlib_SemihostRead``` which is waiting for GDB to return stdin input.
* Issue ```break alarmCallback``` command to GDB. This will set a breakpoint on an ISR that we know fires on a regular basis.
* Issue ```continue``` command to resume execution, which is still waiting for user input from GDB.
* From GDB, type ```3``` following by **Return** to start the ```3) Call breakOnMe() to increment g_global.``` test.

```console
Selection: ^C
Thread 1 received signal SIGINT, Interrupt.
mriNewlib_SemihostRead () at /Users/user/depots/mri-swd/shared/newlib_stubs.S:41
41          bkpt    MRI_NEWLIB_SEMIHOST_READ
(gdb) break alarmCallback
Breakpoint 1 at 0x1000031c: file /Users/user/depots/mri-swd/tests/TestPass/main.c, line 89.
Note: automatically using hardware breakpoints for read-only addresses.
(gdb) c
Continuing.
3
Delaying 10 seconds...

Thread 1 hit Breakpoint 1, alarmCallback (id=1, user_data=0x0) at /Users/user/depots/mri-swd/tests/TestPass/main.c:89
89      {
```

Dump the callstack and look at the lower byte of the **XPSR** register to verify that we have halted in an ISR:
```console
(gdb) bt
#0  alarmCallback (id=1, user_data=0x0) at /Users/user/depots/mri-swd/tests/TestPass/main.c:89
#1  0x1000192e in alarm_pool_alarm_callback (alarm_num=3) at /Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/time.c:157
#2  0x10001dba in hardware_alarm_irq_handler () at /Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_timer/timer.c:144
#3  <signal handler called>
#4  sleep_until (t=...) at /Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/time.c:398
#5  0x10001cea in sleep_us (us=<optimized out>) at /Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/time.c:414
#6  0x10001d00 in sleep_ms (ms=ms@entry=10000) at /Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/time.c:430
#7  0x10000aba in main () at /Users/user/depots/mri-swd/tests/TestPass/main.c:64
(gdb) p/x $xpsr
$1 = 0x1000013
```

Now we can test breakpoints set in thread mode code by disabling the ```alarmCallback``` instance, setting one on ```breakOnMe()``` instead, and then letting the test to continue running. Verify that it stops at the ```breakOnMe()``` breakpoint.
```console
(gdb) delete
Delete all breakpoints? (y or n) y
(gdb) break breakOnMe
Breakpoint 2 at 0x10000308: file /Users/user/depots/mri-swd/tests/TestPass/main.c, line 100.
(gdb) c
Continuing.
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output

Thread 1 hit Breakpoint 2, breakOnMe () at /Users/user/depots/mri-swd/tests/TestPass/main.c:100
100         g_global++;
(gdb) bt
#0  breakOnMe () at /Users/user/depots/mri-swd/tests/TestPass/main.c:100
#1  0x10000abe in main () at /Users/user/depots/mri-swd/tests/TestPass/main.c:65
```


## Test Watchpoints
The following tests are similar to the previous except that they make sure that watchpoints work as well.

Start with a write watchpoint:
```console
(gdb) delete
Delete all breakpoints? (y or n) y
(gdb) watch g_global
Hardware watchpoint 3: g_global
(gdb) c
Continuing.

1) Set registers to known values and crash.
2) Set registers to known values and stop at hardcoded bkpt.
3) Call breakOnMe() to increment g_global.
4) Log to stdout from both cores at the same time.
5) Log to stdout from main() and an ISR at the same time.
6) Run Semi-Hosting tests.
7) Trigger stacking exception.
Selection: 3
Delaying 10 seconds...
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output

Thread 1 hit Hardware watchpoint 3: g_global

Old value = 1
New value = 2
main () at /Users/user/depots/mri-swd/tests/TestPass/main.c:67
67                      cancel_alarm(alarm);
```

Run the same test as above but for a read watchpoint:
```console
(gdb) delete
Delete all breakpoints? (y or n) y
(gdb) rwatch g_global
Hardware read watchpoint 4: g_global
(gdb) c
Continuing.

1) Set registers to known values and crash.
2) Set registers to known values and stop at hardcoded bkpt.
3) Call breakOnMe() to increment g_global.
4) Log to stdout from both cores at the same time.
5) Log to stdout from main() and an ISR at the same time.
6) Run Semi-Hosting tests.
7) Trigger stacking exception.
Selection: 3
Delaying 10 seconds...
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output

Thread 1 hit Hardware read watchpoint 4: g_global

Value = 3
breakOnMe () at /Users/user/depots/mri-swd/tests/TestPass/main.c:101
101         __dsb();
(gdb) delete
Delete all breakpoints? (y or n) y
(gdb) c
Continuing.

1) Set registers to known values and crash.
2) Set registers to known values and stop at hardcoded bkpt.
3) Call breakOnMe() to increment g_global.
4) Log to stdout from both cores at the same time.
5) Log to stdout from main() and an ISR at the same time.
6) Run Semi-Hosting tests.
7) Trigger stacking exception.
Selection:
```


## Test Single Stepping from Second Core
Normally when single stepping through code on one core with GDB, the code on the other core will continue to execute normally. This test verifies that happens when both cores are logging output.

Use the GDB console to select the ```4) Log to stdout from both cores at the same time.```.

Once the 2 cores have started logging text to the terminal, we can break in and perform some single stepping tests on the second core. In the terminal window, output from both cores should be seen while performing this single stepping. I use the ```fin``` command on the second core so that I can single step through ```thread2Func()``` instead of the Pico C SDK timer code.

```
Selection: 4
Set g_stop to true to end test...
Thread1 Output
Thread2 Output
Thread1 Output
^C
Thread 1 received signal SIGINT, Interrupt.
get_absolute_time () at /Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/include/pico/time.h:61
61      static inline absolute_time_t get_absolute_time(void) {
(gdb) info thread
  Id   Target Id         Frame
* 1    Thread 1 (Core0)  get_absolute_time () at /Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/include/pico/time.h:61
  2    Thread 2 (Core1)  0x10000384 in get_absolute_time () at /Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/include/pico/time.h:65
(gdb) thread apply all bt

Thread 2 (Thread 2 (Core1)):
#0  0x10000384 in get_absolute_time () at /Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/include/pico/time.h:65
#1  0x1000046e in sleep_ms_with_check (timeout_ms=timeout_ms@entry=2000) at /Users/user/depots/mri-swd/tests/TestPass/main.c:150
#2  0x10000498 in thread2Func () at /Users/user/depots/mri-swd/tests/TestPass/main.c:139
#3  0x10004dc4 in core1_wrapper (entry=0x1000048d <thread2Func>, stack_base=<optimized out>) at /Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_multicore/multicore.c:97
#4  0x00000172 in ?? ()
Backtrace stopped: previous frame identical to this frame (corrupt stack?)

Thread 1 (Thread 1 (Core0)):
#0  get_absolute_time () at /Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/include/pico/time.h:61
#1  0x1000046e in sleep_ms_with_check (timeout_ms=timeout_ms@entry=1000) at /Users/user/depots/mri-swd/tests/TestPass/main.c:150
#2  0x100004cc in thread1Func () at /Users/user/depots/mri-swd/tests/TestPass/main.c:130
#3  0x1000059a in runThreads () at /Users/user/depots/mri-swd/tests/TestPass/main.c:117
#4  0x10000aca in main () at /Users/user/depots/mri-swd/tests/TestPass/main.c:70
(gdb) thread 2
[Switching to thread 2 (Thread 2)]
#0  0x10000384 in get_absolute_time () at /Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/include/pico/time.h:65
65      }
(gdb) bt
#0  0x10000384 in get_absolute_time () at /Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/include/pico/time.h:65
#1  0x1000046e in sleep_ms_with_check (timeout_ms=timeout_ms@entry=2000) at /Users/user/depots/mri-swd/tests/TestPass/main.c:150
#2  0x10000498 in thread2Func () at /Users/user/depots/mri-swd/tests/TestPass/main.c:139
#3  0x10004dc4 in core1_wrapper (entry=0x1000048d <thread2Func>, stack_base=<optimized out>)
    at /Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_multicore/multicore.c:97
#4  0x00000172 in ?? ()
Backtrace stopped: previous frame identical to this frame (corrupt stack?)
(gdb) fin
Run till exit from #0  0x10000384 in get_absolute_time () at /Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/include/pico/time.h:65
Note: automatically using hardware breakpoints for read-only addresses.
0x1000046e in sleep_ms_with_check (timeout_ms=timeout_ms@entry=2000) at /Users/user/depots/mri-swd/tests/TestPass/main.c:150
150         } while (!g_stop && absolute_time_diff_us(get_absolute_time(), endTime) > 0);
Value returned is $1 = {
  _private_us_since_boot = 3035973
}
(gdb) fin
Run till exit from #0  0x1000046e in sleep_ms_with_check (timeout_ms=timeout_ms@entry=2000) at /Users/user/depots/mri-swd/tests/TestPass/main.c:150
Thread1 Output
thread2Func () at /Users/user/depots/mri-swd/tests/TestPass/main.c:140
140             printf("Thread2 Output\n");
(gdb) n
Thread2 Output
137         while (!g_stop)
(gdb)
Thread1 Output
139             sleep_ms_with_check(2000);
(gdb)
Thread1 Output
Thread1 Output
140             printf("Thread2 Output\n");
(gdb)
Thread1 Output
Thread2 Output
137         while (!g_stop)
(gdb)
Thread1 Output
139             sleep_ms_with_check(2000);
(gdb)
Thread1 Output
Thread1 Output
140             printf("Thread2 Output\n");
(gdb)
Thread1 Output
Thread2 Output
137         while (!g_stop)
(gdb) ni
0x100004aa      137         while (!g_stop)
(gdb)
Thread1 Output
0x100004ae      137         while (!g_stop)
(gdb)
Thread1 Output
0x10000492      139             sleep_ms_with_check(2000);
(gdb)
0x10000494      139             sleep_ms_with_check(2000);
(gdb)
Thread1 Output

Thread1 Output
140             printf("Thread2 Output\n");
(gdb)
Thread1 Output
0x1000049c      140             printf("Thread2 Output\n");
(gdb)
Thread1 Output
0x100004a0      140             printf("Thread2 Output\n");
(gdb)
Thread1 Output
0x100004a4      140             printf("Thread2 Output\n");
(gdb)
Thread2 Output
137         while (!g_stop)
(gdb)
```


## Test Locked Single Stepping from Second Core
This test can continue where we left off with the last one but switch GDB into the ```step``` locking mode where it freezes other cores when single stepping through the current core. When single stepping in this mode we should only see output from Thread2 in the terminal.

```
(gdb) set scheduler-locking step
(gdb) n
139             sleep_ms_with_check(2000);
(gdb)
140             printf("Thread2 Output\n");
(gdb)
Thread2 Output
137         while (!g_stop)
(gdb) ni
0x100004aa      137         while (!g_stop)
(gdb)
0x100004ac      137         while (!g_stop)
(gdb)
0x100004ae      137         while (!g_stop)
(gdb)
139             sleep_ms_with_check(2000);
(gdb)
0x10000492      139             sleep_ms_with_check(2000);
(gdb)
0x10000494      139             sleep_ms_with_check(2000);
(gdb)
140             printf("Thread2 Output\n");
(gdb)
0x1000049a      140             printf("Thread2 Output\n");
(gdb)
0x1000049c      140             printf("Thread2 Output\n");
(gdb)
0x1000049e      140             printf("Thread2 Output\n");
(gdb)
0x100004a0      140             printf("Thread2 Output\n");
(gdb)
0x100004a2      140             printf("Thread2 Output\n");
(gdb)
0x100004a4      140             printf("Thread2 Output\n");
(gdb)
Thread2 Output
137         while (!g_stop)
```

At the end of the test we should set the mode back to its original, ```off``` setting.

```
(gdb) set scheduler-locking off
```


# CTRL+C During Long Single Step
When stepping over the ```delay(2000)``` calls, try pressing **CTRL+C** to break in and dump the callstack.

```console
(gdb) n
139             sleep_ms_with_check(2000);
(gdb)
^C
Thread 2 received signal SIGINT, Interrupt.
time_us_64 () at /Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_timer/timer.c:53
53              if (hi == next_hi) break;
(gdb) bt
#0  time_us_64 () at /Users/user/depots/mri-swd/pico-sdk/src/rp2_common/hardware_timer/timer.c:53
#1  0x10000372 in get_absolute_time () at /Users/user/depots/mri-swd/pico-sdk/src/common/pico_time/include/pico/time.h:63
#2  0x1000046e in sleep_ms_with_check (timeout_ms=timeout_ms@entry=2000) at /Users/user/depots/mri-swd/tests/TestPass/main.c:150
#3  0x10000498 in thread2Func () at /Users/user/depots/mri-swd/tests/TestPass/main.c:139
#4  0x10004dc4 in core1_wrapper (entry=0x1000048d <thread2Func>, stack_base=<optimized out>)
    at /Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_multicore/multicore.c:97
#5  0x00000172 in ?? ()
Backtrace stopped: previous frame identical to this frame (corrupt stack?)
```

Set ```g_stop``` to true to stop the current test.
```console
(gdb) set var g_stop=true
(gdb) c
Continuing.
Multi-threaded test stopping...

1) Set registers to known values and crash.
2) Set registers to known values and stop at hardcoded bkpt.
3) Call breakOnMe() to increment g_global.
4) Log to stdout from both cores at the same time.
5) Log to stdout from main() and an ISR at the same time.
6) Run Semi-Hosting tests.
7) Trigger stacking exception.
Selection:
```


## Single Stepping Disabling of Interrupts
When single stepping through a thread with GDB, **mri-swd** will disable interrupts that would allow ISRs to run in the background and potentially add randomness to the code walk through. This test, ```5) Log to stdout from main() and an ISR at the same time.```, makes sure that an alarm ISR will not fire while single stepping through a simple NOP loop from the main code. While single stepping through the main loop, you should see no ```Alarm Callback Output``` output on stdout. Can use ```set var g_stop=true``` to stop the test and return to the menu. The missed alarms will all spew output once you issue the final ```continue``` instruction.

```console
Selection: 5
Set g_stop to true to end test...
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
^C
Thread 1 received signal SIGINT, Interrupt.
runThreadAndISR () at /Users/user/depots/mri-swd/tests/TestPass/main.c:168
168             pico_default_asm_volatile("nop");
(gdb) si
166         while (!g_stop)
(gdb)
0x10000536      166         while (!g_stop)
(gdb)
0x10000538      166         while (!g_stop)
(gdb)
0x1000053a      166         while (!g_stop)
(gdb)
168             pico_default_asm_volatile("nop");
(gdb)
166         while (!g_stop)
(gdb)
0x10000536      166         while (!g_stop)
(gdb)
0x10000538      166         while (!g_stop)
(gdb)
0x1000053a      166         while (!g_stop)
(gdb)
168             pico_default_asm_volatile("nop");
(gdb)
166         while (!g_stop)
(gdb)
0x10000536      166         while (!g_stop)
(gdb)
0x10000538      166         while (!g_stop)
(gdb)
0x1000053a      166         while (!g_stop)
(gdb)
168             pico_default_asm_volatile("nop");
(gdb)
166         while (!g_stop)
(gdb)
0x10000536      166         while (!g_stop)
(gdb)
0x10000538      166         while (!g_stop)
(gdb)
0x1000053a      166         while (!g_stop)
(gdb)
168             pico_default_asm_volatile("nop");
(gdb)
166         while (!g_stop)
(gdb)
0x10000536      166         while (!g_stop)
(gdb)
0x10000538      166         while (!g_stop)
(gdb)
0x1000053a      166         while (!g_stop)
(gdb)
168             pico_default_asm_volatile("nop");
(gdb)
166         while (!g_stop)
(gdb)
0x10000536      166         while (!g_stop)
(gdb) set var g_stop=true
(gdb) c
Continuing.
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
Alarm Callback Output
ISR test stopping...

1) Set registers to known values and crash.
2) Set registers to known values and stop at hardcoded bkpt.
3) Call breakOnMe() to increment g_global.
4) Log to stdout from both cores at the same time.
5) Log to stdout from main() and an ISR at the same time.
6) Run Semi-Hosting tests.
7) Trigger stacking exception.
Selection:
```


## Semi-Hosting
MRI is able to redirect mbed LocalFileSystem I/O to access the local filesystem on the GDB host.

Use GDB to select ```6) Run Semi-Hosting tests.``` and verify that all of its tests run successfully.

```console
Selection: 6
Semi-hosting Tests
Test 1: fopen() for write
Test 2: fprintf()
Test 3: fclose() on written file
Test 4: fopen() for read
Test 5: fscanf()
Contents of out.txt: Hello
Test 6: Determine size of file through fseek and ftell calls
Test 7: Determine size of file through fstat() call
Test 8: Determine size of file through stat() call
Test 9: fclose() on read file
Test 10: rename() the test output file
Test 11: remove() the renamed output file
Test 12: Send this output to stderr

Test completed

1) Set registers to known values and crash.
2) Set registers to known values and stop at hardcoded bkpt.
3) Call breakOnMe() to increment g_global.
4) Log to stdout from both cores at the same time.
5) Log to stdout from main() and an ISR at the same time.
6) Run Semi-Hosting tests.
7) Trigger stacking exception.
Selection:
```

## Stacking Exception HardFault
Use GDB to select ```7) Trigger stacking exception.```.

This should lead to a stacking exception (because msp/sp is set to an invalid address) as seen below:

```console
Selection: 7

Thread 1 received signal SIGSEGV, Segmentation fault.
isr_hardfault () at /Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_standard_link/crt0.S:98
98      decl_isr_bkpt isr_hardfault
(gdb) bt
#0  isr_hardfault () at /Users/user/depots/mri-swd/pico-sdk/src/rp2_common/pico_standard_link/crt0.S:98
warning: Could not fetch required XPSR content.  Further unwinding is impossible.
#1  <signal handler called>
(gdb) info all-reg
r0             0xfffffff0          -16
r1             0x1                 1
r2             0x100110d0          268505296
r3             0x10000ad8          268438232
r4             0x200008a8          536873128
r5             0x20041f01          537140993
r6             0x18000000          402653184
r7             0x0                 0
r8             0xffffffff          -1
r9             0xffffffff          -1
r10            0xffffffff          -1
r11            0xffffffff          -1
r12            0x20041fb8          537141176
sp             0xffffffd0          0xffffffd0
lr             0xfffffff9          -7
pc             0x100001c4          0x100001c4 <isr_hardfault>
xpsr           0x1000003           16777219
msp            0xffffffd0          -48
psp            0xfffffffc          -4
primask        0x0                 0
basepri        0x0                 0
faultmask      0x0                 0
control        0x0                 0
```
