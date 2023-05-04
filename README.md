# Non-Functional Work In Progress!
## MRI-SWD (Monitor for Remote Inspection - The Hardware SWD Version)
MRI-SWD is debug hardware which allows the [GNU debugger](https://www.sourceware.org/gdb/), GDB, to debug Cortex-M based microcontrollers via the SWD debug port.

## Important Notes
* __[5/4/2023]:__ This GitHub repository is just a placeholder for now. The code found within is just me getting basic SWD and gdb communications up and running. It only supports read/writing of memory and CPU registers at this time. I will plumb in more of the actual debug abilities over the upcoming weeks.
* The initial goal is to just be able to debug and program the  dual core [RP2040 microcontroller](https://www.raspberrypi.com/products/rp2040/). I will add other microcontrollers in the future. For example, I have an nRF52 based development board sitting just next to the [Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/) so it will get some testing as well.
* The initial target for this mri-swd firmware is to run on the low cost [Pico W](https://www.adafruit.com/product/5526) using its WiFi capabilities to wirelessly communicate with GDB. No intermediate program like OpenOCD will be required since the mri remote debug stub functionality will be running on the Pico W itself.

```gdb
GNU gdb (GNU Arm Embedded Toolchain 9-2020-q4-major) 8.3.1.20191211-git
Copyright (C) 2019 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "--host=x86_64-apple-darwin10 --target=arm-none-eabi".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from blink.elf...
Remote debugging using 10.0.0.24:4242
0x10000d5a in sleep_until ()
(gdb) bt
#0  0x10000d5a in sleep_until ()
#1  0x10000dca in sleep_ms ()
#2  0x10000328 in main ()
(gdb) c
Continuing.
^C
Program stopped.
0x10000d5a in sleep_until ()
(gdb) bt
#0  0x10000d5a in sleep_until ()
#1  0x10000dca in sleep_ms ()
#2  0x10000328 in main ()
(gdb) c
Continuing.
^C
Program stopped.
0x10000d5a in sleep_until ()
(gdb) info all-reg
r0             0x0                 0
r1             0x0                 0
r2             0x20000424          536871972
r3             0xd0000128          -805306072
r4             0xa0385c0           168003008
r5             0x0                 0
r6             0xa0385ba           168003002
r7             0x40054000          1074085888
r8             0x0                 0
r9             0xffffffff          -1
r10            0xffffffff          -1
r11            0xffffffff          -1
r12            0x0                 0
sp             0x20041fb8          0x20041fb8
lr             0x0                 0
pc             0x10000d5a          0x10000d5a <sleep_until+122>
xpsr           0x21000000          553648128
msp            0x20041fb8          537141176
psp            0xfffffffc          -4
primask        0x0                 0
basepri        0x0                 0
faultmask      0x0                 0
control        0x0                 0
(gdb) detach
Detaching from program: pico-examples/build/blink/blink.elf, Remote target
Ending remote debugging.
[Inferior 1 (Remote target) detached]
(gdb) q
```