# Non-Functional Work In Progress!
## MRI-SWD (Monitor for Remote Inspection - The Hardware SWD Version)
MRI-SWD is debug hardware which allows the [GNU debugger](https://www.sourceware.org/gdb/), GDB, to debug Cortex-M based microcontrollers via the SWD debug port.

## Important Notes
* __[5/2/2023]:__ This GitHub repository is just a placeholder for now. The code found within is just me getting basic SWD and gdb communications up and running. I will plumb in the actual debug abilities over the upcoming weeks.
* The initial goal is to just be able to debug and program the  dual core [RP2040 microcontroller](https://www.raspberrypi.com/products/rp2040/). I will add other microcontrollers in the future. For example, I have an nRF52 based development board sitting just next to the [Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/) so it will get some testing as well.
* The initial target for this mri-swd firmware is to run on the low cost [Pico W](https://www.adafruit.com/product/5526) using its WiFi capabilities to wirelessly communicate with GDB. No intermediate program like OpenOCD will be required since the mri remote debug stub functionality will be running on the Pico W itself.

```gdb
Reading symbols from build/QuadratureDecoder.elf...
Remote debugging using 10.0.0.24:4242
0x00000000 in ?? ()
(gdb) x/2wa 0x10000000
0x10000000 <__flash_binary_start>:	0x4b32b500	0x60582021
(gdb) x/1wa 0x2001FC00
0x2001fc00:	0xcbbf0e8b
(gdb) set var *(uint32_t*)0x2001FC00 =0xbaadfeed
(gdb) x/1wa 0x2001FC00
0x2001fc00:	0xbaadfeed
```