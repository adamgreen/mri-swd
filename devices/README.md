# MRI-SWD Device Support
This folder contains the code which adds support for specific devices or family of devices.
This includes the ability to program the FLASH.

## Currently Supported Devices
* [Raspberry Pi RP2040](https://www.raspberrypi.com/products/rp2040/)
* [Nordic Semiconductor nRF52xxx](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fstruct_nrf52%2Fstruct%2Fnrf52.html)

## Useful Notes for Adding New Device Support
* When adding support for a new device, .h and .cpp source files will be added with a name reflecting that device.
  * The **device_name.h** header will declare the existence of an external `DeviceFunctionTable` for this particular device. You can see
    examples of this in [rp2040.h](rp2040.h) and [nrf52.h](nrf52.h).
  * The **device_name.cpp** source file will implement each of the custom functions to be placed in the `DeviceFunctionTable`.
  * The `DeviceFunctionTable` definition in [devices.h](devices.h) gives more information about what each of these functions need to do. The [rp2040.cpp](rp2040.cpp) and [nrf52.cpp](nrf52.cpp) source files give working example implementations.
* The newly defined `DeviceFunctionTable` global for this device then needs to be added to the `g_supportedDevices` array in [devices.cpp](devices.cpp).
* The last thing that needs to be done is adding the new **devices/device_name.cpp** module to [CMakeLists.txt](../CMakeLists.txt).
