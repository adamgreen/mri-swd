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
// Interface used to add support for specific devices or family of devices.
// This includes the ability to program the FLASH.
//
// * When adding support for a new device, .h and .cpp source files will be added with a name reflecting that device.
//   * The header will declare the existence of an external DeviceFunctionTable for this particular device. You can see
//     examples of this in rp2040.h and nrf52.h
//   * The device_name.cpp source file will implement each of the custom functions to be placed in this
//     DeviceFunctionTable. The DeviceFunctionTable definition below in this header file gives more information about
//     what each of these functions need to do. The rp2040.cpp and nrf52.cpp source files give working example
//     implementations.
// * The newly defined DeviceFunctionTable global for this device then needs to be added to the g_supportedDevices array
//   in devices.cpp
// * The last thing that needs to be done is adding the new devices/device_name.cpp module to the CMakeLists.txt
#ifndef DEVICES_H_
#define DEVICES_H_

#include "swd.h"

// MRI C headers.
extern "C"
{
    #include "core/buffer.h"
}


// Types of memory that can be defined in the device's memory layout.
enum DeviceMemoryType
{
    DEVICE_MEMORY_RAM,
    DEVICE_MEMORY_ROM,
    DEVICE_MEMORY_FLASH
};

// Description of a region of device memory (RAM, ROM, FLASH): Starting address, size, etc.
struct DeviceMemoryRegion
{
    // Starting address of this region.
    uint32_t address;
    // Number of bytes in this region.
    uint32_t length;
    // Erase FLASH in blocks of this size.
    uint32_t blockSize;
    DeviceMemoryType type;
};

// Memory layout of the device's various memory regions.
// Returned from a device's DeviceFunctionTable::getMemoryLayout() function.
struct DeviceMemoryLayout
{
    // Pointer to an array of memory regions.
    const DeviceMemoryRegion* pRegions;
    // The number of regions in the pRegions array.
    uint32_t regionCount;
};

// The device can allocate an object to contain any necessary context between calls to its DeviceFunctionTable
// functions. It will be allocated in the DeviceFunctionTable::detect() method, freed in the DeviceFunctionTable::free()
// method, and passed as context to all others.
typedef void DeviceObject;

// Each specific device or device family supported by mri-swd implements this interface and puts its implementation
// into g_supportedDevices (in device.cpp).
struct DeviceFunctionTable
{
    // mri-swd will call this function on each of the devices listed in g_supportedDevices until a non-NULL response
    // is encountered, indicating that the device attached to the debugger is supported by that element.
    //
    // pSWD is a pointer to the SWD object used for interfacing to the device.
    //
    // The returned pointer can point to anything that the device specific code wants to access on subsequent calls to
    // other routines in this function table. It can even be something malloc()ed. Calling the free() function from this
    // table can then be used to free it. Should return NULL if the connected target isn't recognized as this type of
    // device.
    DeviceObject* (*detect)(SWD* pSWD);

    // Can be used by the device specific code to free the object returned from detect().
    //
    // pvObject is a pointer to the object allocated by detect().
    // pSWD is a pointer to the SWD object used for interfacing to the device.
    void (*free)(DeviceObject* pvObject, SWD* pSWD);


    // Gets friendly name for this device.
    //
    // pvObject is a pointer to the object allocated by detect().
    // pSWD is a pointer to the SWD object used for interfacing to the device.
    //
    // Returns a \0 terminated string indicating this device's name.
    const char* (*getName)(DeviceObject* pvObject, SWD* pSWD);

    // Return true if the device(s) contain a FPU.
    // An implementation can place deviceHasNoFpu() or deviceHasFpu() in this entry of its function table.
    //
    // pvObject is a pointer to the object allocated by detect().
    // pSWD is a pointer to the SWD object used for interfacing to the device.
    bool (*hasFpu)(DeviceObject* pvObject, SWD* pSWD);

    // The SWD interface will be set to this frequency from the possibly lower rate used during SWD::init().
    //
    // pvObject is a pointer to the object allocated by detect().
    // pSWD is a pointer to the SWD object used for interfacing to the device.
    //
    // Returns the maximum SWCLK frequency supported by this device.
    uint32_t (*getMaximumSWDClockFrequency)(DeviceObject* pvObject, SWD* pSWD);

    // Called at the beginning of the FLASH programming process. The device specific code can perform whatever
    // initialization it needs here before the flashErase() and flashProgram() routines are called.
    //
    // pvObject is a pointer to the object allocated by detect().
    // pSWD is a pointer to the SWD object used for interfacing to the device.
    //
    // Returns true if successful and false otherwise.
    bool (*flashBegin)(DeviceObject* pvObject, SWD* pSWD);

    // GDB would like the FLASH at the specified address range to be erased in preparation for programming.
    //
    // pvObject is a pointer to the object allocated by detect().
    // pSWD is a pointer to the SWD object used for interfacing to the device.
    // addressStart is the address at the beginning of the range to be erased. (inclusive)
    // length is the number of bytes starting at address to be erased.
    //
    // Returns true if successful and false otherwise.
    bool (*flashErase)(DeviceObject* pvObject, SWD* pSWD, uint32_t addressStart, uint32_t length);

    // GDB would like the FLASH at the specified address to be programmed with the supplied data.
    //
    // pvObject is a pointer to the object allocated by detect().
    // pSWD is a pointer to the SWD object used for interfacing to the device.
    // addressStart is the address at the beginning of the range to be programmed.
    // pBuffer is the address of the data to be programmed into the specified FLASH location.
    // bufferSize is the number of bytes in pBuffer to be programmed into FLASH at the specified location.
    //
    // Returns true if successful and false otherwise.
    bool (*flashProgram)(DeviceObject* pvObject, SWD* pSWD, uint32_t addressStart, const void* pBuffer, size_t bufferSize);

    // Called at the end of the FLASH programming process. The device specific code can perform whatever
    // cleanup it needs here after all the flashErase() and flashProgram() call have been made.
    //
    // pvObject is a pointer to the object allocated by detect().
    // pSWD is a pointer to the SWD object used for interfacing to the device.
    //
    // Returns true if successful and false otherwise.
    bool (*flashEnd)(DeviceObject* pvObject, SWD* pSWD);

    // Returns the memory layout of the ROM, FLASH, and RAM regions of the device(s). This will be used to provide the
    // memory layout XML to GDB.
    //
    // If it isn't possible to figure out the exact memory layout for a device then set this function pointer to the
    // deviceDefaultMemoryLayout() function declared below.
    //
    // pvObject is a pointer to the object allocated by detect().
    // pSWD is a pointer to the SWD object used for interfacing to the device.
    //
    // Returns a pointer to the memory layout description for this device. Can't be NULL.
    const DeviceMemoryLayout* (*getMemoryLayout)(DeviceObject* pvObject, SWD* pSWD);

#ifdef UNDONE
    // Resets the device from GDB "monitor reset" request. The arguments to the monitor reset are passed into the
    // function as well so that different types of reset can be performed based on user request.
    bool reset(pv, pSWD, pMonitorCommand)
#endif // UNDONE
};


// Function that can be placed in hasFpu() member of DeviceFunctionTable entries where CPU is known to have no FPU.
bool deviceHasNoFpu(DeviceObject* pvObject, SWD* pSWD);

// Function that can be placed in hasFpu() member of DeviceFunctionTable entries where CPU is known to have a FPU.
bool deviceHasFpu(DeviceObject* pvObject, SWD* pSWD);

// Function that can be placed in getMemoryLayout() member of DeviceFunctionTable entries when it isn't possible to
// figure out the specific memory layout for a particular device. This uses the default memory map that Cortex-M CPUs
// use.
const DeviceMemoryLayout* deviceDefaultMemoryLayout(DeviceObject* pvObject, SWD* pSWD);


// The list of all supported devices can be found in this table, defined in devices/devices.cpp
extern DeviceFunctionTable* g_supportedDevices[];
extern size_t               g_supportedDevicesLength;


#endif // DEVICES_H_
