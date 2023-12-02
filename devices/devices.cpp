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
// List of devices or family of devices supported by mri-swd.
// Also contains a few utility functions to aid in implementing target device drivers.
#include "devices.h"
#include "rp2040.h"
#include "nrf52.h"


// The list of all supported devices can be found in this table.
DeviceFunctionTable* g_supportedDevices[] = {
    &g_rp2040DeviceSupport,
    &g_nrf52DeviceSupport
};

// The number of entries in the g_supportedDevices array.
size_t g_supportedDevicesLength = count_of(g_supportedDevices);




// Return false to indicate that there is no FPU available.
bool deviceHasNoFpu(DeviceObject* pvObject, CpuCore* pCore)
{
    return false;
}

// Return true to indicate that there is a FPU available.
bool deviceHasFpu(DeviceObject* pvObject, CpuCore* pCore)
{
    return true;
}


// Default memory layout of Cortex-M microcontrollers to use if we don't have device specific memory layout information.
static const DeviceMemoryRegion g_defaultMemoryRegions[] =
{
    { .address = 0x00000000, .length = 0x20000000, .blockSize = 0, .type = DEVICE_MEMORY_ROM },
    { .address = 0x20000000, .length = 0x20000000, .blockSize = 0, .type = DEVICE_MEMORY_RAM },
    { .address = 0x60000000, .length = 0x40000000, .blockSize = 0, .type = DEVICE_MEMORY_RAM },
};
static const DeviceMemoryLayout g_defaultMemoryLayout =
{
    .pRegions = g_defaultMemoryRegions,
    .regionCount = count_of(g_defaultMemoryRegions)
};

const DeviceMemoryLayout* deviceDefaultMemoryLayout(DeviceObject* pvObject, CpuCore* pCore)
{
    return &g_defaultMemoryLayout;
}


// Just return with pTargetCount set to 0 to indicate that no additional targets exist for this device.
bool deviceNoAdditionalTargets(DeviceObject* pvObject, CpuCore* pCore, CpuCore* pCoreArray, size_t coreArrayLength, size_t* pCoreCount)
{
    *pCoreCount = 0;
    return true;
}
