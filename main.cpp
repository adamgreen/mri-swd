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
// MRI SWD - Main Module
#define MAIN_MODULE "main.cpp"
#include "logging.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <pico/rand.h>
#include <pico/stdlib.h>
#include "mri_platform.h"
#include "version.h"


int main()
{
    stdio_init_all();

    // UNDONE: Giving time for serial terminal to reconnect.
    sleep_ms(2000);

    logInfo("|mri-swd| Monitor for Remote Inspection - SWD Edition (" MRI_SWD_VERSION_STRING ")");
    logInfo(MRI_SWD_BRANCH);
    logInfo("");

    mainDebuggerLoop();

    // UNDONE: Reset the microcontroller if we get all the way out here.
    logError("Have hit a critical internal error. Halting!");
    fflush(stdout);
    while (true)
    {
    }

    return 0;
}
