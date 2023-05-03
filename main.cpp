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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <pico/cyw43_arch.h>
#include <pico/rand.h>
#include <pico/stdlib.h>
#include "mri_platform.h"


// Function Prototypes.
static bool connectToWiFiRouter();


int main()
{
    stdio_init_all();

    // UNDONE: Giving time for serial terminal to reconnect.
    sleep_ms(2000);

    printf("Starting up...\n");

    // UNDONE: What needs to be done if router goes away? Can we reconnect.
    printf("Connecting to Wi-Fi router...\n");
    bool result = connectToWiFiRouter();
    if (!result)
    {
        printf("Failed to connect to Wi-Fi router.\n");
        __breakpoint();
        return 1;
    }
    printf("Connected.\n");

    mainDebuggerLoop();

    cyw43_arch_deinit();
    return 0;
}


static bool connectToWiFiRouter()
{
    if (cyw43_arch_init())
    {
        printf("Wi-Fi chip failed to initialize.\n");
        __breakpoint();
        return false;
    }
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("Failed to connect to Wi-Fi.\n");
        __breakpoint();
        return false;
    }

    return true;
}
