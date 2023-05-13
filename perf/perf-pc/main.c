/*  Copyright (C) 2019  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <mach/mach_time.h>
#include <netdb.h>
#include <unistd.h>


mach_timebase_info_data_t   g_machTimebaseInfo;


static int LookupAddress(const char* pServerName, struct sockaddr_in* pServerAddress, uint32_t portNumber);
static unsigned int GetTimeInMicroseconds(void);


int main(int argc, const char** argv)
{
    int                 Return = -1;
    int                 Socket = -1;
    int                 ConnectResult = -1;

    if (argc < 4)
    {
        printf("Usage: perf-pc ip_address port_number test_size\n");
        goto Error;
    }
    const char* pServerName = argv[1];
    uint32_t portNumber = strtoul(argv[2], NULL, 0);
    uint32_t testSize = strtoul(argv[3], NULL, 0);

    struct sockaddr_in socketAddress;
    if (LookupAddress(pServerName, &socketAddress, portNumber) != 0)
    {
        printf("error: Failed to lookup IP address for %s.\n", pServerName);
        goto Error;
    }
    mach_timebase_info(&g_machTimebaseInfo);

    Socket = socket(PF_INET, SOCK_STREAM, 0);
    if (-1 == Socket)
    {
        printf("error: Failed to create socket.\n");
        goto Error;
    }

    printf("Connecting...\n");
    ConnectResult = connect(Socket, (struct sockaddr*)&socketAddress, sizeof(socketAddress));
    if (ConnectResult)
    {
        printf("error: Failed to connect to %s:%u.\n", pServerName, portNumber);
        goto Error;
    }

    uint8_t* pBuffer = malloc(testSize);
    memset(pBuffer, 0, testSize);

    printf("Starting test...\n");
    const int NoSendFlags = 0;
    const int NoRecvFlags = 0;
    ssize_t sendResult = send(Socket, &testSize, sizeof(testSize), NoSendFlags);
    if (sendResult != (ssize_t)sizeof(testSize))
    {
        printf("error: Failed to send test size to server.\n");
        goto Error;
    }

    uint32_t StartTime = GetTimeInMicroseconds();
    uint32_t bytesLeft = testSize;
    uint8_t* pCurr = pBuffer;
    while (bytesLeft > 0)
    {
        ssize_t RecvResult = recv(Socket,
                                pCurr,
                                bytesLeft,
                                NoRecvFlags);
        if (RecvResult < 0)
        {
            printf("error: Failed to receive test response.\n");
            goto Error;
        }
        bytesLeft -= RecvResult;
        pCurr += RecvResult;
    }

    // UNDONE: Verify the data is all 0xBAADFEED.
    printf("Verifying contents of buffer...\n");
    uint32_t* pTest = (uint32_t*)pBuffer;
    size_t i = 0;
    for (i = 0 ; i < testSize/sizeof(uint32_t) ; i++, pTest++)
    {
        if (*pTest != 0xBAADFEED)
        {
            printf("Data mismatch at offset %lu. Expected: 0x%08X Actual: 0x%08X\n", i*sizeof(uint32_t), 0xBAADFEED, *pTest);
            break;
        }
    }

    uint32_t ElapsedTime = GetTimeInMicroseconds() - StartTime;
    printf("Received %u bytes in %u microseconds.\n", testSize, ElapsedTime);
    printf("%.1f bytes/sec\n", testSize * 1000000.0f / ElapsedTime);

Error:
    if (Socket != -1)
    {
        close(Socket);
    }

    return Return;
}

static int LookupAddress(const char* pServerName, struct sockaddr_in* pServerAddress, uint32_t portNumber)
{
    int                 Return = 1;
    struct hostent*     pHostEntry = NULL;

    pHostEntry = gethostbyname(pServerName);
    if (!pHostEntry)
    {
        printf("error: Failed to lookup address for \"%s\".\n", pServerName);
        goto Error;
    }

    memset(pServerAddress, 0, sizeof(*pServerAddress));
    pServerAddress->sin_family = AF_INET;
    memcpy(&pServerAddress->sin_addr.s_addr, pHostEntry->h_addr_list[0], sizeof(pServerAddress->sin_addr.s_addr));
    pServerAddress->sin_port = htons(portNumber);

    Return = 0;
Error:
    return Return;
}

static unsigned int GetTimeInMicroseconds(void)
{
    return (unsigned int)((mach_absolute_time() * g_machTimebaseInfo.numer) /
                          (1000 * g_machTimebaseInfo.denom));
}
