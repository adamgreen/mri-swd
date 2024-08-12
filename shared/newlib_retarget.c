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
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include "mri.h"
#include "fileio.h"


/* Place errno in the global variable as that is where functions like _open_r() which call _open() expect it to be. */
#undef errno
extern int errno;

/* Flag set to flag that MRI hooks should be disabled when writing to stdout. */
volatile int g_mriDisableHooks = 0;


/* Define MRI_SEMIHOST_SUPPORT_FILE on compiler command line to support all file I/O APIs.
   By default just enough to redirect stdin/stdout/stderr to GDB will be supported.
*/
#ifdef MRI_SEMIHOST_SUPPORT_FILE

static uint32_t extractWordFromBigEndianWord(const void* pBigEndianValueToExtract);


/* File system related syscalls. */
int _open(const char *pFilename, int flags, int mode)
{
    int result = mriNewLib_SemihostOpen(pFilename, strlen(pFilename)+1, flags, mode);
    if (result < 0)
    {
        errno = mriNewlib_SemihostGetErrNo();
    }
    return result;
}

int rename(const char *pOldFilename, const char *pNewFilename)
{
    int result = mriNewLib_SemihostRename(pOldFilename, strlen(pOldFilename)+1, pNewFilename, strlen(pNewFilename)+1);
    if (result < 0)
    {
        errno = mriNewlib_SemihostGetErrNo();
    }
    return result;
}

int _unlink(const char *pFilename)
{
    int result = mriNewLib_SemihostUnlink(pFilename, strlen(pFilename)+1);
    if (result < 0)
    {
        errno = mriNewlib_SemihostGetErrNo();
    }
    return result;
}

int _fstat(int file, struct stat *pStat)
{
    GdbStats gdbStats;

    memset(&gdbStats, 0, sizeof(gdbStats));
    int result = mriNewlib_SemihostFStat(file, &gdbStats);
    if (result < 0)
    {
        errno = mriNewlib_SemihostGetErrNo();
    }

    pStat->st_dev = extractWordFromBigEndianWord(&gdbStats.device);
    pStat->st_ino = extractWordFromBigEndianWord(&gdbStats.inode);
    pStat->st_mode = extractWordFromBigEndianWord(&gdbStats.mode);
    pStat->st_nlink = extractWordFromBigEndianWord(&gdbStats.numberOfLinks);
    pStat->st_uid = extractWordFromBigEndianWord(&gdbStats.userId);
    pStat->st_gid = extractWordFromBigEndianWord(&gdbStats.groupId);
    pStat->st_rdev = extractWordFromBigEndianWord(&gdbStats.deviceType);
    pStat->st_size = extractWordFromBigEndianWord(&gdbStats.totalSizeLowerWord);
    pStat->st_atim.tv_sec = extractWordFromBigEndianWord(&gdbStats.lastAccessTime);
    pStat->st_mtim.tv_sec = extractWordFromBigEndianWord(&gdbStats.lastModifiedTime);
    pStat->st_ctim.tv_sec = extractWordFromBigEndianWord(&gdbStats.lastChangeTime);
    pStat->st_blksize = extractWordFromBigEndianWord(&gdbStats.blockSizeLowerWord);
    pStat->st_blocks = extractWordFromBigEndianWord(&gdbStats.blockCountLowerWord);

    return result;
}

int _stat(const char *pFilename, struct stat *pStat)
{
    GdbStats gdbStats;

    memset(&gdbStats, 0, sizeof(gdbStats));
    int result = mriNewLib_SemihostStat(pFilename, strlen(pFilename)+1, &gdbStats);
    if (result < 0)
    {
        errno = mriNewlib_SemihostGetErrNo();
    }

    pStat->st_dev = extractWordFromBigEndianWord(&gdbStats.device);
    pStat->st_ino = extractWordFromBigEndianWord(&gdbStats.inode);
    pStat->st_mode = extractWordFromBigEndianWord(&gdbStats.mode);
    pStat->st_nlink = extractWordFromBigEndianWord(&gdbStats.numberOfLinks);
    pStat->st_uid = extractWordFromBigEndianWord(&gdbStats.userId);
    pStat->st_gid = extractWordFromBigEndianWord(&gdbStats.groupId);
    pStat->st_rdev = extractWordFromBigEndianWord(&gdbStats.deviceType);
    pStat->st_size = extractWordFromBigEndianWord(&gdbStats.totalSizeLowerWord);
    pStat->st_atim.tv_sec = extractWordFromBigEndianWord(&gdbStats.lastAccessTime);
    pStat->st_mtim.tv_sec = extractWordFromBigEndianWord(&gdbStats.lastModifiedTime);
    pStat->st_ctim.tv_sec = extractWordFromBigEndianWord(&gdbStats.lastChangeTime);
    pStat->st_blksize = extractWordFromBigEndianWord(&gdbStats.blockSizeLowerWord);
    pStat->st_blocks = extractWordFromBigEndianWord(&gdbStats.blockCountLowerWord);
    return result;
}

static uint32_t extractWordFromBigEndianWord(const void* pBigEndianValueToExtract)
{
    const unsigned char* pBigEndianValue = (const unsigned char*)pBigEndianValueToExtract;
    return pBigEndianValue[3]        | (pBigEndianValue[2] << 8) |
          (pBigEndianValue[1] << 16) | (pBigEndianValue[0] << 24);
}


/* File handle related syscalls. */
int _lseek(int file, int ptr, int dir)
{
    int result =  mriNewlib_SemihostLSeek(file, ptr, dir);
    if (result < 0)
    {
        errno = mriNewlib_SemihostGetErrNo();
    }
    return result;
}

int _close(int file)
{
    int result =  mriNewlib_SemihostClose(file);
    if (result < 0)
    {
        errno = mriNewlib_SemihostGetErrNo();
    }
    return result;
}
#endif /* MRI_SEMIHOST_SUPPORT_FILE */

int _read(int file, char *ptr, int len)
{
    int result =  mriNewlib_SemihostRead(file, ptr, len);
    if (result < 0)
    {
        errno = mriNewlib_SemihostGetErrNo();
    }
    return result;
}

int _write(int file, char *ptr, int len)
{
    const int STDOUT_FILE_NO = 1;

    if (file == STDOUT_FILE_NO)
    {
        g_mriDisableHooks = 1;
    }

    int result = mriNewlib_SemihostWrite(file, ptr, len);
    if (result < 0)
    {
        errno = mriNewlib_SemihostGetErrNo();
    }

    if (file == STDOUT_FILE_NO)
    {
        g_mriDisableHooks = 0;
    }
    return result;
}

int _isatty(int file)
{
    if (file >= 0 && file <= 2)
        return 1;
    return 0;
}
