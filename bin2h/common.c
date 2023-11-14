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
#include "common.h"


long GetFileSize(FILE* pFile)
{
    int    result = -1;
    long   fileSize = 0;

    result = fseek(pFile, 0, SEEK_END);
    if (result == -1)
        __throw(fileException);

    fileSize = ftell(pFile);
    if (fileSize < 0)
        __throw(fileException);

    result = fseek(pFile, 0, SEEK_SET);
    if (result == -1)
        __throw(fileException);

    return fileSize;
}
