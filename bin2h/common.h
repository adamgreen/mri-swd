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
/* Commonly used constants and routines. */
#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdio.h>
#include "try_catch.h"


#define FALSE 0
#define TRUE  1

#define ARRAY_SIZE(X) (sizeof(X)/sizeof(X[0]))

__throws long GetFileSize(FILE* pFile);


#endif /* _COMMON_H_ */
