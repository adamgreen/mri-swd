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
/* Very rough exception handling like macros for C. */
#ifndef _TRY_CATCH_H_
#define _TRY_CATCH_H_

#include <assert.h>
#include <setjmp.h>
#include <stdlib.h>


#define noException                         0
#define bufferOverrunException              1
#define invalidHexDigitException            2
#define invalidValueException               3
#define invalidArgumentException            4
#define timeoutException                    5
#define invalidIndexException               6
#define notFoundException                   7
#define exceededHardwareResourcesException  8
#define invalidDecDigitException            9
#define memFaultException                   10
#define serialException                     11
#define badResponseException                12
#define fileException                       13
#define outOfMemoryException                14
#define socketException                     15


typedef struct ExceptionHandler
{
    struct ExceptionHandler* pPrevious;
    jmp_buf*                 pJumpBuffer;
} ExceptionHandler;


extern ExceptionHandler* g_pExceptionHandlers;
extern int               g_exceptionCode;


/* On Linux, it is possible that __try and __catch are already defined. */
#undef __try
#undef __catch

#define __throws

#define __try \
        do \
        { \
            jmp_buf jumpBuffer; \
            struct ExceptionHandler exceptionHandler; \
            exceptionHandler.pPrevious = g_pExceptionHandlers; \
            exceptionHandler.pJumpBuffer = &jumpBuffer; \
            g_pExceptionHandlers = &exceptionHandler; \
            clearExceptionCode(); \
            \
            if (0 == setjmp(jumpBuffer)) \
            { \

#define __catch \
            } \
            g_pExceptionHandlers = exceptionHandler.pPrevious; \
        } while(0); \
        if (g_exceptionCode)

#define __throw(EXCEPTION) \
        { \
            setExceptionCode(EXCEPTION); \
            if (!g_pExceptionHandlers) \
            { \
                assert ( g_pExceptionHandlers ); \
                exit(-1); \
            } \
            else \
            { \
                longjmp(*g_pExceptionHandlers->pJumpBuffer, 1); \
                exit(-1); \
            } \
        }

#define __rethrow __throw(getExceptionCode())

#define __nothrow { clearExceptionCode(); return; }

#define __nothrow_and_return(RETURN) return (clearExceptionCode(), (RETURN))

#define __try_and_catch(X) __try(X); __catch { }


static inline int getExceptionCode(void)
{
    return g_exceptionCode;
}

static inline void setExceptionCode(int exceptionCode)
{
    g_exceptionCode = exceptionCode > g_exceptionCode ? exceptionCode : g_exceptionCode;
}

static inline void clearExceptionCode(void)
{
    g_exceptionCode = noException;
}

#endif /* _TRY_CATCH_H_ */
