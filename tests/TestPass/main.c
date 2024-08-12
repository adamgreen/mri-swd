/* This sample is used to put the mri-swd debug hardware through its paces before a release. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <pico/multicore.h>
#include <hardware/sync.h>

// Assembly language functions found in tests.S
void testContextWithCrash(void);
void testContextWithHardcodedBreakpoint(void);
void testStackingHandlerException(void);

// Forward function declarations.
int64_t alarmCallback(alarm_id_t id, void *user_data);
static void __attribute__ ((noinline)) breakOnMe(void);
static void runThreads(void);
static void thread1Func(void);
static void thread2Func(void);
static void sleep_ms_with_check(uint32_t timeout_ms);
static void runThreadAndISR(void);
static void runFileTests();

// Can use fprintf(stdout, ) to force output to go to GDB even when pico_stdlib is linked in.
#define printf(...) fprintf(stdout, __VA_ARGS__)

// Selection variable to be set from GDB.
static volatile int g_selection = 0;

int main(void)
{
    // Disable buffering for writes to stdout as performed by printf().
    setvbuf(stdout, NULL, _IONBF, 0);

    alarm_pool_init_default();

    while (1)
    {
        printf("\n");
        printf("1) Set registers to known values and crash.\n");
        printf("2) Set registers to known values and stop at hardcoded bkpt.\n");
        printf("3) Call breakOnMe() to increment g_global.\n");
        printf("4) Log to stdout from both cores at the same time.\n");
        printf("5) Log to stdout from main() and an ISR at the same time.\n");
        printf("6) Run Semi-Hosting tests.\n");
        printf("7) Trigger stacking exception.\n");

        printf("Selection: ");
        char buffer[64];
        fgets(buffer, sizeof(buffer), stdin);
        g_selection = atoi(buffer);

        switch (g_selection) {
            case 1:
                testContextWithCrash();
                break;
            case 2:
                testContextWithHardcodedBreakpoint();
                break;
            case 3:
                alarm_id_t alarm = add_alarm_in_ms(500, alarmCallback, NULL, true);

                printf("Delaying 10 seconds...\n");
                sleep_ms(10000);
                breakOnMe();

                cancel_alarm(alarm);
                break;
            case 4:
               runThreads();
                break;
            case 5:
                runThreadAndISR();
                break;
            case 6:
                runFileTests();
                break;
            case 7:
                testStackingHandlerException();
                break;
            default:
                printf("Invalid selection\n");
                break;
        }
    }
}

int64_t alarmCallback(alarm_id_t id, void *user_data)
{
    printf("Alarm Callback Output\n");

    // Keep this alarm repeating every 500ms by returning -500000.
    return -500000;
}

static volatile uint32_t g_global;

static void __attribute__ ((noinline)) breakOnMe(void)
{
    g_global++;
    __dsb();
}

static volatile int g_stop = 0;
static void runThreads(void)
{
    g_stop = false;
    printf("Set g_stop to true to end test...\n");

    // Allow the timer to run when the cores are frozen so that you can single step through code on one core while the
    // other core is still halted.
    uint32_t orig_dbgpause = timer_hw->dbgpause;
    timer_hw->dbgpause = 0;

    // Run the two test functions, one on each core.
    multicore_launch_core1(thread2Func);
    thread1Func();
    multicore_reset_core1();

    // Restore the timer's Debug pause settings.
    timer_hw->dbgpause = orig_dbgpause;

    printf("Multi-threaded test stopping...\n");
}

static void thread1Func(void)
{
    while (!g_stop)
    {
        sleep_ms_with_check(1000);
        printf("Thread1 Output\n");
    }
}

static void thread2Func(void)
{
    while (!g_stop)
    {
        sleep_ms_with_check(2000);
        printf("Thread2 Output\n");
    }
}

static void sleep_ms_with_check(uint32_t timeout_ms)
{
    absolute_time_t endTime = make_timeout_time_ms(timeout_ms);

    do
    {
    } while (!g_stop && absolute_time_diff_us(get_absolute_time(), endTime) > 0);
}

static void runThreadAndISR(void)
{
    g_stop = false;
    printf("Set g_stop to true to end test...\n");

    // Allow the timer to run when Core0 is halted.
    uint32_t orig_dbgpause = timer_hw->dbgpause;
    timer_hw->dbgpause = 0;

    // Run the two test functions, one on each core.
    alarm_id_t alarm = add_alarm_in_ms(500, alarmCallback, NULL, true);

    // Simple code to step through while alarm ISR is firing in background.
    while (!g_stop)
    {
        pico_default_asm_volatile("nop");
    }

    cancel_alarm(alarm);

    // Restore the timer's Debug pause settings.
    timer_hw->dbgpause = orig_dbgpause;

    printf("ISR test stopping...\n");
}


static void runFileTests()
{
    int  Result = -1;
    long Offset = -1;
    char Buffer[32];

    printf("Semi-hosting Tests\n");

    // Open "out.txt" on the GDB host file system for writing
    printf("Test 1: fopen() for write\n");
    FILE *fp = fopen("out.txt", "w");
    if (NULL == fp)
    {
        printf("%s(%d) fopen() failed\n", __FILE__, __LINE__);
        perror(NULL);
        exit(-1);
    }

    printf("Test 2: fprintf()\n");
    Result = fprintf(fp, "Hello World!");
    if (Result < 0)
    {
        printf("%s(%d) fprintf() failed\n", __FILE__, __LINE__);
        perror(NULL);
        exit(-1);
    }

    printf("Test 3: fclose() on written file\n");
    Result = fclose(fp);
    if (0 != Result)
    {
        printf("%s(%d) fclose() failed\n", __FILE__, __LINE__);
        perror(NULL);
        exit(-1);
    }



    printf("Test 4: fopen() for read\n");
    fp = fopen("out.txt", "r");
    if (NULL == fp)
    {
        printf("%s(%d) fopen() failed\n", __FILE__, __LINE__);
        perror(NULL);
        exit(-1);
    }

    printf("Test 5: fscanf()\n");
    Result = fscanf(fp, "%31s", Buffer);
    if (EOF == Result)
    {
        printf("%s(%d) fscanf() failed\n", __FILE__, __LINE__);
        exit(-1);
    }
    printf("Contents of out.txt: %s\n", Buffer);
    if (0 != strcmp(Buffer, "Hello"))
    {
        printf("%s(%d) fscanf read out wrong string\n", __FILE__, __LINE__);
        exit(-1);
    }

    printf("Test 6: Determine size of file through fseek and ftell calls\n");
    Result = fseek(fp, 0, SEEK_END);
    if (0 != Result)
    {
        printf("%s(%d) fseek(..,0, SEEK_END) failed\n", __FILE__, __LINE__);
        perror(NULL);
        exit(-1);
    }
    Offset = ftell(fp);
    if (12 != Offset)
    {
        printf("%s(%d) ftell didn't return the expected value of 12\n", __FILE__, __LINE__);
        exit(-1);
    }

    printf("Test 7: Determine size of file through fstat() call\n");
    struct stat st;
    memset(&st, 0, sizeof(st));
    Result = fstat(__sfileno(fp), &st);
    if (0 != Result)
    {
        printf("%s(%d) fstat(__sfilno(fp), &st) failed\n", __FILE__, __LINE__);
        perror(NULL);
        exit(-1);
    }
    if (12 != st.st_size)
    {
        printf("%s(%d) fstat didn't return the expected size of 12\n", __FILE__, __LINE__);
        exit(-1);
    }

    printf("Test 8: Determine size of file through stat() call\n");
    memset(&st, 0, sizeof(st));
    Result = stat("out.txt", &st);
    if (0 != Result)
    {
        printf("%s(%d) stat(out.txt, &st) failed\n", __FILE__, __LINE__);
        perror(NULL);
        exit(-1);
    }
    if (12 != st.st_size)
    {
        printf("%s(%d) fstat didn't return the expected size of 12\n", __FILE__, __LINE__);
        exit(-1);
    }

    printf("Test 9: fclose() on read file\n");
    Result = fclose(fp);
    if (0 != Result)
    {
        printf("%s(%d) fclose() failed\n", __FILE__, __LINE__);
        perror(NULL);
        exit(-1);
    }


    printf("Test 10: rename() the test output file\n");
    Result = rename("out.txt", "out2.txt");
    if (0 != Result)
    {
        printf("%s(%d) rename(out.txt, out2.txt) failed\n", __FILE__, __LINE__);
        perror(NULL);
        exit(-1);
    }


    printf("Test 11: remove() the renamed output file\n");
    Result = remove("out2.txt");
    if (0 != Result)
    {
        printf("%s(%d) remove() failed\n", __FILE__, __LINE__);
        perror(NULL);
        exit(-1);
    }


    fprintf(stderr, "Test 12: Send this output to stderr\n");


    printf("\nTest completed\n");
}
