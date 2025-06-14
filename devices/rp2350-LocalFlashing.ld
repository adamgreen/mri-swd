/* Based on GCC ARM embedded samples.
   Defines the following symbols for use by code:
    __exidx_start
    __exidx_end
    __etext
    __data_start__
    __preinit_array_start
    __preinit_array_end
    __init_array_start
    __init_array_end
    __fini_array_start
    __fini_array_end
    __data_end__
    __bss_start__
    __bss_end__
    __end__
    end
    __HeapLimit
    __StackLimit
    __StackTop
    __stack (== StackTop)
*/

MEMORY
{
    RAM(rwx) : ORIGIN =  0x20000000, LENGTH = 520k
}

ENTRY(_entry_point)

SECTIONS
{
    /* Note in NO_FLASH builds the entry point for both the bootrom, and debugger
       entry (ELF entry point), are *first* in the image, and the vector table
       follows immediately afterward. This is because the bootrom enters RAM
       binaries directly at their lowest address (preferring main RAM over XIP
       cache-as-SRAM if both are used).
    */

    .text : {
        _entry_point = .;
        __logical_binary_start = .;
        __reset_start = .;
        KEEP (*(.data.g_config))
        KEEP (*(.reset))
        __reset_end = .;
        KEEP (*(.binary_info_header))
        __binary_info_header_end = .;
        /* . = ALIGN(256); */
          KEEP (*(.vectors))
        *(.time_critical*)
        *(.text*)
        . = ALIGN(4);
        *(.init)
        *(.fini)
        /* Pull all c'tors into .text */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)
        /* Followed by destructors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        *(.eh_frame*)
    } > RAM

    .rodata : {
        *(.rodata*)
        . = ALIGN(4);
        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.flashdata*)))
        . = ALIGN(4);
    } > RAM

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > RAM

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > RAM
    __exidx_end = .;

    /* Machine inspectable binary information */
    . = ALIGN(4);
    __binary_info_start = .;
    .binary_info :
    {
        KEEP(*(.binary_info.keep.*))
        *(.binary_info.*)
    } > RAM
    __binary_info_end = .;
    . = ALIGN(4);

    .data : {
        __data_start__ = .;
        *(vtable)
        *(.data*)

        . = ALIGN(4);
        *(.after_data.*)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__mutex_array_start = .);
        KEEP(*(SORT(.mutex_array.*)))
        KEEP(*(.mutex_array))
        PROVIDE_HIDDEN (__mutex_array_end = .);

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(SORT(.preinit_array.*)))
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        *(SORT(.fini_array.*))
        *(.fini_array)
        PROVIDE_HIDDEN (__fini_array_end = .);

        *(.jcr)
        . = ALIGN(4);
        /* All data end */
        __data_end__ = .;
    } > RAM

    .uninitialized_data (NOLOAD): {
        . = ALIGN(4);
        *(.uninitialized_data*)
    } > RAM
    /* __etext is (for backwards compatibility) the name of the .data init source pointer (...) */
    __etext = LOADADDR(.data);

    .bss  : {
        . = ALIGN(4);
        __bss_start__ = .;
        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.bss*)))
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    .heap (NOLOAD):
    {
        __end__ = .;
        end = __end__;
        KEEP(*(.heap*))
        __HeapLimit = .;
    } > RAM
}

