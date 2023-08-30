_main_stack_size = 1024;                             /* main stack = 512Bytes */
_main_stack_top  = 0x20000000 + _main_stack_size;
ASSERT(_main_stack_size   % 8 == 0, "MAIN stack size error");

/* Mapping the heap to the end of SRAM2 */
_heap_end = 0x20000000 + 32K;                       /* end of available ram */

/* Identify the Entry Point  */
ENTRY(_Z13Reset_Handlerv)

MEMORY
{
    sram(wx)  : ORIGIN = 0x20000400, LENGTH = 32K
    flash(rx) : ORIGIN = 0x08000000, LENGTH = 128K
}

SECTIONS
{
    . = 0;
    
    /* .text section: code goes to flash */
    .text :
    {
        /* Before the isr_vector put a jump to the Reset_Handler */
        KEEP(*(.reset_handler))
        KEEP(*(.isr_vector))
        
        *(.text)
        *(.text.*)
        *(.gnu.linkonce.t.*)
        /* These sections for thumb interwork? */
        *(.glue_7)
        *(.glue_7t)
        /* These sections for C++? */
        *(.gcc_except_table)
        *(.gcc_except_table.*)
        *(.ARM.extab*)
        *(.gnu.linkonce.armextab.*)

        . = ALIGN(4);
        /* .rodata: constant data */
        *(.rodata)
        *(.rodata.*)
        *(.gnu.linkonce.r.*)

        /* C++ Static constructors/destructors (eabi) */
        . = ALIGN(4);
        KEEP(*(.init))
        
        . = ALIGN(4);
        __miosix_init_array_start = .;
        KEEP (*(SORT(.miosix_init_array.*)))
        KEEP (*(.miosix_init_array))
        __miosix_init_array_end = .;

        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;

        . = ALIGN(4);
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;

        . = ALIGN(4);
        KEEP(*(.fini))

        . = ALIGN(4);
        __fini_array_start = .;
        KEEP (*(.fini_array))
        KEEP (*(SORT(.fini_array.*)))
        __fini_array_end = .;

        /* C++ Static constructors/destructors (elf)  */
        . = ALIGN(4);
        _ctor_start = .;
        KEEP (*crtbegin.o(.ctors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*crtend.o(.ctors))
       _ctor_end = .;

        . = ALIGN(4);
        KEEP (*crtbegin.o(.dtors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*crtend.o(.dtors))
    } > flash
    
    /* .ARM.exidx is sorted, so has to go in its own output section.  */
    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > flash
    __exidx_end = .;

	/*
     * .data section: global variables go to sram, but also store a copy to
     * flash to initialize them
     */
    .data : ALIGN(4)
    {
        _data = .;
        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d.*)
        . = ALIGN(4);
        PROVIDE(__global_pointer$ = . + 0x800);
        *(.sdata)
        *(.sdata.*)
        . = ALIGN(4);
        *(.sdata2)
        *(.sdata2.*)
        _edata = .;
    } > sram AT > flash
    _etext = LOADADDR(.data);

    /* .bss section: uninitialized global variables go to sram */
    _bss_start = .;
    .bss :
    {
        *(.bss)
        *(.bss.*)
        *(.gnu.linkonce.b.*)
        . = ALIGN(4);
        *(.sbss)
        *(.sbss.*)
        . = ALIGN(8);
    } > sram
    _bss_end = .;

    _end = .;
    PROVIDE(end = .);
}
