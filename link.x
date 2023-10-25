INCLUDE memory.x



PROVIDE(_stext = ORIGIN(REGION_TEXT));
PROVIDE(_stack_start = ORIGIN(REGION_STACK) + LENGTH(REGION_STACK));
PROVIDE(_max_hart_id = 0);
PROVIDE(_hart_stack_size = 2K);
PROVIDE(_heap_size = 0);


PROVIDE(NonMaskableInt = DefaultHandler);
PROVIDE(HardFault = DefaultHandler);
PROVIDE(EcallM = DefaultHandler);
PROVIDE(EcallU = DefaultHandler);
PROVIDE(BreakPoint = DefaultHandler);
PROVIDE(SysTick = DefaultHandler);
PROVIDE(Software = DefaultHandler);

PROVIDE(TMR0 = DefaultHandler);
PROVIDE(GPIOA = DefaultHandler);
PROVIDE(GPIOB = DefaultHandler);
PROVIDE(SPI0 = DefaultHandler);
PROVIDE(BLEB = DefaultHandler);
PROVIDE(BLEL = DefaultHandler);
PROVIDE(USB = DefaultHandler);
PROVIDE(TMR1 = DefaultHandler);
PROVIDE(TMR2 = DefaultHandler);
PROVIDE(UART0 = DefaultHandler);
PROVIDE(UART1 = DefaultHandler);
PROVIDE(RTC = DefaultHandler);
PROVIDE(ADC = DefaultHandler);
PROVIDE(I2C = DefaultHandler);
PROVIDE(PWMX = DefaultHandler);
PROVIDE(TMR3 = DefaultHandler);
PROVIDE(UART2 = DefaultHandler);
PROVIDE(UART3 = DefaultHandler);
PROVIDE(WDOG_BAT = DefaultHandler);

PROVIDE(DefaultHandler = DefaultInterruptHandler);

ENTRY(_start)

SECTIONS
{
    .init :
    {
        . = ALIGN(4);
        KEEP(*(SORT_NONE(.init)))
        . = ALIGN(4);
    } >FLASH AT>FLASH

    .vector_table :
    {
        . = ALIGN(4);
        __vector_base = .;
        KEEP(*(.vector_table.interrupts));
        . = ALIGN(4);
    } >FLASH AT>FLASH

    .text :
    {
        . = ALIGN(4);
        KEEP(*(SORT_NONE(.handle_reset)))
        *(.text .text.*)
    } >FLASH AT>FLASH

    .highcode : ALIGN(4)
    {
        _highcode_lma = LOADADDR(.highcode);
        PROVIDE(_highcode_vma_start = .);
        KEEP(*(SORT_NONE(.trap))) /* All trap handlers */
        *(.highcode);
        *(.highcode.*);
		. = ALIGN(4);
        PROVIDE(_highcode_vma_end = .);
    } >RAM AT>FLASH

    .rodata : ALIGN(4)
    {
        *(.srodata .srodata.*);
        *(.rodata .rodata.*);
    } >FLASH AT>FLASH

    .data : ALIGN(4)
    {
        _data_lma = LOADADDR(.data);
        PROVIDE(_data_vma = .);
        *(.gnu.linkonce.r.*)
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        . = ALIGN(8);
        PROVIDE( __global_pointer$ = . + 0x800 );
        *(.sdata .sdata.*)
        *(.gnu.linkonce.s.*)
        . = ALIGN(8);
        *(.srodata.cst16)
        *(.srodata.cst8)
        *(.srodata.cst4)
        *(.srodata.cst2)
        *(.srodata .srodata.*)
        . = ALIGN(4);
        PROVIDE( _edata = .);
    } >RAM AT>FLASH

    .bss : ALIGN(4)
    {
        PROVIDE( _sbss = .);
        *(.sbss .sbss.* .bss .bss.*)
        PROVIDE( _ebss = .);
    } >RAM AT>FLASH

    .stack ORIGIN(RAM)+LENGTH(RAM) :
    {
        . = ALIGN(4);
        PROVIDE(_stack_top = . );
    } >RAM

    .got (INFO) :
    {
        KEEP(*(.got .got.*));
    }

    .eh_frame (INFO) : { KEEP(*(.eh_frame)) }
    .eh_frame_hdr (INFO) : { *(.eh_frame_hdr) }
}
