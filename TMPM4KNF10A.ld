/**
 * @file        TMPM4KNF10A.ld
 * @brief       GNU linker script for the TOSHIBA TMPM4KNF10AFG.
 *
 * @details
 *   Memory layout transcribed from the uVision scatter file TMPM4KNA.scat:
 *     LR_IROM1        0x00000000  1 MB    code flash
 *     FLASH_CODE_RAM  0x20000000  1 KB    executes from RAM, loaded from flash
 *     RW_IRAM1        0x20000400  63 KB   read-write and zero-init data
 *
 *   The stack is placed at the top of RAM and grows down, which is the GNU
 *   convention. The scatter file instead reserved a fixed 512 byte block, so
 *   this arrangement gives more headroom rather than less.
 *
 *   Symbols consumed by startup_TMPM4KNA.S:
 *     _estack, _sidata, _sdata, _edata, _sbss, _ebss,
 *     _siramfunc, _sramfunc, _eramfunc
 */

ENTRY(Reset_Handler)

MEMORY
{
    FLASH    (rx)  : ORIGIN = 0x00000000, LENGTH = 1024K
    RAMFUNC  (rwx) : ORIGIN = 0x20000000, LENGTH = 1K
    RAM      (rwx) : ORIGIN = 0x20000400, LENGTH = 63K
}

/* Link fails rather than silently overflowing if these cannot be honoured. */
_Min_Heap_Size  = 0x000;
_Min_Stack_Size = 0x400;

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    /* Vector table first, at address zero. */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
    .ARM : {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } >FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } >FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } >FLASH

    /* Code that must execute from RAM, the FLASH_ROM region in the scatter
     * file. Stored in flash, copied by Reset_Handler before main runs. */
    _siramfunc = LOADADDR(.ramfunc);

    .ramfunc :
    {
        . = ALIGN(4);
        _sramfunc = .;
        *(.ramfunc)
        *(.ramfunc*)
        *(FLASH_ROM)
        . = ALIGN(4);
        _eramfunc = .;
    } >RAMFUNC AT >FLASH

    /* Initialised data: lives in RAM, image stored in flash. */
    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT >FLASH

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >RAM

    /* Reserve room so the link reports an overflow instead of the stack
     * quietly colliding with .bss at run time. */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >RAM

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
