ENTRY(Reset_Handler)
MEMORY { FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 96K }
_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0x1000;
_Min_Stack_Size = 0x2000;
PHDRS { flash PT_LOAD FLAGS(5); ram PT_LOAD FLAGS(6); ramzero PT_LOAD FLAGS(6); }
SECTIONS {
.isr_vector : { KEEP(*(.isr_vector)) } > FLASH :flash
.text : { *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); } > FLASH :flash
.ARM.extab : { *(.ARM.extab*) } > FLASH :flash
.ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH :flash
.preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } > FLASH :flash
.init_array : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array)) PROVIDE_HIDDEN(__init_array_end = .); } > FLASH :flash
.fini_array : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array)) PROVIDE_HIDDEN(__fini_array_end = .); } > FLASH :flash
_sidata = LOADADDR(.data);
.data : { . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH :ram
.bss (NOLOAD) : AT(ADDR(.bss)) { . = ALIGN(4); _sbss = .; __bss_start__ = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; __bss_end__ = .; } > RAM :ramzero
.heap_stack (NOLOAD) : AT(ADDR(.heap_stack)) { . = ALIGN(8); PROVIDE(end = .); PROVIDE(_end = .); . += _Min_Heap_Size; . += _Min_Stack_Size; . = ALIGN(8); } > RAM :ramzero
/DISCARD/ : { *(.note.GNU-stack) }
}
