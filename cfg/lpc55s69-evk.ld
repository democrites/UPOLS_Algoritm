/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright (c) 2008-2013 Code Red Technologies Ltd,
 * Copyright 2015, 2018-2019 NXP
 * (c) NXP Semiconductors 2013-2020
 * Generated linker script file for LPC55S69
 * Created from linkscript.ldt by FMCreateLinkLibraries
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v11.1.1 [Build 3241] [2020-03-02] on 20 avr. 2020 11:23:36
 */

ENTRY(ResetISR)

GROUP (
  "libgcc.a"
  "libc.a"
  "libcr_newlib_nohost.a"
  "libm.a"
)

MEMORY
{
  /* Define each memory region */
  PROGRAM_FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x90000 /* 576K bytes (alias Flash) */  
  SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x18000 /* 96K bytes (alias RAM) */  
  CALC_RAM (rwx) : ORIGIN = 0x20018000, LENGTH = 0x18000 /* 96K bytes (alias RAM2) */  
  SRAM_CORE1 (rwx) : ORIGIN = 0x20030000, LENGTH = 0xc000 /* 48K bytes (alias RAM3) */  
  SHMEM (rwx) : ORIGIN = 0x2003c000, LENGTH = 0x4000 /* 16K bytes (alias RAM4) */  
  SRAM_PQ (rwx) : ORIGIN = 0x20040000, LENGTH = 0x4000 /* 16K bytes (alias RAM5) */  
  SRAMX (rwx) : ORIGIN = 0x4000000, LENGTH = 0x8000 /* 32K bytes (alias RAM6) */  
  USB_RAM (rwx) : ORIGIN = 0x40100000, LENGTH = 0x4000 /* 16K bytes (alias RAM7) */  
}

  /* Define a symbol for the top of each memory region */
  __base_PROGRAM_FLASH = 0x0  ; /* PROGRAM_FLASH */  
  __top_PROGRAM_FLASH = 0x0 + 0x90000 ; /* 576K bytes */  
  __base_SRAM = 0x20000000  ; /* SRAM */  
  __base_RAM = 0x20000000 ; /* RAM */  
  __top_SRAM = 0x20000000 + 0x18000 ; /* 96K bytes */  
  __top_RAM = 0x20000000 + 0x18000 ; /* 96K bytes */  
  __base_CALC_RAM = 0x20018000  ; /* CALC_RAM */  
  __top_CALC_RAM = 0x20018000 + 0x18000 ; /* 96K bytes */  
  __base_SRAM_CORE1 = 0x20030000  ; /* SRAM_CORE1 */  
  __top_SRAM_CORE1 = 0x20030000 + 0xc000 ; /* 48K bytes */  
  __base_shmem = 0x2003c000  ; /* shmem */  
  __top_shmem = 0x2003c000 + 0x4000 ; /* 16K bytes */  
  __base_SRAM_PQ = 0x20040000  ; /* SRAM_PQ */  
  __top_SRAM_PQ = 0x20040000 + 0x4000 ; /* 16K bytes */  
  __base_SRAMX = 0x4000000  ; /* SRAMX */  
  __top_SRAMX = 0x4000000 + 0x8000 ; /* 32K bytes */  
  __base_USB_RAM = 0x40100000  ; /* USB_RAM */  
  __top_USB_RAM = 0x40100000 + 0x4000 ; /* 16K bytes */  

SECTIONS
{
     /* MAIN TEXT SECTION */
    .text : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))
        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        LONG(LOADADDR(.data_core1_text));
        LONG(    ADDR(.data_core1_text));
        LONG(  SIZEOF(.data_core1_text));
        LONG(LOADADDR(.data_core1_ARM_extab));
        LONG(    ADDR(.data_core1_ARM_extab));
        LONG(  SIZEOF(.data_core1_ARM_extab));
        LONG(LOADADDR(.data_core1_ARM_exidx));
        LONG(    ADDR(.data_core1_ARM_exidx));
        LONG(  SIZEOF(.data_core1_ARM_exidx));
        LONG(LOADADDR(.data_core1_data));
        LONG(    ADDR(.data_core1_data));
        LONG(  SIZEOF(.data_core1_data));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

       *(.text*)
       KEEP (*(.init))
       KEEP (*(.fini))
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(4);
    } > PROGRAM_FLASH
    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > PROGRAM_FLASH

    .ARM.exidx : ALIGN(4)
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > PROGRAM_FLASH
 
    _etext = .;
        
     /* USB_RAM */
    .m_usb_data (NOLOAD) :
    {
        *(m_usb_global)
    } > USB_RAM AT> USB_RAM

    /* DATA section for SRAM_CORE1 */
    .data_core1_text : 
    {
       FILL(0xff)
       PROVIDE(__start_data_RAM3 = .) ;
       __core_m33slave_START__ = .; /* start of slave image */
       KEEP(*(.core_m33slave))
    } > SRAMX AT>PROGRAM_FLASH

    /* M33SLAVE extab and exidx sections */
    .data_core1_ARM_extab :
    {
        FILL(0xff)
        KEEP(*(.core_m33slave.ARM.extab))
    } > SRAMX AT>PROGRAM_FLASH 

    .data_core1_ARM_exidx :
    {
        FILL(0xff)
        KEEP(*(.core_m33slave.ARM.exidx))
    } > SRAMX AT>PROGRAM_FLASH 

    /* M33SLAVE data section */
    .data_core1_data :
    {
        FILL(0xff)
        KEEP(*(.core_m33slave.data_*)) KEEP(*(.core_m33slave.data))
        __core_m33slave_END__ = .; /* end of slave image */

        /* perform some simple sanity checks */
#        ASSERT(!(__core_m33slave_START__ == __core_m33slave_END__), "No slave code for _core_m33slave");
#        ASSERT( (ABSOLUTE(__core_m33slave_START__) == __vectors_start___core_m33slave), "M33SLAVE execute address differs from address provided in source image");
    } > SRAM_CORE1 AT>PROGRAM_FLASH 


    /* MAIN DATA SECTION */
    .data : ALIGN(4) {
       FILL(0xff)
       _data = . ;
       PROVIDE(__start_data_RAM = .) ;
       PROVIDE(__start_data_SRAM = .) ;
       *(vtable)
       *(.ramfunc*)
       KEEP(*(CodeQuickAccess))
       KEEP(*(DataQuickAccess))
       *(RamFunction)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
       PROVIDE(__end_data_RAM = .) ;
       PROVIDE(__end_data_SRAM = .) ;
    } > SRAM AT>PROGRAM_FLASH

    /* MAIN BSS SECTION */
    .bss (NOLOAD) : ALIGN(4) {
        _bss = .;
        PROVIDE(__start_bss_RAM = .) ;
        PROVIDE(__start_bss_SRAM = .) ;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(__end_bss_RAM = .) ;
        PROVIDE(__end_bss_SRAM = .) ;
        PROVIDE(end = .);
    } > SRAM

    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        PROVIDE(__start_noinit_RAM = .) ;
        PROVIDE(__start_noinit_SRAM = .) ;
        *(.noinit*)
         . = ALIGN(4) ;
        _end_noinit = .;
       PROVIDE(__end_noinit_RAM = .) ;
       PROVIDE(__end_noinit_SRAM = .) ;        
    } > SRAM

    /* Reserve and place Heap within memory map */
    _HeapSize = 0x2000;
    .heap (NOLOAD) :  ALIGN(4)
    {
        _pvHeapStart = .;
        . += _HeapSize;
        . = ALIGN(4);
        _pvHeapLimit = .;
    } > SRAM

     _StackSize = 0x8000;
     /* Reserve space in memory for Stack */
    .heap2stackfill (NOLOAD) :
    {
        . += _StackSize;
    } > SRAM
    /* Locate actual Stack in memory map */
    .stack ORIGIN(SRAM) + LENGTH(SRAM) - _StackSize - 0 (NOLOAD) :  ALIGN(4)
    {
        _vStackBase = .;
        . = ALIGN(4);
        _vStackTop = . + _StackSize;
    } > SRAM

	.shmem (NOLOAD) : ALIGN(4) {
		*(.shmem)
	} >SHMEM
	
    /* ## Create checksum value (used in startup) ## */
    PROVIDE(__valid_user_code_checksum = 0 - 
                                         (_vStackTop 
                                         + (ResetISR + 1) 
                                         + (NMI_Handler + 1) 
                                         + (HardFault_Handler + 1) 
                                         + (( DEFINED(MemManage_Handler) ? MemManage_Handler : 0 ) + 1)   /* MemManage_Handler may not be defined */
                                         + (( DEFINED(BusFault_Handler) ? BusFault_Handler : 0 ) + 1)     /* BusFault_Handler may not be defined */
                                         + (( DEFINED(UsageFault_Handler) ? UsageFault_Handler : 0 ) + 1) /* UsageFault_Handler may not be defined */
                                         ) );

    /* Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
}
