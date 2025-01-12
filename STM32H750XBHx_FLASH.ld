/*
******************************************************************************
**

**  File        : LinkerScript.ld
**
**  Author		: STM32CubeMX
**
**  Abstract    : Linker script for STM32H750XBHx series
**                16908288Kbytes QSPIFLASH and 0Kbytes DTCMRAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
**
** Redistribution and use in source and binary forms, with or without modification,
** are permitted provided that the following conditions are met:
**   1. Redistributions of source code must retain the above copyright notice,
**      this list of conditions and the following disclaimer.
**   2. Redistributions in binary form must reproduce the above copyright notice,
**      this list of conditions and the following disclaimer in the documentation
**      and/or other materials provided with the distribution.
**   3. Neither the name of STMicroelectronics nor the names of its contributors
**      may be used to endorse or promote products derived from this software
**      without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);    /* end of DTCMRAM */

/* Specify the memory areas */
MEMORY
{
  DTCMRAM (rwx)      : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_AXI0 (rwx)      : ORIGIN = 0x24000000, LENGTH = 256K
  RAM_D2 (rwx)      : ORIGIN = 0x30000000, LENGTH = 288K
  RAM_D3 (rwx)      : ORIGIN = 0x38000000, LENGTH = 64K
  ITCMRAM (rwx)      : ORIGIN = 0x00000000, LENGTH = 64K
  RAM_AXI1 (rwx)      : ORIGIN = 0x24040000, LENGTH = 256K
  EXSDRAM (rw)      : ORIGIN = 0xc0000000, LENGTH = 32M
  QSPIFLASH (rx)      : ORIGIN = 0x90000000, LENGTH = 16M
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into QSPIFLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    __isr_vector_start__ = .;
    KEEP(*(.isr_vector)) /* Startup code */
    __isr_vector_end__ = .;
    . = ALIGN(4);
  } >QSPIFLASH

  .fast_ram :
  {
    . = ALIGN(8);
    __fast_ram_start__ = .;
    KEEP(*(.fast_ram*))
    KEEP(*(.text._fast_ram_*))
    __fast_ram_end__ = .;
    . = ALIGN(8);
  } > ITCMRAM AT> QSPIFLASH
  __fast_ram_init_load_addr__ = LOADADDR(.fast_ram);

  /* The program code and other data goes into QSPIFLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >QSPIFLASH

  /* Constant data goes into QSPIFLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >QSPIFLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >QSPIFLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >QSPIFLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >QSPIFLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >QSPIFLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >QSPIFLASH

  /* used by the startup to initialize data */
  __data_load_addr__ = LOADADDR(.data);

  /* Initialized data sections goes into DTCMRAM, load LMA copy after code */
  .data : 
  {
    . = ALIGN(4);
    __data_start__ = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    __data_end__ = .;        /* define a global symbol at data end */
  } > DTCMRAM AT> QSPIFLASH

  
  /* Uninitialized data section */
  . = ALIGN(4);
  .bss (NOLOAD) :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } > DTCMRAM

  .axisram.data : {
    . = ALIGN(4);
    __axisram_data_start_addr__ = .;
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    __axisram_data_end_addr__ = .;
    . = ALIGN(4);
  } > RAM_AXI0 AT> QSPIFLASH
  __axisram_data_load_addr__ = LOADADDR(.axisram.data);

  .axisram.bss (NOLOAD) : {
    . = ALIGN(4);
    __axisram_bss_start_addr__ = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    __axisram_bss_end_addr__ = .;
    . = ALIGN(4);
  } > RAM_AXI0 AT> QSPIFLASH

  .noncacheable.init : {
      . = ALIGN(8);
      __noncacheable_init_start__ = .;
      KEEP(*(.noncacheable.init))
      __noncacheable_init_end__ = .;
      . = ALIGN(8);
  } > RAM_AXI1 AT> QSPIFLASH
  __noncacheable_init_load_addr__ = LOADADDR(.noncacheable.init);

  .noncacheable.bss (NOLOAD) : {
    . = ALIGN(8);
    KEEP(*(.noncacheable))
    __noncacheable_bss_start__ = .;
    KEEP(*(.noncacheable.bss))
    __noncacheable_bss_end__ = .;
    . = ALIGN(8);
  } > RAM_AXI1

  . = ORIGIN(EXSDRAM);
  .sdram (NOLOAD) :
  {
    . = ALIGN(8);
    KEEP(*(.sdram))
    __sdram_bss_start__ = .;
    KEEP(*(.sdram.bss))
    __sdram_bss_end__ = .;
    __heap_start__ = .;
    . = ALIGN(8);
  } > EXSDRAM
  __heap_end__ = ORIGIN(EXSDRAM) + LENGTH(EXSDRAM);

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}


