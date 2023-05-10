/******************************************************************************

 @file  cc13x2_cc26x2_app_tirtos7.cmd

 @brief CC26X2 or CC13X2 OAD specific linker configuration file for TI-RTOS with
        Code Composer Studio.

        Imported Symbols
        Note: Linker defines are located in the CCS IDE project by placing them
        in
        Properties->Build->Linker->Advanced Options->Command File Preprocessing.

        CACHE_AS_RAM:       Disable system cache to be used as GPRAM for
                            additional volatile memory storage.
        FLASH_ROM_BUILD:	If defined, it should be set to 1 or 2 to indicate
                            the ROM version of the device being used. When using
                            Flash-only configuration, this symbol should not be
                            defined.
        ICALL_RAM0_START:   RAM start of BLE stack.
        ICALL_STACK0_START: Flash start of BLE stack.
        PAGE_AlIGN:         Align BLE stack boundary to a page boundary.
                            Aligns to Flash word boundary by default.

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2017-2023, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*******************************************************************************
 * CCS Linker configuration
 */

/* Retain interrupt vector table variable                                    */
--retain=g_pfnVectors
/* Suppress warnings and errors:                                             */
/* - 10063: Warning about entry point not being _c_int00                     */
/* - 16011, 16012: 8-byte alignment errors. Observed when linking in object  */
/*   files compiled using Keil (ARM compiler)                                */
/* - 10068: (No matching section) caused by Z_ITERABLE_SECTION_ROM macro if  */
/*   section is empty (no symbols defined in the code that match the given   */
/*   section).                                                               */
--diag_suppress=10063,16011,16012,10068

/* The following command line options are set as part of the CCS project.    */
/* If you are building using the command line, or for some reason want to    */
/* define them here, you can uncomment and modify these lines as needed.     */
/* If you are using CCS for building, it is probably better to make any such */
/* modifications in your CCS project and leave this file alone.              */
/*                                                                           */
/* --heap_size=0                                                             */
--stack_size=1024
/* --library=rtsv7M3_T_le_eabi.lib                                           */

/* The starting address of the application.  Normally the interrupt vectors  */
/* must be located at the beginning of the application. Flash is 128KB, with */
/* sector length of 4KB                                                      */
/*******************************************************************************
 * Macros
 */
#define Z_ITERABLE_SECTION_ROM(struct_type, align) \
    struct_type ## _area : ALIGN(align) \
    { \
        _ ## struct_type ## _list_start = .; \
        *(._ ## struct_type.static.*) \
        _ ## struct_type ## _list_end = .; \
    } > FLASH

/*******************************************************************************
 * Memory Sizes
 */
#define FLASH_BASE   0x00000000
#define GPRAM_BASE   0x11000000
#define RAM_BASE     0x20000000
#define ROM_BASE     0x10000000

#define FLASH_SIZE   0x00058000
#define GPRAM_SIZE   0x00002000
#define RAM_SIZE     0x00013F00
#define ROM_SIZE     0x00040000

#define RTOS_RAM_SIZE           0x0000012C
#define RESERVED_RAM_SIZE_ROM_1 0x00000B08
#define RESERVED_RAM_SIZE_ROM_2 0x00000EB3

#define NET_BUF_BASE            0x20013F00
#define NET_BUF_SIZE            0x100

/*******************************************************************************
 * Memory Definitions
 ******************************************************************************/

/*******************************************************************************
 * RAM
 */
#if defined(FLASH_ROM_BUILD)
  #if (FLASH_ROM_BUILD == 1)
    #define RESERVED_RAM_SIZE_AT_START 0
    #define RESERVED_RAM_SIZE_AT_END   RESERVED_RAM_SIZE_ROM_1
  #else // (FLASH_ROM_BUILD == 2)
    #define RESERVED_RAM_SIZE_AT_START (RTOS_RAM_SIZE + RESERVED_RAM_SIZE_ROM_2)
    #define RESERVED_RAM_SIZE_AT_END   0
  #endif
#else /* Flash Only */
  #define RESERVED_RAM_SIZE_AT_START 0
  #define RESERVED_RAM_SIZE_AT_END   0
#endif // FLASH_ROM_BUILD

#define RAM_START      (RAM_BASE + RESERVED_RAM_SIZE_AT_START)
#ifdef ICALL_RAM0_START
  #define RAM_END      (ICALL_RAM0_START - 1)
#else
  #define RAM_END      (RAM_BASE + RAM_SIZE - RESERVED_RAM_SIZE_AT_END - 1)
#endif /* ICALL_RAM0_START */

/* For ROM 2 devices, the following section needs to be allocated and reserved */
#define RTOS_RAM_START RAM_BASE
#define RTOS_RAM_END   (RAM_BASE + RTOS_RAM_SIZE - 1)

/*******************************************************************************
 * Flash
 */

#define WORD_SIZE                  4

#define PAGE_SIZE                  0x2000

#ifdef PAGE_ALIGN
  #define FLASH_MEM_ALIGN          PAGE_SIZE
#else
  #define FLASH_MEM_ALIGN          WORD_SIZE
#endif /* PAGE_ALIGN */

#define PAGE_MASK                  0xFFFFE000

/* The last Flash page is reserved for the application. */
#define NUM_RESERVED_FLASH_PAGES   1
#define RESERVED_FLASH_SIZE        (NUM_RESERVED_FLASH_PAGES * PAGE_SIZE)

/* Check if page alingment with the Stack image is required.  If so, do not link
 * into a page shared by the Stack.
 */
#define  OAD_HDR_START             FLASH_BASE
#ifdef SECURITY
  #define  OAD_HDR_SIZE            0x90
#else
  #define  OAD_HDR_SIZE            0x50
#endif
#define  OAD_HDR_END               OAD_HDR_START + OAD_HDR_SIZE - 1

#define ENTRY_START                OAD_HDR_END + 1
#define ENTRY_SIZE                 0x40
#define ENTRY_END                  ENTRY_START + ENTRY_SIZE - 1

#define FLASH_START                (ENTRY_END + 1)

#ifdef ICALL_STACK0_START
  #ifdef PAGE_ALIGN
    #define ADJ_ICALL_STACK0_START (ICALL_STACK0_START * PAGE_MASK)
  #else
    #define ADJ_ICALL_STACK0_START ICALL_STACK0_START
  #endif /* PAGE_ALIGN */

  #define FLASH_END                (ADJ_ICALL_STACK0_START - 1)
#else
  #define FLASH_END                (FLASH_BASE + FLASH_SIZE - RESERVED_FLASH_SIZE - 1)
#endif /* ICALL_STACK0_START */

#define FLASH_LAST_PAGE_START      FLASH_END + 1

/*******************************************************************************
 * Stack
 */

/* Create global constant that points to top of stack */
/* CCS: Change stack size under Project Properties    */
__STACK_TOP = __stack + __STACK_SIZE;

/*******************************************************************************
 * GPRAM
 */

#ifdef CACHE_AS_RAM
  #define GPRAM_START GPRAM_BASE
  #define GPRAM_END   (GPRAM_START + GPRAM_SIZE - 1)
#endif /* CACHE_AS_RAM */

/*******************************************************************************
 * ROV
 * These symbols are used by ROV2 to extend the valid memory regions on device.
 * Without these defines, ROV will encounter a Java exception when using an
 * autosized heap. This is a posted workaround for a known limitation of
 * RTSC/rta. See: https://bugs.eclipse.org/bugs/show_bug.cgi?id=487894
 *
 * Note: these do not affect placement in RAM or FLASH, they are only used
 * by ROV2, see the BLE Stack User's Guide for more info on a workaround
 * for ROV Classic
 *
 */
__UNUSED_SRAM_start__ = RAM_BASE;
__UNUSED_SRAM_end__ = RAM_BASE + RAM_SIZE;

__UNUSED_FLASH_start__ = FLASH_BASE;
__UNUSED_FLASH_end__ = FLASH_BASE + FLASH_SIZE;

/*******************************************************************************
 * Main arguments
 */

/* Allow main() to take args */
/* --args 0x8 */

/*******************************************************************************
 * System Memory Map
 ******************************************************************************/
MEMORY
{
  /* EDITOR'S NOTE:
   * the FLASH and SRAM lengths can be changed by defining
   * ICALL_STACK0_START or ICALL_RAM0_START in
   * Properties->ARM Linker->Advanced Options->Command File Preprocessing.
   */

  /* Application stored in and executes from internal flash */
  FLASH (RX) : origin = FLASH_START, length = (FLASH_END - FLASH_START + 1)

  ENTRY (RX) : origin = ENTRY_START, length = ENTRY_SIZE
  /* CCFG Page, contains .ccfg code section and some application code. */
  FLASH_IMG_HDR (RX) : origin = OAD_HDR_START, length = OAD_HDR_SIZE

  /* Application uses internal RAM for data */
#if (defined(FLASH_ROM_BUILD) && (FLASH_ROM_BUILD == 2))
  RTOS_SRAM (RWX) : origin = RTOS_RAM_START, length = (RTOS_RAM_END - RTOS_RAM_START + 1)
#endif
  SRAM (RWX) : origin = RAM_START, length = (RAM_END - RAM_START + 1)

  #ifdef CACHE_AS_RAM
      GPRAM(RWX) : origin = GPRAM_START, length = GPRAM_SIZE
  #endif /* CACHE_AS_RAM */
  NET_BUF_SRAM (RWX) : origin = NET_BUF_BASE, length = NET_BUF_SIZE
}


/*******************************************************************************
 * Section Allocation in Memory
 ******************************************************************************/
SECTIONS
{
 GROUP > FLASH_IMG_HDR
  {
    .image_header //align PAGE_SIZE
  }

  GROUP > ENTRY
  {
    .resetVecs LOAD_START(prgEntryAddr)
    .intvecs
    EntrySection
  }
 // .intvecs        :   >  FLASH_START
  GROUP >>  FLASH
  {
    .text
    .const
    .constdata
    .rodata
    .emb_text
    .pinit
  }

  .cinit        : > FLASH LOAD_END(flashEndAddr)

  //.ccfg           :   >  FLASH_LAST_PAGE (HIGH)
  _net_buf_pool_area : {
     _net_buf_pool_list = .;
   } > NET_BUF_SRAM

   ._net_buf_pool.static.*  : > NET_BUF_SRAM

  Z_ITERABLE_SECTION_ROM(bt_mesh_app_key_cb, 4)
  Z_ITERABLE_SECTION_ROM(bt_mesh_friend_cb, 4)
  Z_ITERABLE_SECTION_ROM(bt_mesh_lpn_cb, 4)
  Z_ITERABLE_SECTION_ROM(bt_mesh_hb_cb, 4)
  Z_ITERABLE_SECTION_ROM(bt_mesh_subnet_cb, 4)
  Z_ITERABLE_SECTION_ROM(settings_handler_static, 4)

  GROUP > SRAM
  {
    .data LOAD_START(ramStartHere)
    #ifndef CACHE_AS_RAM
    .bss
    #endif /* CACHE_AS_RAM */
    .vtable
    .vtable_ram
    vtable_ram
    .sysmem
    .nonretenvar
    /*This keeps ll.o objects out of GPRAM, if no ll.o would be placed here
      the warning #10068 is supressed.*/
    #ifdef CACHE_AS_RAM
    ll_bss
    {
      --library=*ll_*.a<ll.o> (.bss)
      --library=*ll_*.a<ll_ae.o> (.bss)
    }
    #endif /* CACHE_AS_RAM */
    .noinit
    .ramVecs
  } LOAD_END(heapStart)

  .stack            :   >  SRAM (HIGH) LOAD_START(heapEnd)

  #ifdef CACHE_AS_RAM

  .bss :
  {
    *(.bss)
  } > GPRAM
  #endif /* CACHE_AS_RAM */
  .ramVecs
}

--symbol_map __TI_STACK_SIZE=__STACK_SIZE
--symbol_map __TI_STACK_BASE=__stack

-u_c_int00
--retain "*(.resetVecs)"
--retain "*(.vecs)"

SECTIONS
{
	.resetVecs: load > ENTRY_START
  .vecs: load > 0x20000000, type = NOLOAD
}
