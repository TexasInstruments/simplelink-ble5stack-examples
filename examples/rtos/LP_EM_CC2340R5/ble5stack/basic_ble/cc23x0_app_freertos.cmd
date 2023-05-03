/******************************************************************************

 @file  cc23x0_app_freertos.cmd

 @brief cc23x0R5 linker configuration file for FreeRTOS
        with Code Composer Studio.

        Imported Symbols
        Note: Linker defines are located in the CCS IDE project by placing them
        in
        Properties->Build->Linker->Advanced Options->Command File Preprocessing.

        ICALL_RAM0_START:   RAM start of BLE stack.
        ICALL_STACK0_START: Flash start of BLE stack.
        PAGE_AlIGN:         Align BLE stack boundary to a page boundary.
                            Aligns to Flash word boundary by default.

 Group: WCS, BTS
 Target Device: cc23xx

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

/* Override default entry point.                                             */
--entry_point=resetISR

/* Retain interrupt vector table variable                                    */
--retain=resetVectors

/* Suppress warnings and errors:                                             */
/* - 10063: Warning about entry point not being _c_int00                     */
/* - 16011, 16012: 8-byte alignment errors. Observed when linking in object  */
/*   files compiled using Keil (ARM compiler)                                */
--diag_suppress=10063,16011,16012
--heap_size=0
--stack_size=800
/* The following command line options are set as part of the CCS project.    */
/* If you are building using the command line, or for some reason want to    */
/* define them here, you can uncomment and modify these lines as needed.     */
/* If you are using CCS for building, it is probably better to make any such */
/* modifications in your CCS project and leave this file alone.              */
/*                                                                           */
/* --heap_size=0                                                             */
/* --stack_size=256                                                          */
/* --library=rtsv7M3_T_le_eabi.lib                                           */

/* The starting address of the application.  Normally the interrupt vectors  */
/* must be located at the beginning of the application. Flash is 128KB, with */
/* sector length of 4KB                                                      */
/*******************************************************************************
 * Memory Sizes
 */
#ifdef OAD_CFG
#define PERSISTENT_BASE 0x6000
#define NVS_BASE        0x30000
#define APP_BASE        0x34000
#define MCU_HDR_SIZE    0x100
#define MCU_TLV_SIZE    0x120
#define MCUBOOT_SIZE    0x6000
#define NVS_SIZE        0x4000
#define MCU_OVERHEADS (MCU_HDR_SIZE + MCU_TLV_SIZE)
#endif //OAD_CFG

#define FLASH_BASE   0x00000000
#define RAM_BASE     0x20000000

#define FLASH_SIZE   0x00080000
#define RAM_SIZE     0x00009000

#define CCFG_BASE    0x4E020000
#define CCFG_SIZE    0x800

/*******************************************************************************
 * Memory Definitions
 ******************************************************************************/

/*******************************************************************************
 * RAM
 */
#define RAM_START      (RAM_BASE)
#ifdef ICALL_RAM0_START
  #define RAM_END      (ICALL_RAM0_START - 1)
#else
  #define RAM_END      (RAM_BASE + RAM_SIZE - 1)
#endif /* ICALL_RAM0_START */

/*******************************************************************************
 * Flash
 */

#define FLASH_START                FLASH_BASE
#define WORD_SIZE                  4

#define PAGE_SIZE                  0x800

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

#define FLASH_LAST_PAGE_START      (FLASH_SIZE - PAGE_SIZE)

/*******************************************************************************
 * Stack
 */

/* Create global constant that points to top of stack */
/* CCS: Change stack size under Project Properties    */
__STACK_TOP = __stack + __STACK_SIZE;

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

#ifdef OAD_CFG
    /* Application stored in and executes from internal flash
     * Note there are some holes in this memory due to issues with
     * first-sampling silicon.
     * FLASH_1 - MCUBoot
     * FLASH_2m - Persistent_app header (Secondary image)
     * FLASH_2 - Persistent_app  (Secondary image)
     * FLASH_3m - Main_app header (Primary image)
     * FLASH_3 - Main_app  (Primary image)
     */

    FLASH_1 (RX) : origin = FLASH_BASE                       ,length = MCUBOOT_SIZE
    FLASH_2m(RX) : origin = PERSISTENT_BASE                  ,length = MCU_HDR_SIZE
    FLASH_2 (RX) : origin = (PERSISTENT_BASE + MCU_HDR_SIZE) ,length = (NVS_BASE - MCU_OVERHEADS - PERSISTENT_BASE)
    NVS(RX)      : origin = NVS_BASE                         ,length = NVS_SIZE
    FLASH_3m(RX) : origin = APP_BASE                         ,length = MCU_HDR_SIZE
    FLASH_3 (RX) : origin = (APP_BASE + MCU_HDR_SIZE)        ,length = (FLASH_SIZE - APP_BASE - MCU_HDR_SIZE - PAGE_SIZE)
#else

  /* Application stored in and executes from internal flash */
  FLASH (RX) : origin = FLASH_START, length = (FLASH_END - FLASH_START + 1)

#endif //OAD_CFG

  /* contains some application code. */
  FLASH_LAST_PAGE (RX) :  origin = FLASH_LAST_PAGE_START, length = PAGE_SIZE

  /* Application uses internal RAM for data */
  SRAM (RWX) : origin = RAM_START, length = (RAM_END - RAM_START + 1)

  CCFG (RW) : origin = CCFG_BASE, length = CCFG_SIZE
}

/*******************************************************************************
 * Section Allocation in Memory
 ******************************************************************************/
SECTIONS
{
#ifdef OAD_CFG
  .mcuboot_hdr    :   >  FLASH_3m, type = NOLOAD
  .resetVecs      :   >  (APP_BASE + MCU_HDR_SIZE)
  .text           :   >> FLASH_3|FLASH_LAST_PAGE
  .const          :   >> FLASH_3|FLASH_LAST_PAGE
  .constdata      :   >> FLASH_3|FLASH_LAST_PAGE
  .rodata         :   >> FLASH_3|FLASH_LAST_PAGE
  .cinit          :   >  FLASH_3|FLASH_LAST_PAGE
  .pinit          :   >> FLASH_3|FLASH_LAST_PAGE
  .init_array     :   >  FLASH_3|FLASH_LAST_PAGE
  .emb_text       :   >> FLASH_3|FLASH_LAST_PAGE
  .ccfg           :   >  CCFG

#else
  .resetVecs      :   >  FLASH_START
  .text           :   >> FLASH | FLASH_LAST_PAGE
  .const          :   >> FLASH | FLASH_LAST_PAGE
  .constdata      :   >> FLASH | FLASH_LAST_PAGE
  .rodata         :   >> FLASH | FLASH_LAST_PAGE
  .cinit          :   >  FLASH | FLASH_LAST_PAGE
  .pinit          :   >> FLASH | FLASH_LAST_PAGE
  .init_array     :   >  FLASH | FLASH_LAST_PAGE
  .emb_text       :   >> FLASH | FLASH_LAST_PAGE
  .ccfg           :   >  CCFG

#endif //OAD_CFG

  .vtable         :   > SRAM
  .vtable_ram     :   > SRAM
  vtable_ram      :   > SRAM
  .data           :   > SRAM
  .bss            :   > SRAM
  .sysmem         :   > SRAM
  .nonretenvar    :   > SRAM
  .heap           :   > SRAM
  .stack          :   > SRAM (HIGH)

}
