// ***************************************************************************************
//
// zjag.c - Broadcom EJTAG Debrick Utility v1.8 RC3 - by Volkan K.
//
//////////////////////////////////////////////////////////////////////////////////////////
// Re-baseline the brjtag and tornado version - zjtag is based on brjtag 1.9o
// zjtag supports DIYGADGET's USB Multi Protocol Adapter (TUMPA), TUMPA-Lite and 
// DIYGADGET's Parallel JTAG Cables
//
// zJTAG project website:
//   http://zjtag.sourceforge.net/
// zJTAG project downloads:
//   http://sourceforge.net/projects/zjtag/files
//
// For more information on TUMPA and other JTAG adapters, please visit:
//   http://www.diygadget.com
// For product manuals and tutorials, please visit:
//   http://www.tiaowiki.com
//
//////////////////////////////////////////////////////////////////////////////////////////
// Includes code from brjtag: written by hugebird  @ http://www.chinadsl.net/
// http://www.chinadsl.net/thread-21684-1-1.html
//////////////////////////////////////////////////////////////////////////////////////////
//==========================================================================                     
// Includes code from TJTAG: Tornado's modifications
// tornado@odessaua.com
// - Thanks to HDM's great work
// **************************************************************************
//  Includes code written by HairyDairyMaid (a.k.a. - lightbulb)
//  hairydairymaid@yahoo.com
// **************************************************************************
//
//  This program is copyright (C) 2004-2006 HairyDairyMaid (a.k.a. Lightbulb),
//  2007-2012 Tornado, 2008-2011 hugebird, 2012-2013 Volkan K.
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of version 2 the GNU General Public License as published
//  by the Free Software Foundation.
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//  To view a copy of the license go to:
//  http://www.fsf.org/copyleft/gpl.html
//  To receive a copy of the GNU General Public License write the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
// **************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>

#if ( defined(_MSC_VER) && !defined(WINDOWS_VERSION) )
 #define WINDOWS_VERSION             //windows make with: cl xjtag.c xxx.c
#endif           


#ifdef WINDOWS_VERSION
  #include <windows.h>
  #define strcasecmp  stricmp
  #define strncasecmp strnicmp
  #define mssleep(s)    Sleep(s)
  #define ussleep(s)  iusleep(s)
    
#else   //For linux

   #include <unistd.h>
   #include <sys/ioctl.h>

   #ifdef __FreeBSD__
      #include <dev/ppbus/ppi.h>
      #include <dev/ppbus/ppbconf.h>
      #define PPWDATA PPISDATA
      #define PPRSTATUS PPIGSTATUS
   #else
      #include <linux/ppdev.h>
   #endif

#ifndef FTDIXXX
  #ifndef MY_TYPE
    #define MY_TYPE
    typedef uint32_t     DWORD;
    typedef uint16_t     WORD;
    typedef unsigned char     BYTE;
    typedef int               bool;
  #endif
#else
    #define MY_TYPE
#endif

  #define mssleep(s)   usleep((s)* 1000)
  #define ussleep(s)   usleep(s)

#endif


#define true  1
#define false 0

#define MAX_ATTEMPTS 3
#define MAX_TIMEOUT 200
#define MAX_LOOP_CNT 1000

/*
 * The followint table shows the pin assignment of 25-pin  Parallel Printer Port.
 * please refer to IEEE 1284 standard for detailed description.
 * Operate in SPP way, set to ECP mode to get better speed
 * data port (Out) (0x378)                 status port (In) (0x379)
 * bit[7] -- pin9 (Out)         bit[7] -- pin11 (In), busy (Hardware Inverted)
 * bit[6] -- pin8 (Out)         bit[6] -- pin10 (In), Ack
 * bit[5] -- pin7 (Out)         bit[5] -- pin12 (In), Paper out
 * bit[4] -- pin6 (Out)         bit[4] -- pin13 (In), Select
 * bit[3] -- pin5 (Out)         bit[3] -- pin15 (In), Error
 * bit[2] -- pin4 (Out)         bit[2] -- IRQ(Not)
 * bit[1] -- pin3 (Out)         bit[1] -- Reserved
 * bit[0] -- pin2 (Out)         bit[0] -- Reserved
 */
#define PORT378	0x378

/* OOCD paraport cable info
	   // name				tdo   trst  tms   tck   tdi   srst  o_inv i_inv init  exit  led 
	{ "wiggler",			0x80, 0x10, 0x02, 0x04, 0x08, 0x01, 0x01, 0x80, 0x80, 0x80, 0x00 },
	{ "wiggler2",			0x80, 0x10, 0x02, 0x04, 0x08, 0x01, 0x01, 0x80, 0x80, 0x00, 0x20 },
	{ "wig_ntrst_inv",      0x80, 0x10, 0x02, 0x04, 0x08, 0x01, 0x11, 0x80, 0x80, 0x80, 0x00 },
	{ "old_amt_wig",	    0x80, 0x01, 0x02, 0x04, 0x08, 0x10, 0x11, 0x80, 0x80, 0x80, 0x00 },
	{ "arm-jtag",			0x80, 0x01, 0x02, 0x04, 0x08, 0x10, 0x01, 0x80, 0x80, 0x80, 0x00 },
	{ "chameleon",		    0x80, 0x00, 0x04, 0x01, 0x02, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00 },
	{ "dlc5",			    0x10, 0x00, 0x04, 0x02, 0x01, 0x00, 0x00, 0x00, 0x10, 0x10, 0x00 },
	{ "triton",			    0x80, 0x08, 0x04, 0x01, 0x02, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00 },
	{ "lattice",			0x40, 0x10, 0x04, 0x02, 0x01, 0x08, 0x00, 0x00, 0x18, 0x18, 0x00 },
	{ "flashlink",		    0x20, 0x10, 0x02, 0x01, 0x04, 0x20, 0x30, 0x20, 0x00, 0x00, 0x00 },
	{ "altium",			    0x10, 0x20, 0x04, 0x02, 0x01, 0x80, 0x00, 0x00, 0x10, 0x00, 0x08 },
	{ NULL,				    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

*/

// --- BlackCat Type Cable  ---
#define BTDI     6       //0x40
#define BTCK     7       //0x80
#define BTMS     5       //0x20
#define BTDO     7       //0x80
#define BSRST_N  5       //0x20, pin7
#define BI_INV   0x80
#define BO_INV   0x00

// --- Xilinx Type Cable DLC5 ---
#define TDI     0       //0x01
#define TCK     1       //0x02
#define TMS     2       //0x04
#define TDO     4       //0x10
#define SRST_N  5       //0x20, pin7
#define I_INV   0x00
#define O_INV   0x20

// --- Wiggler Type Cable AMD5120---
#define WTDI      3       //0x08
#define WTCK      2       //0x04
#define WTMS      1       //0x02
#define WTDO      7       //0x80
#define WTRST_N   4       //0x10
#define WSRST_N   0       //0x01 , hardware inverted
#define WI_INV    0x80
#define WO_INV    0x10

// --- Cable Type ---
#define FT2232H   0
#define XILINX    1
#define BLACKCAT  2
#define WIGGLER   3
#define USBASP    4
#define STM32     5
#define JLINK     6
#define FT2232D   7

// --- Some EJTAG Instruction Registers ---
#define INSTR_EXTEST    0x00
#define INSTR_IDCODE    0x01
#define INSTR_SAMPLE    0x02
#define INSTR_IMPCODE   0x03
#define INSTR_ADDRESS   0x08
#define INSTR_DATA      0x09
#define INSTR_CONTROL   0x0A
#define INSTR_EJTAGALL  0x0B
#define INSTR_BYPASS    0xFF
#define LV_BYPASS               0xffffffff
#define CCJT_BYPASS             0xff
#define INSTR_EJTAGBOOT 0x0C
#define INSTR_NORMALBOOT 0x0D
// below from bcm1125H
#define INSTR_SYSCTRL   0x20
#define INSTR_TRACE     0x21
#define INSTR_PERF      0x22
#define INSTR_TRACECTRL 0x23
#define INSTR_WAFERID   0x24
#define INSTR_BCMU0O    0x26       //broadcom cpu0 observation chain
#define INSTR_BCMU0D    0x27       //broadcom cpu0 Debug chain
#define INSTR_BCMU0T    0x28       //broadcom cpu0 Test chain
#define INSTR_BCMSCANMC   0x34     //broadcom MC chain
#define INSTR_BCMSCANSCD  0x36     //broadcom SCD chain
#define INSTR_BCMSCANALL  0x38     //broadcom all Agent scan chain in
#define INSTR_BSRMODE   0x3A
#define INSTR_CLAMP     0x3C
#define INSTR_PRELOAD   0x3D
#define INSTR_INTEST    0x3E
#define INSTR_BCMBYPASS 0x3F

// --- Some EJTAG Control Register Bit Masks ---
#define TOF             (1 << 1 ) //ClkEn,  permit DCLK out
#define TIF             (1 << 2 )
#define BRKST           (1 << 3 ) //Indicate cpu in debug mode     
#define DINC            (1 << 4 ) //Increase Address auto on DMA
#define DLOCK           (1 << 5 ) //lock bus for DMA
#define DRWN            (1 << 9 ) // DMA R or W   
#define DERR            (1 << 10) //Indicate a error occur on DMA
#define DSTRT           (1 << 11) //DMA xfer start
#define JTAGBRK         (1 << 12) //generate a JTAG debug interupt
#define SETDEV          (1 << 14) //prob trap
#define PROBEN          (1 << 15) //cpu probe is enabled by deug support block
#define PRRST           (1 << 16) //Reset cpu
#define DMAACC          (1 << 17) //Request DMA
#define PRACC           (1 << 18) //PraCC interactive ext cpu probe.
#define PRNW            (1 << 19) //Indicate cpu R or W activity
#define PERRST          (1 << 20)
#define SYNC            (1 << 23)
#define DNM             (1 << 28)
#define ROCC            (1 << 31) 

/* define data type for MIPS32 */
#define MIPS_BYTE        0 
#define MIPS_HALFWORD    1 
#define MIPS_WORD        2 
#define MIPS_TRIPLEBYTE  3 

#define DMASZ(x)     ((x) << 7)        //make sz flag for DMA xfer

#define DMA_BYTE        (MIPS_BYTE       << 7)  //DMA tranfser size one byte       0x00000000
#define DMA_HALFWORD    (MIPS_HALFWORD   << 7)  //DMA transfer size double bytes   0x00000080
#define DMA_WORD        (MIPS_WORD       << 7)  //DMA transfer size four bytes     0x00000100 
#define DMA_TRIPLEBYTE  (MIPS_TRIPLEBYTE << 7)  //DMA transfer size three bytes    0x00000180

#define __LE			0		// Little Endian
#define __BE      1   // Big Endian

#define  size4K        0x1000
#define  size8K        0x2000
#define  size16K       0x4000
#define  size32K       0x8000
#define  size64K       0x10000
#define  size128K      0x20000
#define  size192K	     0x30000      // use for re-address tiny CFE and Kernel

#define  size1MB       0x100000
#define  size2MB       0x200000
#define  size4MB       0x400000
#define  size8MB       0x800000
#define  size16MB      0x1000000
#define  size32MB      0x2000000
#define  size64MB      0x4000000
#define  size128MB     0x8000000
#define  size256MB     0x10000000
#define  size512MB     0x20000000

#define  MEM_TOP			 0x20000000
#define  CFE_LEN       0x40000
#define  NVRAM_LEN     0x10000

#define  CMD_TYPE_BCS  0x01
#define  CMD_TYPE_SCS  0x02
#define  CMD_TYPE_AMD  0x03
#define  CMD_TYPE_SST  0x04
#define  CMD_TYPE_UND  0xFE

#define  SP_PPB        0x01      //Spansion Advanced Sector Protection


#define  STATUS_READY  0x0080    //DQ7


// EJTAG DEBUG Unit Vector on Debug Break
#define MIPS_DEBUG_VECTOR_ADDRESS           0xFF200200

// Our 'Pseudo' Virtual Memory Access Registers
#define MIPS_VIRTUAL_ADDRESS_ACCESS         0xFF200000        //not used by new pracc code
#define MIPS_VIRTUAL_DATA_ACCESS            0xFF200004

typedef struct _cable_prop_type
{
  DWORD feature;
  void (*close)(void);
  void (*test_reset)(void);
  int (*det_instr)(void);
  DWORD (*set_instr)(DWORD instr);
  DWORD (*ReadWriteData)(DWORD in_data);
  DWORD (*ReadData)(void);
  void (*WriteData)(DWORD in_data);
  DWORD (*ejtag_dma_read_x)(DWORD addr, int mode);
  void  (*ejtag_dma_write_x)(DWORD addr, DWORD data, int mode);
  DWORD (*ejtag_pracc_read_x)(DWORD addr, int mode);
  void  (*ejtag_pracc_write_x)(DWORD addr, DWORD data, int mode);

  int (*sflash_blkread) (DWORD Inaddr, DWORD* pbuff, int len);
  int (*sflash_blkwrite)(DWORD Inaddr, DWORD* pbuff, int len, int flpg_x8);

}cable_prop_type;

#define CBL_DMA_RD    (1UL<<0)
#define CBL_DMA_WR    (1UL<<1)
#define CBL_PRACC_RD  (1UL<<2)
#define CBL_PRACC_WR  (1UL<<3)


// --- public functions ---
#ifdef BRJMAIN

#ifdef WINDOWS_VERSION

__inline double dbl(LARGE_INTEGER x)
    {return (double)(x.HighPart * 4294967296.0 + x.LowPart);}
#endif
    
//----- public fn---------    
void ShowData(DWORD);
void ShowData_h(DWORD);
DWORD rev_endian_h(DWORD);
DWORD rev_endian(DWORD);
void iusleep(DWORD);

//------ private fn-------

static void chip_detect(void);
static void chip_shutdown(void);
static BYTE clockin(int, int);
static int det_instr(int iz_Total);
static int get_irlen_for_dev(int device_num);
static void define_block(DWORD, DWORD);
static DWORD ejtag_cmd_read(DWORD);
static void  ejtag_cmd_write(DWORD, DWORD);
static DWORD *ejtag_fix_writecode( DWORD, DWORD, int);
static DWORD *ejtag_fix_readcode( DWORD, int);
static void identify_flash_part();
static void lpt_closeport(void);
static void lpt_openport(void);
static void lpt_srst(void);
#ifdef WINDOWS_VERSION
static void	_outp64(WORD p, int d);
static BYTE	_inp64(WORD p);
#endif
static DWORD ReadData(void);
static DWORD ReadWriteData(DWORD);
static void run_backup(char*, DWORD, DWORD);
static void run_erase(char*, DWORD, DWORD);
static void run_flash(char*, DWORD, DWORD);
static DWORD set_instr(DWORD);
static void sflash_config(void);
static int  sflash_erase_area(DWORD, DWORD);
static void sflash_erase_block(DWORD);
static void sflash_probe(void);
static void sflash_reset(void);
static void sflash_write_word(DWORD, DWORD);
static void show_usage(void);
static void show_flashlist(void);
static void chip_filllist(void);
static void test_reset(void);
static void WriteData(DWORD);
static void ExecuteDebugModule(DWORD *);
static void check_ejtag_features(void);
static void sp_exit_cmdset(void);
static void sflash_erase_chip(void);
static void sp_check_ppb(void);
static void g_init_cpu();
static void g_init_dreg();
static void sp_unlock_bypass(void);
static void sp_unlock_bypass_reset(void);
static int usb_sflash_blkread(DWORD Inaddr, DWORD* pbuff, int len);
static int usb_sflash_blkwrite(DWORD Inaddr, DWORD* pbuff, int len);
static void usb_sflash_write_word(DWORD addr, DWORD data);
static void usb_dma_rdtraining(DWORD addr,DWORD length);
static void usb_dma_wrtraining(DWORD addr,DWORD length);
static DWORD ejtag_dma_read_x(DWORD addr, int mode);
static void ejtag_dma_write_x(DWORD addr, DWORD data, int mode);
static DWORD ejtag_pracc_read_x(DWORD addr, int mode);
static void ejtag_pracc_write_x(DWORD addr, DWORD data, int mode);
static DWORD ejtag_read_x(DWORD addr, int mode);
static void ejtag_write_x(DWORD addr, DWORD data,int mode);
static void sflash_write_word_x(DWORD addr, DWORD data);
static void sflash_write_x16(DWORD addr, DWORD data);
static void sflash_write_x8(DWORD addr, DWORD data);
static void sflash_poll_x16(DWORD addr, DWORD data);
static void sflash_poll_x8(DWORD addr, DWORD data);
static int getusb_setting(char* choice);

// **************** hugebird new code ************************

DWORD pracc_init_dreg[] = {
	             // #
               // # hugebird: PrAcc init data register
               // # 
	             // start:
	             // # keep $1 for all R/W operation
  0x3C01FF20,  // lui  $1,0xFF20  # li $1, MIPS_VIRTUAL_DATA_BASE
  0x00000000,
  0x1000FFFD,  // b  start
  0x00000000}; // nop

    
DWORD pracc_read_x32[] = {
               // #
               // # hugebird: PrAcc Read Word Routine (5 sets)
               // #
               // start:
               //
               // # Load R1 with the address of the pseudo-address register
 // 0x3C01FF20,// lui $1,  0xFF20
 // 0x34210000,// ori $1,  0x0000
               //
               // # Load R2 with the address %hi
  0x3C021F01,  // lui $2,  addr_hi   # p+0
               //
               // # Load R3 with the word @R2
  0x8C438100,  // lw $3, addr_lo($2) #p+1        <-   100011 00010 00011
               //
               // # Store the value into the pseudo-data register
  0xAC230004,  // sw $3, 4($1)       #p+2
               //
  0x00000000,  // nop   ,delay must have, wait data arrive.            
  0x1000FFFB,  // b  start
  0x00000000}; // nop

DWORD pracc_read_x16[] = {
               // #
               // # hugebird: PrAcc Read half Word Routine(5 sets)
               // #
               // start:
               //
               // # Load R1 with the address of the pseudo-address register
 // 0x3C01FF20,// lui $1,  0xFF20
 // 0x34210000,// ori $1,  0x0000
               //
               // # Load R2 with the address %hi
  0x3C021F01,  // lui $2,  addr_hi    # p+0
               //
               // # Load R3 with the word @R2
  0x94438100,  // lhu $3, addr_lo($2) # p+1         <- 100101 00010 00011
               //
               // # Store the value into the pseudo-data register
  0xAC230004,  // sw $3, 4($1)        # p+2         
               //
  0x00000000,  // nop   ,delay must have, wait wait data arrive            
  0x1000FFFB,  // b  start
  0x00000000}; // nop
  
DWORD pracc_read_x8[] = {
               // #
               // # hugebird: PrAcc Read byte Routine(5 sets)
               // #
               // start:
               //
               // # Load R1 with the address of the pseudo-address register
 // 0x3C01FF20,// lui $1,  0xFF20
 // 0x34210000,// ori $1,  0x0000
               //
               // # Load R2 with the address %hi
  0x3C021F01,  // lui $2,  addr_hi    # p+0
               //
               // # Load R3 with the byte @R2
  0x90438100,  // lbu $3, addr_lo($2) # p+1         <- 100100 00010 00011
               //  
               // # Store the value into the pseudo-data register
  0xAC230004,  // sw $3, 4($1)        # p+2         
               //
  0x00000000,  // nop   ,delay must have, wait wait data arrive            
  0x1000FFFB,  // b  start
  0x00000000}; // nop  
  
DWORD pracc_write_x8[] = {
               // #
               // # hugebird : PrAcc Write half Word Routine (4 sets)
               // #
               // start:
               //
               // # Load R1 with the address of the pseudo-address register
  //0x3C01FF20,  // lui $1,  0xFF20
  //0x34210000,  // ori $1,  0x0000
               //
               // # Load R2 with the address %hi
  0x3C021F01,  // lui $2,  addr_hi    # p+0
               //
               // # Load R3 with the data
  0x34030010,  // li $3, data         # p+1
               //
               // # Store the byte at @R2 (the address)
  0xA0438100,  // sb $3,  addr_lo($2) # p+2    <- 101000 00010 00011
               //
  0x1000FFFC,  // beq $0, $0, start
  0x00000000}; // nop  

DWORD pracc_write_x16[] = {
               // #
               // # hugebird : PrAcc Write half Word Routine (4 sets)
               // #
               // start:
               //
               // # Load R1 with the address of the pseudo-address register
  //0x3C01FF20,  // lui $1,  0xFF20
  //0x34210000,  // ori $1,  0x0000
               //
               // # Load R2 with the address %hi
  0x3C021F01,  // lui $2,  addr_hi    # p+0
               //
               // # Load R3 with the data
  0x34030010,  // li $3, data         # p+1
               //
               // # Store the half word at @R2 (the address)
  0xA4438100,  // sh $3,  addr_lo($2) # p+2    <- 101001 00010 00011
               //             
  0x1000FFFC,  // beq $0, $0, start
  0x00000000}; // nop

DWORD pracc_write_x32[] = {
               // #
               // # hugebird : PrAcc Write Word Routine (5 sets)
               // #
               // start:
               //
               // # Load R1 with the address of the pseudo-address register
  //0x3C01FF20,  // lui $1,  0xFF20
  //0x34210000,  // ori $1,  0x0000
               //
               // # Load R2 with the address %hi
  0x3C021F01,  // lui $2,  addr_hi        # p+0
               //
               // # Load R3 with the data
  0x3C030001,  // lui $3, data_hi         # p+1
  0x34631000,	 // ori	$3,$3,data_lo       # p+2      <-001101 00011 00011
               //
               // # Store the half word at @R2 (the address)
  0xAC438100,  // sw $3, addr_lo($2)      # p+3      
               //             
  0x1000FFFB,  // beq $0, $0, start
  0x00000000}; // nop
  
// **************************************************************************
//     hugeird : add cpu init configuration code below.
//               don't forget add point to CPU structure.
// **************************************************************************
DWORD i6358[] = {
               // #
               // # hugebird: initialize 6358 to fully access 16MB flash
               // #
               // start:
               //
  0x00000000,  // nop
  0x0000e021,  // move	gp,zero
  0x3c09ff40,  // lui	$9, 0xff40   
  0x3529000c,  // ori	$9, $9, 0x0c   
  0x4089b006,  // mtc0	$9, $22, 6  
               //
  0x00000000,  // nop
  0x1000FFF9,  // beq $0, $0, start
  0x00000000}; // nop

DWORD i5354[] = {
               // #
               // # hugebird: initialize 5354
               // #
               // start:
               //
  0x00000000,  // nop
  
  0x3c081fa0,   // li      $t0, 0x1FA0000C
  0x3508000c,   // mtc0    $t0, $22, 6
  0x4088b006,   //lui     $t1, 0x1FA0
  0x3c091fa0,   //lw      $t2, 0x1FA00014
  0x8d2a0014,
  0x3c01c000,   //lui     $1, 0xC000
  0x01415825,   //or      $t3, $t2, $1
  0xad2b0014,   //sw      $t3, 0x1FA00014
  0x3c08ff40,   //li      $t0, 0xFF40000C
  0x3508000c,
  0x4088b006,   //mtc0    $t0, $22, 6
               //
  0x00000000,  // nop
  0x1000FFF2,  // beq $0, $0, start
  0x00000000}; // nop

#else   //BRJMAIN

#ifdef WINDOWS_VERSION
extern void iusleep(DWORD);
#endif

#endif   //BRJMAIN

// **************************************************************************
// End of File
// **************************************************************************
