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

#if ( defined(_MSC_VER)  )
#define WINDOWS_VERSION             //windows make with: cl zjtag.c xxx.c
#endif                    

#define BRJMAIN

#ifdef WINDOWS_VERSION

#define _OUTP(d) if (is_os64) _outp64(lpt_port,d); \
	                 else       _outp(lpt_port, d)
#define _INP(d)   d = is_os64?_inp64(lpt_port+1):_inp(lpt_port+1)

#else   //For linux

#define _OUTP(d)  ioctl(pfd, PPWDATA, &d)
#define _INP(d)		ioctl(pfd, PPRSTATUS, &d)

#endif

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef WINDOWS_VERSION
// extern int getch(void);
#endif

#include "zjtag.h"
#include "cfi.h"

#define BigEndian		(endian == __BE)
//#define DEBUGMSG1 0
//#define DEBUGMSG  0
#define DEBUGFLASH  0

//#if (DEBUGMSG1 || DEBUGMSG ||DEBUGFLASH )
#define DBG(x)     printf x
//#else
//#define DBG(x)
//#endif

//---------------------------------------------

//cpu
int issue_reset      = 1;
int issue_enable_mw  = 1;
int issue_watchdog   = 1;
int issue_break      = 1;
int endian     	     = __LE;
int force_endian = 0;
int skipdetect       = 0;
int issue_initcpu    = 0;

//ejtag access
int force_dma        = 0;
int force_nodma      = 0;
int USE_DMA          = 0;
int ejtag_speed      = 0;      //0: fast clock; 1: low clock, 500KHz
int no_mpidetect     = 0;
int wait_break       = 0;
int ejtag_version    = 0;
int prg_x8           = 0;    // 1: flash program in x8 mode , 0: in x16 mode
int issue_srst       = 0;

#define PRG_X8     (prg_x8==1)
#define PGMASK     (PRG_X8?0xFFFFFFFF:0x00FF00FF)
#define PGMODE     (PRG_X8?MIPS_BYTE:MIPS_HALFWORD)
#define PGBW       (PRG_X8?0:1)
#define AMDUL1     (0xAAA)
#define AMDUL2     (PRG_X8?0x555:0x554)
#define AMDCFI     (0xAA)
	
int ad=0;
int sz = 0;	

//flash
int issue_erase      = 1;
int issue_chip_erase = 0;
int issue_clear_ppb  = 0;
int issue_show_ppb   = 0;
int PPB_type         = 0;
int bypass           = 0;
int issue_cfi_qry    = 1;
int force_noflip     = 0;     //use CFI erase block definition as is, no flipping
int force_flip       = 0;
int safemode         = 0;     //safe access routine

//file other
int is_probe         = 0;
int issue_timestamp  = 1;
int probe_custom     = 0; 
int silent_mode      = 1;
int force_align      = 0;
int check_align      = 0;
int showgpio         = 0;
int custom_options   = 0;
int is_os64          = 0;    //0:win32; 1:win64
int DEBUGMSG1 = 0;
int DEBUGMSG  = 0;


//--------------------------------------------------
int pfd;
int cable_type       = FT2232H;
int cable_intf       = 0;         //1 usb
int cable_id         = 0;
// added by tumpa
int delay_in_ms		 = 50; // default to delay 50ms in dma write

WORD lpt_port	       = PORT378;

DWORD selected_window  = 0;
DWORD selected_start   = 0;
DWORD selected_length  = 0;


int instrlen         = 0;
int selected_fc      = 0;
DWORD * init_code = NULL;


//--------USB Jtager configure--------------------

DWORD LL1 = 0;   // io freq divisor
BYTE  LL2 = 0;   // usb latency
DWORD LL3 = 0xFFFF;   // dma xfer timeout
DWORD LL4 = 0xFFFF;   // flash program polling timeout
DWORD LL5 = 0;   // dma bulk xfer queue size
DWORD LL6 = 0;
DWORD LL7 = 0;   // dma bulk read block size
DWORD LL8 = 0;   // dma bulk write x16 mode block size
DWORD LL9 = 0;   // dma bulk write x8 mode block size
DWORD USBID = 0; 
DWORD GG1 = 0;     //gpio low setting
DWORD GG2 = 0;     //gpio high setting

int set_LL3          = 0;
int flsPrgTimeout    = 0;

//-------------------------------------------------

char     flash_part[128];
DWORD    flash_size = 0;

int      block_total = 0;
DWORD    block_addr = 0;
DWORD    blocks[1025];
DWORD    cmd_type = 0;

char     AREA_NAME[128];
DWORD    AREA_START;
DWORD    AREA_LENGTH;
DWORD    FLASH_MEMORY_START;
DWORD	   AREA_OFFSET;
DWORD    mfrid;
DWORD    venid;
DWORD    devid;
DWORD    mpi_base = 0;
DWORD    chip_features = 0;

DWORD    data_register;
DWORD    address_register;
static DWORD ctrl_reg;
int instruction_length;
//---------------------------------------------------

cfi_qry_type  cfi_qry_data;
cfi_pri_type  cfi_pri_data;
cfi_pri_amdstd_type cfi_amd_pri_data;
cfi_pri_atmel_type cfi_at_pri_data;
cable_prop_type *cable_prop, cbl_prop;


typedef struct _cable_list_type
{
  int   id;
	DWORD subid;        // parallel cable type, or for same driver but diff config
  DWORD devid;        // usb vid:pid
  DWORD is_bulk;      // allow bulk r/w, e.g. usb
  int (*init)(void *);  // fill cable op call, init device etc.
  char * name;
}cable_list_type;

extern int ftinit(void *);
extern int jlinit(void *);
extern int uinit(void *);
extern int stinit(void *);
cable_list_type cable_list[]={
 {0, FT2232H , 0x04038A98, 1, ftinit ,"TIAO USB Multi-Protocol Adapter (TUMPA)" },
 {1, XILINX  , 0         , 0, 0      ,"TIAO All In One/20/10 PIN JTAG cable (XILINX)" },  
 {2, BLACKCAT, 0         , 0, 0      ,"TIAO Blackcat JTAG cable (for cable modems)" },  
 {3, FT2232H , 0x04038A99, 1, ftinit ,"TIAO USB Multi-Protocol Adapter Lite (TUMPA-Lite)" },
 {4, XILINX  , 0         , 0, 0      ,"Parallel port type (DLC5/WIGGLER)" },
 {5, FT2232D , 0x14575118, 1, ftinit ,"FT2232C/D based USB cable(OpenMoko,JTAGkey,OpenJTAG)" },
 {6, JLINK   , 0x13660101, 1, jlinit ,"SEGGER J-Link EMU"     },
 {7, USBASP  , 0x16C005DF, 1, uinit  ,"HID-BRJTAG v1.xx(USBASP)" },
 {8, STM32   , 0x04835750, 1, stinit ,"HID-BRJTAG v2.xx(STM32F10x/SAM7S)" },
 
 {-1,0,0,0,0,0}};       //list end with id =-1

typedef struct _cpumfr_type {
	DWORD   id;
	char *  name;
	} cpumfr_type;


#define  C_BRCM   0x17E
#define  C_TI1    0x00E
#define  C_TI2    0x02E
#define  C_PMCS   0x2E0
#define  C_INTL   0x012
#define  C_ARM    0xF0E
#define  C_IDT    0x066 // 0x067 - 1100111 - Integrated Device Technology, Inc.
#define  C_AMD    0x000 // 0x001 - 0000001 - Advanced Micro Devices, Inc.
#define  C_CNXT   0x026 // 0x027 - 0100111 - Conexant Systems, Inc.
#define  C_STM    0x040 // 0x041 - 1000001 - STMicroelectronics (SGS-Thomson)
#define  C_LXRA   0x00C // 0x00D - 0001101 - Lexra, Inc. (Defunct)


cpumfr_type cpu_mfr_list[]={
 {0xFFE       ,"Unknown"      },
 {C_BRCM      ,"Broadcom"     },
 {C_TI1       ,"TI"           },
 {C_TI2       ,"TI"           },
 {C_PMCS      ,"PMC-Serria"   },
 {C_INTL      ,"INTEL"        },
 {C_ARM       ,"ARM"          }, 
 {C_IDT       ,"IDT"          },
 {C_AMD       ,"AMD"          },
 {C_CNXT      ,"Conexant"     },
 {C_STM       ,"SGS-Thomson"  },
 {C_LXRA      ,"Lexra"        },
 {0,0}};

typedef struct _processor_chip_type
  {
    DWORD          chip_id;        // Processor Chip ID  IDCODE:0x0|4710|17F
    DWORD          chip_mfrid;       // Processor Chip manufiacture id
//    DWORD        chip_rev;       // Processor Chip reversion
    
    int            instr_length;   // EJTAG Instruction Length
    int						 chip_endn;  		// Chip endian
    DWORD*         init_code;      // Initialize code start
    DWORD          mpi_base;       // mpi register base
    cpumfr_type*   mfrdscr_idx;    // Procesor vendor description index;
    char*          chip_descr;     // Processor Chip Description
    
  } processor_chip_type; 	

#define GETDEVID(x)     (( x >> 12 )& 0xFFFF ) 
#define GETREVID(x)     (( x >> 28 )>0 ? ( x >> 28 ):1 )
#define GETMFRID(x)     (  x        & 0x0FFE )

processor_chip_type  processor_chip_list[] =
{
/* ********* Broadcom Wifi Chips ***********************************/    
  { 0x3349,  C_BRCM, 5, __BE,     0,          0, 0,"BCM3349"       },
  { 0x4710,  C_BRCM, 5, __LE,     0,          0, 0,"BCM4702"       },
  { 0x4704,  C_BRCM, 8, __LE,     0,          0, 0,"BCM4704"       }, // BCM4704 chip (used in the WRTSL54GS units)
  { 0x00C3,  C_BRCM, 7, __LE,     0,          0, 0,"BCM4706"       }, // BCM4706 Not Completely Verified Yet
  { 0x4712,  C_BRCM, 8, __LE,     0,          0, 0,"BCM4712"       },
  { 0x4716,  C_BRCM, 5, __LE,     0,          0, 0,"BCM4716"       },
  { 0x00C8,  C_BRCM, 5, __LE,     0,          0, 0,"BCM4716"       }, // BCM4716 Not Completely Verified Yet
  { 0x4785,  C_BRCM, 8, __LE,     0,          0, 0,"BCM4705|4785"  }, // Tornado WRT350N
  { 0x5350,  C_BRCM, 8, __LE,     0,          0, 0,"BCM5350"       },
  { 0x5352,  C_BRCM, 8, __LE,     0,          0, 0,"BCM5352"       },
  { 0x5354,  C_BRCM, 8, __LE, i5354,          0, 0,"BCM5354"       }, // Tornado - WRT54G GV8/GSV7
  { 0x5365,  C_BRCM, 8, __LE,     0,          0, 0,"BCM5365"       }, // BCM5365 Not Completely Verified Yet
/* ********* Broadcom DSL Chips ***********************************/
  { 0x6345,  C_BRCM, 5, __BE,     0,          0, 0,"BCM6345"       }, // BCM6345 Not Completely Verified Yet
  { 0x6338,  C_BRCM, 5, __BE,     0, 0xfffe3160, 0,"BCM6338"       }, //Fully Tested
  { 0x6348,  C_BRCM, 5, __BE,     0, 0xfffe2000, 0,"BCM6348"       }, //Fully Tested
  { 0x6358,  C_BRCM, 5, __BE, i6358, 0xfffe1000, 0,"BCM6358"       }, //Fully Tested
  { 0x6368,  C_BRCM, 5, __BE,     0, 0xb0001000, 0,"BCM6368"       },
/* ********* Broadcom PON Chips ************************************/  
  { 0x6816,  C_BRCM, 5, __BE,     0, 0xb0002000, 0,"BCM6816"       },  
/* ********* VSAT/HDV/DVB Chips ************************************/
  { 0x7401,  C_BRCM, 5, __BE,     0, 0xb0001000, 0,"BCM7401"       }, //20100904 
/* ********* Non Broadcom Chips ************************************/
  { 0x0200,  C_PMCS, 5, __LE,     0,          0, 0,"BRECIS MSP2007-CA-A1" },   // BRECIS chip - Not Completely Verified Yet
  { 0x0001,   C_TI1, 5, __BE,     0,          0, 0,"TNETD7300GDU(AR7WRD)" },   // TI AR7WRD Only Partially Verified
  { 0xB52D,   C_TI2, 5, __LE,     0,          0, 0,"TNETV1060GDW"         },   // Fox WRTP54G
/* ********* from UrJTAG data files ********************************/
  { 0x1250,  C_BRCM, 6, __LE,     0,          0, 0,"BCM1250"       },
  { 0x3310,  C_BRCM, 5, __LE,     0,          0, 0,"BCM3310"       },
  { 0x5421,  C_BRCM, 3, __LE,     0,          0, 0,"BCM5421S"      },
  { 0xB6C6,   C_TI2, 5, __LE,     0,          0, 0,"TNETV1061"     },
/* ********* from bsdl.info ****************************************/
  { 0x5482,  C_BRCM,11, __LE,     0,          0, 0,"BCM5482"       },
  { 0x5498,  C_BRCM,11, __LE,     0,          0, 0,"BCM54980"      },
  { 0xB639,  C_BRCM,32, __LE,     0,          0, 0,"BCM56639"      },
  { 0x6421,  C_BRCM, 3, __BE,     0,          0, 0,"BCM6421"       },
  { 0x6510,  C_BRCM, 5, __BE,     0,          0, 0,"BCM6510"       },
/* ********* from TJTAG v3.0.1 *************************************/
  { 0x3345,  C_BRCM, 5, __LE,     0,          0, 0,"BCM3345 KPB"   },   // Eko QAMLink  BCM3345 KPB SB4200
  { 0x4321,  C_BRCM, 5, __LE,     0,          0, 0,"BCM4321/L RADIO STOP" },   // Radio JP3 on a WRT300N V1.1 / EKO Radio on WRT300n
  { 0x9277,  C_INTL, 7, __LE,     0,          0, 0,"XScale IXP42X 266mhz" },            // GW2348-2 Eko Gateworks Avila GW234X (IXP42X 266MHz) BE
  { 0x9275,  C_INTL, 7, __LE,     0,          0, 0,"XScale IXP42X 400mhz" },
  { 0x9274,  C_INTL, 7, __LE,     0,          0, 0,"XScale IXP42X 533mhz" },
  { 0x0217,   C_IDT, 5, __LE,     0,          0, 0,"Linkstation 2 with RISC K4C chip" }, // Not verified
  { 0x0000,   C_AMD, 5, __LE,     0,          0, 0,"Atheros AR531X/231X"  },             // WHR-HP-AG108
  { 0x0940,  C_CNXT, 4, __LE,     0,          0, 0,"ARM 940T"      }, // Eko  Linksys BEFSX41
  { 0x7926,   C_STM, 4, __LE,     0,          0, 0,"Marvell Feroceon 88F5181" },
  { 0x4380,  C_LXRA, 5, __LE,     0,          0, 0,"LX4380"        },
/* ****** Add your self CPU before this line ****/
  { 0, 0, 0, 0 }
};


typedef struct _flash_area_type
  {
    DWORD        chip_size;
    char*               area_name;
    DWORD        area_start;
    DWORD        area_length;
  } flash_area_type;


flash_area_type  flash_area_list[] =
{
  //---------   ----------     -----------  ------------
  //chip_size   area_name      area_start   area_length
  //                             offset 
  //---------   ----------     -----------  ------------
  { size1MB,    "CFE",         0x00000000,  0x40000 },
  { size2MB,    "CFE",         0x00000000,  0x40000 },
  { size4MB,    "CFE",         0x00000000,  0x40000 },
  { size8MB,    "CFE",         0x00000000,  0x40000 },
  { size16MB,   "CFE",         0x00000000,  0x40000 },
  // for tiny CFE
  { size1MB,    "TFE",         0x00000000,  0x10000 },
  { size2MB,    "TFE",         0x00000000,  0x10000 },    //Tiny CFE usually for BCM ADSL device 
  { size4MB,    "TFE",         0x00000000,  0x10000 },    //keep at least 64KB in size,
  { size8MB,    "TFE",         0x00000000,  0x10000 },    //Tiny CFE also need keep 1 bottom flash sector size.
  { size16MB,   "TFE",         0x00000000,  0x10000 },    

  { size1MB,    "KERNEL",      0x00040000,  0x0B0000 },
  { size2MB,    "KERNEL",      0x00040000,  0x1B0000 },
  { size4MB,    "KERNEL",      0x00040000,  0x3B0000 },
  { size8MB,    "KERNEL",      0x00040000,  0x7A0000 },
  { size16MB,   "KERNEL",      0x00040000,  0x7A0000 },

  { size1MB,    "NVRAM",       0x000F0000,  0x10000 },
  { size2MB,    "NVRAM",       0x001F0000,  0x10000 },
  { size4MB,    "NVRAM",       0x003F0000,  0x10000 },     //default NVRAM size 64KB
  { size8MB,    "NVRAM",       0x007E0000,  0x20000 },     //NVRAM need keep at least 1 top flash sector size
  { size16MB,   "NVRAM",       0x007E0000,  0x20000 },   

  { size1MB,    "WHOLEFLASH",  0x00000000,  0x100000 },
  { size2MB,    "WHOLEFLASH",  0x00000000,  0x200000 },
  { size4MB,    "WHOLEFLASH",  0x00000000,  0x400000 },
  { size8MB,    "WHOLEFLASH",  0x00000000,  0x800000 },
  { size16MB,   "WHOLEFLASH",  0x00000000, 0x1000000 },

  { size1MB,    "BSP",         0x00000000,  0x50000 },
  { size2MB,    "BSP",         0x00000000,  0x50000 },
  { size4MB,    "BSP",         0x00000000,  0x50000 },
  { size8MB,    "BSP",         0x00000000,  0x50000 },
  { size16MB,   "BSP",         0x00000000,  0x50000 },

  { 0, 0, 0, 0 }
};


typedef struct _flash_chip_type
  {
    DWORD        venid;          // Manufacturer Id
    DWORD        devid;          // Device Id
    DWORD        flash_size;     // Total size in MBytes
    DWORD        cmd_type;       // Device CMD TYPE
    DWORD        prot_type;      // Block Protection Type, 
    char*        flash_part;     // Flash Chip Description
    DWORD        region1_num;    // Region 1 block count
    DWORD        region1_size;   // Region 1 block size
    DWORD        region2_num;    // Region 2 block count
    DWORD        region2_size;   // Region 2 block size
    DWORD        region3_num;    // Region 3 block count
    DWORD        region3_size;   // Region 3 block size
    DWORD        region4_num;    // Region 4 block count
    DWORD        region4_size;   // Region 4 block size
  } flash_chip_type;


flash_chip_type  flash_chip_list[] =
{
  /** for CFI auto detection **/    	
  { 0xAAAA, 0xAAAA, 0, CMD_TYPE_UND, 0, "CFI Compatiable"  ,0,0  ,0,0  ,0,0  ,0,0  },
  /* AMD, Spansion */
  { 0x00C2, 0x22DA, size1MB, CMD_TYPE_AMD, 0, "MX29LV800BTC 512kx16 TopB  (1MB)"   ,15,size64K,    1,size32K,    2,size8K,   1,size16K  },
  { 0x00C2, 0x225B, size1MB, CMD_TYPE_AMD, 0, "MX29LV800BBC 512kx16 BotB  (1MB)"   ,1,size16K,     2,size8K,     1,size32K,  15,size64K },

  { 0x0001, 0x2249, size2MB, CMD_TYPE_AMD, 0, "AMD 29lv160DB 1Mx16 BotB   (2MB)"   ,1,size16K,    2,size8K,     1,size32K,  31,size64K },
  { 0x0001, 0x22c4, size2MB, CMD_TYPE_AMD, 0, "AMD 29lv160DT 1Mx16 TopB   (2MB)"   ,31,size64K,   1,size32K,    2,size8K,   1,size16K  },
  { 0x007f, 0x2249, size2MB, CMD_TYPE_AMD, 0, "EON EN29LV160A 1Mx16 BotB  (2MB)"   ,1,size16K,    2,size8K,     1,size32K,  31,size64K },
  { 0x007f, 0x22C4, size2MB, CMD_TYPE_AMD, 0, "EON EN29LV160A 1Mx16 TopB  (2MB)"   ,31,size64K,   1,size32K,    2,size8K,   1,size16K  },
  { 0x0004, 0x2249, size2MB, CMD_TYPE_AMD, 0, "MBM29LV160B 1Mx16 BotB     (2MB)"   ,1,size16K,    2,size8K,     1,size32K,  31,size64K },
  { 0x0004, 0x22c4, size2MB, CMD_TYPE_AMD, 0, "MBM29LV160T 1Mx16 TopB     (2MB)"   ,31,size64K,   1,size32K,    2,size8K,   1,size16K  },
  { 0x00C2, 0x2249, size2MB, CMD_TYPE_AMD, 0, "MX29LV161B/160B 1Mx16 BotB (2MB)"   ,1,size16K,    2,size8K,     1,size32K,  31,size64K },
  { 0x00C2, 0x22c4, size2MB, CMD_TYPE_AMD, 0, "MX29LV161T/160T 1Mx16 TopB (2MB)"   ,31,size64K,   1,size32K,    2,size8K,   1,size16K  },
  { 0x00C2, 0x0014, size2MB, CMD_TYPE_AMD, 0, "MX25L160A                  (2MB)"   ,32,size64K,   0,0,          0,0,        0,0        }, /* new */
  { 0x00EC, 0x2275, size2MB, CMD_TYPE_AMD, 0, "K8D1716UTC  1Mx16 TopB     (2MB)"   ,31,size64K,    8,size8K,     0,0,        0,0       },
  { 0x00EC, 0x2277, size2MB, CMD_TYPE_AMD, 0, "K8D1716UBC  1Mx16 BotB     (2MB)"   ,8,size8K,      31,size64K,   0,0,        0,0       },
  { 0x0020, 0x2249, size2MB, CMD_TYPE_AMD, 0, "ST M29W160EB 1Mx16 BotB    (2MB)"   ,1,size16K,    2,size8K,     1,size32K,  31,size64K },
  { 0x0020, 0x22c4, size2MB, CMD_TYPE_AMD, 0, "ST M29W160ET 1Mx16 TopB    (2MB)"   ,31,size64K,   1,size32K,    2,size8K,   1,size16K  },

  { 0x0001, 0x227E, size4MB, CMD_TYPE_AMD, 0, "AMD 29lv320MT 2Mx16 TopB   (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
  { 0x0001, 0x2200, size4MB, CMD_TYPE_AMD, 0, "AMD 29lv320MB 2Mx16 BotB   (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
  { 0x0001, 0x2201, size4MB, CMD_TYPE_AMD, 0, "AMD 29lv320MT 2Mx16 TopB   (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
  { 0x0001, 0x22f9, size4MB, CMD_TYPE_AMD, 0, "AMD 29lv320DB 2Mx16 BotB   (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
  { 0x0001, 0x22f6, size4MB, CMD_TYPE_AMD, 0, "AMD 29lv320DT 2Mx16 TopB   (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
  { 0x0098, 0x009C, size4MB, CMD_TYPE_AMD, 0, "TC58FVB321 2Mx16 BotB      (4MB)"   ,1,size16K,    2,size8K,     1,size32K,  63,size64K },
  { 0x0098, 0x009A, size4MB, CMD_TYPE_AMD, 0, "TC58FVT321 2Mx16 TopB      (4MB)"   ,63,size64K,   1,size32K,    2,size8K,   1,size16K  },
  { 0x001F, 0x00C0, size4MB, CMD_TYPE_AMD, 0, "AT49BV/LV16X 2Mx16 BotB    (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
  { 0x001F, 0x00C2, size4MB, CMD_TYPE_AMD, 0, "AT49BV/LV16XT 2Mx16 TopB   (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
  { 0x0004, 0x2253, size4MB, CMD_TYPE_AMD, 0, "MBM29DL323BE 2Mx16 BotB    (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
  { 0x0004, 0x2250, size4MB, CMD_TYPE_AMD, 0, "MBM29DL323TE 2Mx16 TopB    (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
  { 0x0004, 0x22F9, size4MB, CMD_TYPE_AMD, 0, "MBM29LV320BE 2Mx16 BotB    (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
  { 0x0004, 0x22F6, size4MB, CMD_TYPE_AMD, 0, "MBM29LV320TE 2Mx16 TopB    (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
  { 0x00C2, 0x00A8, size4MB, CMD_TYPE_AMD, 0, "MX29LV320AB 2Mx16 BotB     (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
  { 0x00C2, 0x00A7, size4MB, CMD_TYPE_AMD, 0, "MX29LV320AT 2Mx16 TopB     (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
  { 0x00C2, 0x22A8, size4MB, CMD_TYPE_AMD, 0, "MX29LV320AB/BB 2Mx16 BotB  (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
  { 0x00C2, 0x22A7, size4MB, CMD_TYPE_AMD, 0, "MX29LV320AT/BT 2Mx16 TopB  (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
  { 0x00EC, 0x22A2, size2MB, CMD_TYPE_AMD, 0, "K8D3216UBC  2Mx16 BotB     (4MB)"   ,8,size8K,      63,size64K,   0,0,        0,0       },
  { 0x00EC, 0x22A0, size2MB, CMD_TYPE_AMD, 0, "K8D3216UTC  2Mx16 TopB     (4MB)"   ,63,size64K,    8,size8K,     0,0,        0,0       },
  { 0x0020, 0x22CB, size4MB, CMD_TYPE_AMD, 0, "ST 29w320DB 2Mx16 BotB     (4MB)"   ,1,size16K,    2,size8K,     1,size32K,  63,size64K },
  { 0x0020, 0x22CA, size4MB, CMD_TYPE_AMD, 0, "ST 29w320DT 2Mx16 TopB     (4MB)"   ,63,size64K,   1,size32K,    2,size8K,   1,size16K  },
  { 0x0020, 0x225D, size4MB, CMD_TYPE_AMD, 0, "ST M29DW324DB 2Mx16 BotB   (4MB)"   ,8,size8K,   63,size64K,     0,0,        0,0        }, /* new */
  { 0x0020, 0x225C, size4MB, CMD_TYPE_AMD, 0, "ST M29DW324DT 2Mx16 TopB   (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        }, /* new */
  { 0x00DA, 0x22BA, size4MB, CMD_TYPE_AMD, 0, "W19B(L)320ST   2Mx16 TopB  (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        }, /* new */
  { 0x00DA, 0x222A, size4MB, CMD_TYPE_AMD, 0, "W19B(L)320SB   2Mx16 BotB  (4MB)"   ,8,size8K,   63,size64K,     0,0,        0,0        }, /* new */


  { 0x0098, 0x0057, size8MB, CMD_TYPE_AMD, 0, "TC58FVM6T2A  4Mx16 TopB    (8MB)"   ,127,size64K,   8,size8K,     0,0,        0,0       }, /* new */
  { 0x0098, 0x0058, size8MB, CMD_TYPE_AMD, 0, "TC58FVM6B2A  4Mx16 BopB    (8MB)"   ,8,size8K,   127,size64K,     0,0,        0,0       }, /* new */
  { 0x00EC, 0x22E0, size8MB, CMD_TYPE_AMD, 0, "K8D6316UTM  4Mx16 TopB     (8MB)"   ,127,size64K,    8,size8K,     0,0,        0,0      }, /* new */
  { 0x00EC, 0x22E2, size8MB, CMD_TYPE_AMD, 0, "K8D6316UBM  4Mx16 BotB     (8MB)"   ,8,size8K,      127,size64K,   0,0,        0,0      }, /* new */

  /* BSC */
  { 0x0089, 0x8891, size2MB, CMD_TYPE_BCS, 0, "Intel 28F160B3 1Mx16 BotB  (2MB)"   ,8,size8K,     31,size64K,   0,0,        0,0        },
  { 0x0089, 0x8890, size2MB, CMD_TYPE_BCS, 0, "Intel 28F160B3 1Mx16 TopB  (2MB)"   ,31,size64K,   8,size8K,     0,0,        0,0        },
  { 0x0089, 0x88C3, size2MB, CMD_TYPE_BCS, 0, "Intel 28F160C3 1Mx16 BotB  (2MB)"   ,8,size8K,     31,size64K,   0,0,        0,0        },
  { 0x0089, 0x88C2, size2MB, CMD_TYPE_BCS, 0, "Intel 28F160C3 1Mx16 TopB  (2MB)"   ,31,size64K,   8,size8K,     0,0,        0,0        },

  { 0x0089, 0x8897, size4MB, CMD_TYPE_BCS, 0, "Intel 28F320B3 2Mx16 BotB  (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
  { 0x0089, 0x8896, size4MB, CMD_TYPE_BCS, 0, "Intel 28F320B3 2Mx16 TopB  (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
  { 0x0089, 0x88C5, size4MB, CMD_TYPE_BCS, 0, "Intel 28F320C3 2Mx16 BotB  (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
  { 0x0089, 0x88C4, size4MB, CMD_TYPE_BCS, 0, "Intel 28F320C3 2Mx16 TopB  (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
  { 0x00b0, 0x00e3, size4MB, CMD_TYPE_BCS, 0, "Sharp 28F320BJE 2Mx16 BotB (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },

  { 0x0089, 0x8899, size8MB, CMD_TYPE_BCS, 0, "Intel 28F640B3 4Mx16 BotB  (8MB)"   ,8,size8K,     127,size64K,  0,0,        0,0        },
  { 0x0089, 0x8898, size8MB, CMD_TYPE_BCS, 0, "Intel 28F640B3 4Mx16 TopB  (8MB)"   ,127,size64K,  8,size8K,     0,0,        0,0        },
  { 0x0089, 0x88CD, size8MB, CMD_TYPE_BCS, 0, "Intel 28F640C3 4Mx16 BotB  (8MB)"   ,8,size8K,     127,size64K,  0,0,        0,0        },
  { 0x0089, 0x88CC, size8MB, CMD_TYPE_BCS, 0, "Intel 28F640C3 4Mx16 TopB  (8MB)"   ,127,size64K,  8,size8K,     0,0,        0,0        },

  /* SCS */
  { 0x00b0, 0x00d0, size2MB, CMD_TYPE_SCS, 0, "Intel 28F160S3/5 1Mx16     (2MB)"   ,32,size64K,   0,0,          0,0,        0,0        },

  { 0x0089, 0x0016, size4MB, CMD_TYPE_SCS, 0, "Intel 28F320J3 2Mx16       (4MB)"   ,32,size128K,  0,0,          0,0,        0,0        },
  { 0x0089, 0x0014, size4MB, CMD_TYPE_SCS, 0, "Intel 28F320J5 2Mx16       (4MB)"   ,32,size128K,  0,0,          0,0,        0,0        },
  { 0x00b0, 0x00d4, size4MB, CMD_TYPE_SCS, 0, "Intel 28F320S3/5 2Mx16     (4MB)"   ,64,size64K,   0,0,          0,0,        0,0        },

  { 0x0089, 0x0017, size8MB, CMD_TYPE_SCS, 0, "Intel 28F640J3 4Mx16       (8MB)"   ,64,size128K,  0,0,          0,0,        0,0        },
  { 0x0089, 0x0015, size8MB, CMD_TYPE_SCS, 0, "Intel 28F640J5 4Mx16       (8MB)"   ,64,size128K,  0,0,          0,0,        0,0        },

  { 0x0089, 0x0018, size16MB, CMD_TYPE_SCS, 0, "Intel 28F128J3 8Mx16      (16MB)"  ,128,size128K, 0,0,          0,0,        0,0        },


  /* SST */
  { 0x00BF, 0x234B, size4MB, CMD_TYPE_SST, 0, "SST39VF1601 1Mx16 BotB     (2MB)"   ,32,size64K,    0,0,          0,0,        0,0       },
  { 0x00BF, 0x234A, size4MB, CMD_TYPE_SST, 0, "SST39VF1602 1Mx16 TopB     (2MB)"   ,32,size64K,    0,0,          0,0,        0,0       },
  { 0x00BF, 0x234F, size4MB, CMD_TYPE_AMD, 0, "SST39VF1601C 1Mx16 BotB    (2MB)"   ,1,size16K,    2,size8K,  1,size32K,    31,size64K  },
  { 0x00BF, 0x234E, size4MB, CMD_TYPE_AMD, 0, "SST39VF1602C 1Mx16 TopB    (2MB)"   ,31,size64K,  1,size32K,  2,size8K,     1,size16K   },
  { 0x00BF, 0x734B, size4MB, CMD_TYPE_SST, 0, "SST39VF1601E 1Mx16 BotB    (2MB)"   ,32,size64K,    0,0,          0,0,        0,0       },
  { 0x00BF, 0x734A, size4MB, CMD_TYPE_SST, 0, "SST39VF1602E 1Mx16 TopB    (2MB)"   ,32,size64K,    0,0,          0,0,        0,0       },

  { 0x00BF, 0x235B, size4MB, CMD_TYPE_SST, 0, "SST39VF3201 2Mx16 BotB     (4MB)"   ,64,size64K,   0,0,          0,0,        0,0       },
  { 0x00BF, 0x235A, size4MB, CMD_TYPE_SST, 0, "SST39VF3202 2Mx16 TopB     (4MB)"   ,64,size64K,   0,0,          0,0,        0,0       },
  { 0x00BF, 0x235D, size4MB, CMD_TYPE_AMD, 0, "SST39VF3201B 2Mx16 BotB    (4MB)"   ,64,size64K,   0,0,          0,0,        0,0       },
  { 0x00BF, 0x235C, size4MB, CMD_TYPE_AMD, 0, "SST39VF3202B 2Mx16 TopB    (4MB)"   ,64,size64K,   0,0,          0,0,        0,0       },


  { 0x00BF, 0x236B, size4MB, CMD_TYPE_SST, 0, "SST39VF6401 4Mx16 BotB     (8MB)"   ,128,size64K,   0,0,          0,0,        0,0       },
  { 0x00BF, 0x236A, size4MB, CMD_TYPE_SST, 0, "SST39VF6402 4Mx16 TopB     (8MB)"   ,128,size64K,   0,0,          0,0,        0,0       },
  { 0x00BF, 0x236D, size4MB, CMD_TYPE_AMD, 0, "SST39VF6401B 4Mx16 BotB    (8MB)"   ,128,size64K,   0,0,          0,0,        0,0       },
  { 0x00BF, 0x236C, size4MB, CMD_TYPE_AMD, 0, "SST39VF6402B 4Mx16 TopB    (8MB)"   ,128,size64K,   0,0,          0,0,        0,0       },


  /* New Added IC */
 
  //  Spansion
  { 0x017E, 0x1A00, size4MB, CMD_TYPE_AMD, SP_PPB,  "Spansion S29GL032MR4 BotB  (4MB)"    ,8,size8K,     63,size64K,   0,0,        0,0  },
  { 0x017E, 0x1A01, size4MB, CMD_TYPE_AMD, SP_PPB,  "Spansion S29GL032MR3 TopB  (4MB)"   ,63,size64K,     8,size8K,    0,0,        0,0  },
  { 0x017E, 0x1000, size8MB, CMD_TYPE_AMD, SP_PPB,  "Spansion S29GL064MR4 BotB  (8MB)"    ,8,size8K,    127,size64K,   0,0,        0,0  },
  { 0x017E, 0x1001, size8MB, CMD_TYPE_AMD, SP_PPB,  "Spansion S29GL064MR3 TopB  (8MB)"  ,127,size64K,     8,size8K,    0,0,        0,0  },


  { 0x017E, 0x1D00, size4MB, CMD_TYPE_AMD, SP_PPB,  "Spansion S29GL032MR1/2 Uni (4MB)"  ,64,size64K,    0,0,      0,0,        0,0   },
  { 0x017E, 0x1301, size8MB, CMD_TYPE_AMD, SP_PPB,  "Spansion S29GL064MR6/7 Uni (8MB)" ,128,size64K,    0,0,      0,0,        0,0   },
  { 0x017E, 0x0C01, size8MB, CMD_TYPE_AMD, SP_PPB,  "Spansion S29GL064MR1/2 Uni (8MB)" ,128,size64K,    0,0,      0,0,        0,0   },
  { 0x017E, 0x1200, size16MB, CMD_TYPE_AMD, SP_PPB, "Spansion S29GL128M Uni    (16MB)" ,128,size128K,   0,0,      0,0,        0,0   },
  { 0x017E, 0x2101, size16MB, CMD_TYPE_AMD, SP_PPB, "Spansion S29GL128N/P Uni  (16MB)" ,128,size128K,   0,0,      0,0,        0,0   },
  
  // ST/Numonyx uniform  
  { 0x207E, 0x1000, size8MB, CMD_TYPE_AMD, SP_PPB,  "ST M29W640GB BotB          (8MB)"  ,8,size8K,    127,size64K,   0,0,  0,0 },
  { 0x207E, 0x1001, size8MB, CMD_TYPE_AMD, SP_PPB, "ST M29W640GT TopB          (8MB)"  ,127,size64K,   8,size8K,    0,0,  0,0 },    
  { 0x207E, 0x0C00, size8MB, CMD_TYPE_AMD, SP_PPB, "ST M29W640GL Uni           (8MB)"  ,128,size64K,      0,0,      0,0,  0,0 },
  { 0x207E, 0x0C01, size8MB, CMD_TYPE_AMD, SP_PPB, "ST M29W640GH Uni           (8MB)"  ,128,size64K,      0,0,      0,0,  0,0 },
 
  { 0x207E, 0x2100, size16MB, CMD_TYPE_AMD, SP_PPB, "ST M29W128GL Uni          (16MB)"  ,128,size128K,    0,0,      0,0,   0,0  },
  { 0x207E, 0x2101, size16MB, CMD_TYPE_AMD, SP_PPB, "ST M29W128GH Uni          (16MB)"  ,128,size128K,    0,0,      0,0,   0,0  },

  // EON
  { 0x007F, 0x22F9, size4MB, CMD_TYPE_AMD, 0,       "EON EN29LV320BB 2Mx16 BotB (4MB)"  ,8,size8K,     63,size64K,   0,0,  0,0 },
  { 0x007F, 0x22F6, size4MB, CMD_TYPE_AMD, 0,      "EON EN29LV320BT 2Mx16 TopB (4MB)"  ,63,size64K,    8,size8K,    0,0,  0,0 },
  
  { 0x007F, 0x22CB, size8MB, CMD_TYPE_AMD, 0,      "EON EN29LV640B  4Mx16 BotB (8MB)"  ,8,size8K,    127,size64K,   0,0,  0,0 },
  { 0x007F, 0x22C9, size8MB, CMD_TYPE_AMD, 0,      "EON EN29LV640T  4Mx16 TopB (8MB)"  ,127,size64K,   8,size8K,    0,0,  0,0 },
  { 0x007F, 0x227E, size8MB, CMD_TYPE_AMD, 0,      "EON EN29LV640H/L Uni       (8MB)"  ,128,size64K,      0,0,      0,0,  0,0 },
  
  { 0x7F7E, 0x1000, size8MB, CMD_TYPE_AMD, SP_PPB, "EON EN29GL064B BotB        (8MB)"  ,8,size8K,    127,size64K,   0,0,  0,0 },
  { 0x7F7E, 0x1001, size8MB, CMD_TYPE_AMD, SP_PPB, "EON EN29GL064T TopB        (8MB)"  ,127,size64K,   8,size8K,    0,0,  0,0 },    
  { 0x7F7E, 0x0C01, size8MB, CMD_TYPE_AMD, SP_PPB, "EON EN29GL064H/L Uni       (8MB)"  ,128,size64K,      0,0,      0,0,  0,0 },  
  { 0x7F7E, 0x2101, size16MB,CMD_TYPE_AMD, SP_PPB, "EON EN29GL128 Uni         (16MB)"  ,128,size128K,     0,0,      0,0,  0,0 },  

  // MXIC
  { 0xC27E, 0x1A00, size4MB, CMD_TYPE_AMD, SP_PPB, "MX29GL320EB/LV320MB BotB   (4MB)"  ,8,size8K,     63,size64K,   0,0,  0,0  },
  { 0xC27E, 0x1A01, size4MB, CMD_TYPE_AMD, SP_PPB, "MX29GL320ET/LV320MT TopB   (4MB)"  ,63,size64K,    8,size8K,    0,0,  0,0  },
  { 0xC27E, 0x1D00, size4MB, CMD_TYPE_AMD, SP_PPB, "MX29GL320EH/L Uni          (4MB)"  ,64,size64K,       0,0,      0,0,  0,0  },

  { 0x00C2, 0x22CB, size8MB, CMD_TYPE_AMD, 0,      "MX29LV640EB/DB  BotB       (8MB)"  ,8,size8K,    127,size64K,   0,0,  0,0 },
  { 0x00C2, 0x22C9, size8MB, CMD_TYPE_AMD, 0,      "MX29LV640ET/DT  TopB       (8MB)"  ,127,size64K,   8,size8K,    0,0,  0,0 },
 
  { 0xC27E, 0x1000, size8MB, CMD_TYPE_AMD, SP_PPB, "MX29GL640EB/LV640MB  BotB  (8MB)"  ,8,size8K,    127,size64K,   0,0,  0,0 },
  { 0xC27E, 0x1001, size8MB, CMD_TYPE_AMD, SP_PPB, "MX29GL640ET/LV640MT  TopB  (8MB)"  ,127,size64K,   8,size8K,    0,0,  0,0 },    
  { 0xC27E, 0x0C01, size8MB, CMD_TYPE_AMD, SP_PPB, "MX29GL640EH/L Uni          (8MB)"  ,128,size64K,      0,0,      0,0,  0,0 },
  { 0xC27E, 0x2101, size16MB,CMD_TYPE_AMD, SP_PPB, "MX29GL128E Uni            (16MB)"  ,128,size128K,     0,0,      0,0,  0,0 },  

  { 0x00C2, 0x227A, size16MB, CMD_TYPE_AMD, 0,     "MX29LV128DB  8Mx16 BotB   (16MB)"  ,8,size8K,    255,size64K,   0,0,  0,0 },
  { 0x00C2, 0x227E, size16MB, CMD_TYPE_AMD, 0,     "MX29LV128DT  8Mx16 TopB   (16MB)"  ,255,size64K,   8,size8K,    0,0,  0,0 },  

  // 20100904  
  { 0x017E, 0x3701, size16MB, CMD_TYPE_AMD, SP_PPB,"S29GL128N/M29W128G Uni    (16MB)"  ,128, size128K, 0, 0,    0, 0,   0, 0},   
 
  { 0x017E, 0x2201, size32MB, CMD_TYPE_AMD, 0, "Spansion S29GL256P U      (32MB)"   ,256,size128K,     0,0,   0,0,        0,0        },
  { 0x017E, 0x2301, size64MB, CMD_TYPE_AMD, 0, "Spansion S29GL512P U      (64MB)"   ,512,size128K,     0,0,   0,0,        0,0        },
  { 0x017E, 0x2801, size128MB,CMD_TYPE_AMD, 0, "Spansion S29GL01GP U     (128MB)"   ,1024,size128K,     0,0,   0,0,        0,0        },

  // Winbond 3-stage ID chips
  { 0xDA7E, 0x0A00, size4MB, CMD_TYPE_AMD, 0, "Winbond W19B320AB BotB     (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
  { 0xDA7E, 0x0A01, size4MB, CMD_TYPE_AMD, 0, "Winbond W19B320AT TopB     (4MB)"   ,63,size64K,     8,size8K,   0,0,        0,0        },

  // Atmel
  { 0x001F, 0x00C8, size4MB, CMD_TYPE_AMD, 0, "AT49BV322A 2Mx16 BotB      (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
  { 0x001F, 0x00C9, size4MB, CMD_TYPE_AMD, 0, "AT49BV322A(T) 2Mx16 TopB   (4MB)"   ,63,size64K,     8,size8K,   0,0,        0,0        },

  // Add new flash model before this line  
  { 0, 0, 0, 0, 0,   0,0,   0,0,   0,0,   0,0 }
};


// -----------------------------------------
// ---- Start of Compiler Specific Code ----
// -----------------------------------------
#ifdef WINDOWS_VERSION

typedef BOOL  (_stdcall *tInitWio) (void);
typedef BOOL  (_stdcall *tShutdownWio) (void);
typedef BOOL  (_stdcall *tGetPortVal) (WORD p, DWORD *d, BYTE sz);
typedef BOOL  (_stdcall *tSetPortVal) (WORD p, DWORD  d, BYTE sz);

static tInitWio   _initwio;
static tShutdownWio _shutdownwio;
static tGetPortVal _getport;
static tSetPortVal _setport;

HINSTANCE hDll = NULL;

static void lpt_openport(void)
{
  
  HANDLE h;
  
  if ( is_os64 )
  	 hDll = LoadLibrary ("WinIo32.dll");
  	else
  	 hDll = LoadLibrary ("WinIo32.dll");
  	
  if (hDll == NULL)
    {
        printf("Couldn't load WinIO DLL liberary\n");
        exit (1);
    }
  
  _initwio =     (tInitWio) GetProcAddress (hDll, "InitializeWinIo");
  _shutdownwio = (tShutdownWio) GetProcAddress (hDll, "ShutdownWinIo");  
  _getport =     (tGetPortVal) GetProcAddress (hDll, "GetPortVal");
  _setport =     (tSetPortVal) GetProcAddress (hDll, "SetPortVal");
  
  if (!_initwio || !_shutdownwio || !_getport || !_setport ) 
  	{
      printf("WinIo func link error!\n");
      exit(1);
    }
 
 if ( _initwio()) return;
 	 printf("WinIo sys driver initialization error!\n");
      exit(1);	

}

static void lpt_closeport(void)
{
	if (hDll != NULL)
		{
			_shutdownwio();
		  FreeLibrary (hDll);
		}
}

static void	_outp64(WORD p, int d)
{
	_setport(p, (DWORD)d, 1);
}

static BYTE	_inp64(WORD p)
{
	  DWORD d;
	  
	  if ( _getport(p, &d, 1)) return (BYTE)d;
   	 return 0xFF;
}

#else  // ifdef WINDOWS_VERSION

static void lpt_openport(void)
{
#ifdef __FreeBSD__     // ---- Compiler Specific Code ----

  pfd = open("/dev/ppi0", O_RDWR);
  if (pfd < 0)
    {
      perror("Failed to open /dev/ppi0");
      exit(1);
    }
  if ((ioctl(pfd, PPEXCL) < 0) || (ioctl(pfd, PPCLAIM) < 0))
    {
      perror("Failed to lock /dev/ppi0");
      close(pfd);
      exit(1);
    }

#else  //ifdef __FreeBSD__ 

  pfd = open("/dev/parport0", O_RDWR);
  if (pfd < 0)
    {
      perror("Failed to open /dev/parport0");
      exit(1);
    }
  if ((ioctl(pfd, PPEXCL) < 0) || (ioctl(pfd, PPCLAIM) < 0))
    {
      perror("Failed to lock /dev/parport0");
      close(pfd);
      exit(1);
    }

#endif //ifdef __FreeBSD__
}

static void lpt_closeport(void)
{
#ifndef __FreeBSD__    // ---- Compiler Specific Code ----

  if (ioctl(pfd, PPRELEASE) < 0)
    {
      perror("Failed to release /dev/parport0");
      close(pfd);
      exit(1);
    }

#endif

  close(pfd);
}
#endif  // ifdef WINDOWS_VERSION


static BYTE clockin(int tms, int tdi)
{
  BYTE data;
 
  tms = tms ? 1 : 0;
  tdi = tdi ? 1 : 0;

  switch( cable_type )
    {
        case WIGGLER:
           data = ((1 << WTDO) | (0 << WTCK) | (tms << WTMS) | (tdi << WTDI)) ^ WO_INV;
           _OUTP(data);
           if(ejtag_speed) ussleep(1);
           data = ((1 << WTDO) | (1 << WTCK) | (tms << WTMS) | (tdi << WTDI)) ^ WO_INV;
           _OUTP(data);
           if(ejtag_speed) ussleep(1);      //LPT port ECP mode max speed up to 2.5MHz, add 2us delay limit to 500KHz
           _INP(data);          // output inversed bit 7
           data ^= WI_INV;      // invert bit 7
           data >>= WTDO;       // shift bit 7 to bit 0
           data &= 1;           // get bit 0 output
           break;      

		case BLACKCAT:    //blackcat
           data = ((0 << BTCK) | (tms << BTMS) | (tdi << BTDI)) ^ BO_INV;
           _OUTP(data);
		   //printf("BIN -> %X\n", data);
           if(ejtag_speed) ussleep(1);
           data = ((1 << BTCK) | (tms << BTMS) | (tdi << BTDI)) ^ BO_INV;
           _OUTP(data);
		   //printf("BIN -> %X\n", data);
           if(ejtag_speed) ussleep(1);
           _INP(data);          // output bit 4
           data ^= BI_INV;       // invert bit
           data >>= BTDO;        // shift bit 4 to bit 0 
           data &= 1;           // get bit 0 output
		   //printf("BTDO = %X\n", data);
		   break;

 		default:    //XILINX
           data = ((1 << TDO) | (0 << TCK) | (tms << TMS) | (tdi << TDI)) ^ O_INV;
           _OUTP(data);
		   //printf("IN -> %X\n", data);
           if(ejtag_speed) ussleep(1);
           data = ((1 << TDO) | (1 << TCK) | (tms << TMS) | (tdi << TDI)) ^ O_INV;
           _OUTP(data);
		   //printf("IN -> %X\n", data);
           if(ejtag_speed) ussleep(1);
           _INP(data);          // output bit 4
           data ^= I_INV;       // invert bit
           data >>= TDO;        // shift bit 4 to bit 0 
           data &= 1;           // get bit 0 output
		   //printf("TDO = %X\n", data);
     }
     
  return data;
}

static void lpt_srst(void)
{
  BYTE data;
 
  switch( cable_type )
    {
        case WIGGLER:
           data = (1 << WSRST_N  ) ^ WO_INV;
           _OUTP(data);
           mssleep(10);  //hold reset line 10ms
           data = (0) ^ WO_INV;
           _OUTP(data);

           break;      

		case BLACKCAT:    //BLACKCAT
           data = (1 << BSRST_N  ) ^ BO_INV;
           _OUTP(data);
           mssleep(10); //hold reset line 10ms
           data = (0 ) ^ BO_INV;
           _OUTP(data);

 		default:    //XILINX
           data = (1 << WSRST_N  ) ^ O_INV;
           _OUTP(data);
           mssleep(10); //hold reset line 10ms
           data = (0 ) ^ O_INV;
           _OUTP(data);
           

     }
     
  mssleep(100);
}

static void test_reset(void)
{
  if (!cable_intf){
  clockin(1, 0);  // Run through a handful of clock cycles with TMS high to make sure
  clockin(1, 0);  // we are in the TEST-LOGIC-RESET state.
  clockin(1, 0);
  clockin(1, 0);
  clockin(1, 0);
  clockin(0, 0);  // enter runtest-idle
  }
  else 
  	cable_prop->test_reset();
}

static DWORD last_det_instr_data = 0x0;
// detect chip instrument length
static int det_instr(int iz_Total)
{
  int i,det_irlen=0;
  DWORD out_data = 0;
  BYTE  out_bit;

  if (!cable_intf){
   clockin(1, 0);  // enter select-dr-scan
   clockin(1, 0);  // enter select-ir-scan
   clockin(0, 0);  // enter capture-ir
   clockin(0, 0);  // enter shift-ir (dummy)
   for (i = 0 ; i < 32 ; i++)
    {
      out_bit  = clockin((i == 31), 1); //shift all 1, BYPASS
      out_data = out_data | (out_bit << i);
    }
   clockin(1, 0);  // enter update-ir
   clockin(0, 0);  // enter runtest-idle
  }
  else {
  	out_data = cable_prop->det_instr();
  }
// printf("out_data=%#08x \n",out_data); // for debugging
// out_data=0x848; // for testing only
// (lsb first) 100001001000 (0x848) means the first irlen is 5, the second is 3, the third is 4.
 last_det_instr_data = out_data;

  for (i = 1; i <32; i++) {
    if ( (out_data >> i)&1 ) {
	  det_irlen=i+1;
	  if (iz_Total!=1){ return (i+1); }
	}
  }

  return det_irlen;

}

static int get_irlen_for_dev(int device_num)
{
  int i, cur_device=0, tot_device=0, cur_irlen=1;
  for (i = 1; i <32; i++) {
    if ( (last_det_instr_data >> i)&1 ) {
		tot_device++;
    }
  }
  cur_device=tot_device;
  cur_irlen=1;
  for (i = 1; i <32; i++) {
    cur_irlen++;
    if ( (last_det_instr_data >> i)&1 ) {
        if ((cur_device==device_num)) {
          return cur_irlen;
        }
		cur_irlen=0;
		cur_device--;
    }
  }
  return 0;
}

static DWORD curinstr = 0xFFFFFFFF;
static DWORD set_instr(DWORD instr)
{
  int i;
  DWORD out_data = 0;
  BYTE  out_bit;

  if (DEBUGMSG) printf("SET INSTRUCTION: 0x%04x  \n", instr);
  if (instr == curinstr)
    return 0;
  if (!cable_intf){
   clockin(1, 0);  // enter select-dr-scan
   clockin(1, 0);  // enter select-ir-scan
   clockin(0, 0);  // enter capture-ir
   clockin(0, 0);  // enter shift-ir (dummy)
   for (i=0; i < instruction_length; i++)
    {
      out_bit  = clockin(i==(instruction_length - 1), (instr>>i)&1);
	  out_data = out_data | (out_bit << i);
    }
   clockin(1, 0);  // enter update-ir
   clockin(0, 0);  // enter runtest-idle
  }
  else
    out_data = cable_prop->set_instr(instr);

  curinstr = instr;
  return out_data;
}


static DWORD ReadWriteData(DWORD in_data)
{
  int i;
  DWORD out_data = 0;
  BYTE  out_bit;

  if (DEBUGMSG) printf("INSTR: 0x%04x  ", curinstr);
  if (DEBUGMSG) printf("W: 0x%08x ", in_data);

  if (!cable_intf){
   clockin(1, 0);  // enter select-dr-scan
   clockin(0, 0);  // enter capture-dr
   clockin(0, 0);  // enter shift-dr
   for (i = 0 ; i < 32 ; i++)
    {
      out_bit  = clockin((i == 31), ((in_data >> i) & 1)); //final bit 1 exit-dr
      out_data = out_data | (out_bit << i);
    }
   clockin(1,0);   // enter update-dr
   clockin(0,0);   // enter runtest-idle
  }
  else
     out_data = cable_prop->ReadWriteData(in_data);  	

  if (DEBUGMSG) printf("R: 0x%08x\n", out_data);

  return out_data;
}


static DWORD ReadData(void)
{
  if (!cable_intf)
       return ReadWriteData(0x00);
  else
  	   return cable_prop->ReadData();
}


static void WriteData(DWORD in_data)
{
  if (!cable_intf)
      ReadWriteData(in_data);
  else
      cable_prop->WriteData(in_data);
}


static DWORD ejtag_read_x(DWORD addr, int mode)
{
  if (USE_DMA) 
  	return(ejtag_dma_read_x(addr,mode));
   else  
   	return(ejtag_pracc_read_x(addr,mode));

}


static void ejtag_write_x(DWORD addr, DWORD data,int mode)
{
  if (USE_DMA) 
  	ejtag_dma_write_x(addr, data, mode);
  else 
  	ejtag_pracc_write_x(addr, data, mode);
}

////////////////////////////////////////////////////////////////////////

static DWORD ejtag_dma_read_x(DWORD addr, int mode)
{
  DWORD data;
  int retries = MAX_ATTEMPTS;
  int timeout = MAX_TIMEOUT;
  int k;

if (cable_intf && (cable_prop->feature&CBL_DMA_RD)) { return cable_prop->ejtag_dma_read_x( addr, mode);  }
  
begin_ejtag_dma_read_h:

  // Setup Address
  set_instr(INSTR_ADDRESS);
  WriteData(addr);

  // Initiate DMA Read & set DSTRT
  set_instr(INSTR_CONTROL);
 WriteData(DMAACC | DRWN | DMASZ(mode) | DSTRT | PROBEN | PRACC);

  // Wait for DSTRT to Clear
  while (ReadWriteData(DMAACC | PROBEN | PRACC) & DSTRT)
      { if(!(timeout--)) break;}
	
  // Read Data
  set_instr(INSTR_DATA);
  data = ReadData();

  // Clear DMA & Check DERR
  set_instr(INSTR_CONTROL);
  if (ReadWriteData(PROBEN | PRACC) & DERR)
    { 
      if (retries--)  
      	{test_reset(); goto begin_ejtag_dma_read_h;}
      else  
      	{printf("DMA Read(x%d) Addr = %08x  Data = (%08x)ERROR ON READ\n",1<<mode, addr, data);return(0xFFFFFFFF);}
    }

// printf("DMA Read(x%d) Addr = %08x  Data = (%08x)\n",mode, addr, data);

  switch(mode)
   {
 	   case MIPS_WORD:
 	 	  break;
 	
 	   case MIPS_HALFWORD:
 	    k = addr & 0x2;
    	if (BigEndian)
  	     	data = (data >> (8*(2-k))) & 0xffff;  //low 16 at high
	      else	//little
  	     	data = (data >> (8*k)) & 0xffff;      //low 16 at low
		  break;
		
	   case MIPS_BYTE:
			 k = addr & 0x3;
    	 if (BigEndian)
  	     	data = (data >> (8*(3-k))) & 0xff;    //low 8 at high
	      else	//little
  	     	data = (data >> (8*k)) & 0xff;        //low 8 at low
		   break;  	     	
	
	   default:      //not supported mode
   		 data = 0xFFFFFFFF;
		   break;
   }
//   printf("return data %08x\n", data);
  return(data);       //endian not corrected
}
static void ejtag_dma_write_x(DWORD addr, DWORD data, int mode)
{
  int   retries = MAX_ATTEMPTS;
  int timeout = MAX_TIMEOUT;

if (cable_intf && (cable_prop->feature&CBL_DMA_WR)) { cable_prop->ejtag_dma_write_x(addr, data, mode); return;  }
  
begin_ejtag_dma_write:

  // Setup Address
  set_instr(INSTR_ADDRESS);
  // added by tumpa
  if ( cable_type == FT2232H) mssleep(delay_in_ms);

  WriteData(addr);
  // added by tumpa
  if ( cable_type == FT2232H) mssleep(delay_in_ms);

  // Setup Data
  set_instr(INSTR_DATA);
  // added by tumpa
  if ( cable_type == FT2232H) mssleep(delay_in_ms);

  WriteData(data);
  // added by tumpa
  if ( cable_type == FT2232H) mssleep(delay_in_ms);
  // Initiate DMA Write & set DSTRT
  set_instr(INSTR_CONTROL);
  // added by tumpa
  if ( cable_type == FT2232H) mssleep(delay_in_ms);

  WriteData(DMAACC | DMASZ(mode)| DSTRT | PROBEN | PRACC);
  // added by tumpa
  if ( cable_type == FT2232H) mssleep(delay_in_ms);
  // Wait for DSTRT to Clear
  while (ReadWriteData(DMAACC | PROBEN | PRACC) & DSTRT)
     { if(!(timeout--)) break;}

  // Clear DMA & Check DERR
//  set_instr(INSTR_CONTROL);
  if (ReadWriteData(PROBEN | PRACC) & DERR)
    { 
      if (retries--)  
      	{test_reset(); goto begin_ejtag_dma_write;}
      else  
      	printf("DMA Write(x%d) Addr = %08x  Data = ERROR ON WRITE\n", 1<<mode,addr);
    }
}

static DWORD ejtag_pracc_read_x(DWORD addr, int mode)
{
if (cable_intf && (cable_prop->feature&CBL_PRACC_RD)) { return cable_prop->ejtag_pracc_read_x( addr, mode);  }

  data_register    = 0x0;
  ExecuteDebugModule(ejtag_fix_readcode(addr, mode));
  return(data_register);
}

static void ejtag_pracc_write_x(DWORD addr, DWORD data, int mode)
{
if (cable_intf && (cable_prop->feature&CBL_PRACC_WR)) { cable_prop->ejtag_pracc_write_x(addr, data, mode); return;  }
  data_register    = data;
  ExecuteDebugModule(ejtag_fix_writecode(addr, data, mode));
}

static DWORD *ejtag_fix_readcode( DWORD addr, int mode )
{
   WORD addr_hi, addr_lo;
   DWORD *p;
	
   addr_lo = addr & 0xFFFF;   // %lo()
   addr_hi = ((addr >> 16) + (addr_lo >> 15)) | 0xA000;  //%hi(), and use uncached kseg1
 
    switch (mode)
   {
     case MIPS_WORD:
           p      =  pracc_read_x32;      
         *(p    ) = (*(p  ) & 0xFFFF0000) | (addr_hi & 0xFFFF);  //fill addr_hi
         *(p + 1) = (*(p+1) & 0xFFFF0000) | (addr_lo & 0xFFFC);  //fill addr_lo and align to word
          break;
                
                
     case MIPS_HALFWORD:
     	     p      =  pracc_read_x16;      
         *(p    ) = (*(p  ) & 0xFFFF0000) | (addr_hi & 0xFFFF);  //fill addr_hi
         *(p + 1) = (*(p+1) & 0xFFFF0000) | (addr_lo & 0xFFFE);  //fill addr_lo and align to halfword
          break;
                
     case MIPS_BYTE:
     	     p      =  pracc_read_x8;      
         *(p    ) = (*(p  ) & 0xFFFF0000) | (addr_hi & 0xFFFF);  //fill addr_hi
         *(p + 1) = (*(p+1) & 0xFFFF0000) | addr_lo;             //fill addr_lo
          break;

     default:  
              printf("\n***ERROR: unsupported read data type!!! exit...\n");
              exit(1);
              break;
   }
	return p;
}

static DWORD *ejtag_fix_writecode( DWORD addr, DWORD data, int mode )
{
   WORD addr_hi, addr_lo;
	 WORD data_hi, data_lo;
	 DWORD *p;
	
   addr_lo = addr & 0xFFFF;   // %lo()
   addr_hi = ((addr >> 16) + (addr_lo >> 15)) | 0xA000;  //%hi(), and use uncached kseg1
      
   switch (mode)
   {
       case MIPS_WORD:
       	 data_hi = data >> 16;
         data_lo = data & 0xFFFF;
     	     p      =  pracc_write_x32;      
         *(p    ) = (*(p  ) & 0xFFFF0000) | addr_hi;             //fill addr_hi
         *(p + 3) = (*(p+3) & 0xFFFF0000) | (addr_lo & 0xFFFC);  //fill addr_lo and align to word
         *(p + 1) = (*(p+1) & 0xFFFF0000) | data_hi;             //fill data_hi
         *(p + 2) = (*(p+2) & 0xFFFF0000) | data_lo;             //fill data_lo
          break;
          
     case MIPS_HALFWORD:
     	     data_lo = data & 0xFFFF;
           p      =  pracc_write_x16;      
         *(p    ) = (*(p  ) & 0xFFFF0000) | addr_hi;             //fill addr_hi
         *(p + 2) = (*(p+2) & 0xFFFF0000) | (addr_lo & 0xFFFE);  //fill addr_lo and align to halfword
         *(p + 1) = (*(p+1) & 0xFFFF0000) | data_lo;             //fill data_hi
          break;
          
     case MIPS_BYTE:
     	     data_lo = data & 0xFF;
           p      =  pracc_write_x8;      
         *(p    ) = (*(p  ) & 0xFFFF0000) | addr_hi;             //fill addr_hi
         *(p + 2) = (*(p+2) & 0xFFFF0000) | addr_lo;             //fill addr_lo
         *(p + 1) = (*(p+1) & 0xFFFF0000) | data_lo;             //fill data_lo8
          break;                
               
     default:  
              printf("\n***ERROR: unsupported write data type!!! exit...\n");
              exit(1);
              break;
   }
	return p;
}

//==========================================================================

static void ExecuteDebugModule(DWORD *pmodule)
{
  DWORD ctrl_reg;
  DWORD address;
  DWORD data   = 0;
  DWORD offset = 0;
  int finished = 0;

  int retries = MAX_ATTEMPTS;
  int loop_limit1 = MAX_LOOP_CNT;
  int loop_limit2 = MAX_LOOP_CNT;

  if (DEBUGMSG) DBG(("DEBUGMODULE: Start module.\n"));

  // Feed the chip an array of 32 bit values into the processor via the EJTAG port as instructions.
  while (1)
    {
      if(!(loop_limit1--)){ 
        if (DEBUGMSG) DBG(("DEBUGMODULE: Loop limit 1 reached.!\n"));
        break;
      }
      loop_limit2 = MAX_LOOP_CNT;
      // Read the control register.  Make sure an access is requested, then do it.
      while (1)
        {
          if(!(loop_limit2--)){ 
            if (DEBUGMSG) DBG(("DEBUGMODULE: Loop limit 2 reached.!\n"));
            break;
          }
          set_instr(INSTR_CONTROL);
          ctrl_reg = ReadWriteData(PRACC | PROBEN | SETDEV);
          if (ctrl_reg & PRACC)
            break;
          if (DEBUGMSG) DBG(("DEBUGMODULE: No memory access in progress!\n"));
         // 	printf("DEBUGMODULE: in progress!\n");
        }

      set_instr(INSTR_ADDRESS);
      address = ReadData();

      // Check for read or write
      if (ctrl_reg & PRNW) // Bit set for a CPU WRITE to us
        {
          // Read the data out of CPU
          set_instr(INSTR_DATA);
          data = ReadData();
          
          // Clear the access pending bit (let the processor eat!)
          set_instr(INSTR_CONTROL);
          ctrl_reg = ReadWriteData(PROBEN | SETDEV);

          // Processor is writing to us
          if (DEBUGMSG1) DBG(("DEBUGMODULE: Write 0x%08X to address 0x%08X\n", data, address));
          // Handle Debug Write
          // If processor is writing to one of our psuedo virtual registers then save off data
          if (address == MIPS_VIRTUAL_ADDRESS_ACCESS)  address_register = data;
          if (address == MIPS_VIRTUAL_DATA_ACCESS)     data_register    = data;
        } //end cpu write

      else

        {
          // Check to see if its reading at the debug vector.  The first pass through
          // the module is always read at the vector, so the first one we allow.  When
          // the second read from the vector occurs we are done and just exit.
          if (address == MIPS_DEBUG_VECTOR_ADDRESS)
            {
              if (finished++) // Allows ONE pass ,we use a "b" instuction at the end, gp will jump back to start
                {
                  if (DEBUGMSG1) DBG(("DEBUGMODULE: Finished module.\n"));
                  return;
                }
            }

          // Processor is reading from us
          if (address >= MIPS_DEBUG_VECTOR_ADDRESS)
            {
              // Reading an instruction from our module so fetch the instruction from the module
              offset = (address - MIPS_DEBUG_VECTOR_ADDRESS) / 4;
              data = *(DWORD *)(pmodule + offset);    //debug instruction, same for both endians. No reverse required for BIG!
              if (DEBUGMSG1) DBG(("DEBUGMODULE: Instruction read at 0x%08X  offset -> %04d  data -> 0x%08X\n", address, offset, data)); //fflush(stdout);
            }
          else
            {
              // Reading from our virtual register area
              if (DEBUGMSG1) DBG(("DEBUGMODULE: Read address 0x%08X  data = 0x%08X\n", address, data));
              // Handle Debug Read
              // If processor is reading from one of our psuedo virtual registers then give it data
              if (address == MIPS_VIRTUAL_ADDRESS_ACCESS)  data = address_register;
              if (address == MIPS_VIRTUAL_DATA_ACCESS)     data = data_register;
            }

          // Send the data out
          set_instr(INSTR_DATA);
          data = ReadWriteData(data);

          // Clear the access pending bit (let the processor eat!)
          set_instr(INSTR_CONTROL);
          ctrl_reg = ReadWriteData(PROBEN | SETDEV);

        } //end cpu read
    }
}


static void chip_detect(void)
{
  DWORD id = 0x0;
  DWORD cdid = 0, cmfr = 0, crev = 1;
  DWORD nbDevices_data;
  int nbDevices;
  int edbk;
  int i;
  int instrlenbk;

  processor_chip_type*   processor_chip = processor_chip_list;
  cpumfr_type* cmfr_dscr = cpu_mfr_list;
  int counter = 0;

  edbk = endian;

#ifdef WINDOWS_VERSION
  if (!cable_intf){
    printf("Selected port = %#x\n\n",lpt_port);
  }
#endif  // ifdef WINDOWS_VERSION

  test_reset();
  det_instr(1); // this will do LV_mode bypass on some chips. sends 0xffffffff
  instruction_length=8; // set to 8 so we can send CCJT_BYPASS (8-bits)
  set_instr(CCJT_BYPASS);

  instruction_length = det_instr(1);
  printf("Detected IR chain length = %d\n\n",instruction_length);
//  printf("(%#x) \n",last_det_instr_data); // for debugging

  ReadWriteData(0x0); // Send plenty of zeros into the DR registers to flush them
  ReadWriteData(0x0);
  nbDevices_data=ReadWriteData(0xFFFFFFFF); // now send ones until we receive one back
//  printf("device_count response = %x\n",nbDevices_data); // for debug only
  for(i=0; i<32; i++) if((nbDevices_data>>i)&0x1) break;

  nbDevices = i;
  //nbDevices = 3; last_det_instr_data=0x848; // for testing only
  // (lsb first) 100001001000 (0x848) means the first irlen is 5, the second is 3, the third is 4.
  printf("There are %d device(s) in the JTAG chain\n", nbDevices);

  // go to reset state (that loads IDCODE into IR of all the devices)
  for(i=0; i <= nbDevices; i++) {
    test_reset();
  }
  // and read the IDCODES
  for(i=0; i < nbDevices; i++) {
    printf(" IDCODE for device %d is 0x%08X (IR length:%d)\n", i+1, ReadData(), get_irlen_for_dev(i+1));
  }
  printf("\n");

  if (instrlen)
  {
    instruction_length = instrlen;
  }

  if (skipdetect)
    {
      // Allow un-listed cpu run

	  test_reset();  // this doesn't hurt while improves detection

      if (!(instrlen))
      {
        instruction_length = det_instr(0);
      }
      printf("Probing bus ... ");
      set_instr(INSTR_IDCODE);
      id = ReadData();
      cdid = GETDEVID(id);
      crev = GETREVID(id);
      cmfr = GETMFRID(id);
      printf("Done\n\n");
      if (instrlen)
      {
        printf("Instruction Length manually set to %d\n\n",instruction_length);
      }
      else {
        printf("Detected IR Length is %d bits\n\n",instruction_length);
      }

      if(BigEndian)   //big endian
      		printf("CPU assumed running under BIG endian\n\n");
      else
      	printf("CPU assumed running under LITTLE endian\n\n");

      printf("CPU Chip ID: ");
      ShowData(id);
      while(cmfr_dscr->id || cmfr_dscr->name){
        if(cmfr_dscr->id == cmfr) break;
        cmfr_dscr++;
      }
      if(!(cmfr_dscr->id || cmfr_dscr->name)) {
        cmfr_dscr = cpu_mfr_list;
      }

      printf("    CPU Manufacturer :%s(ID=0x%3.3X)\n", cmfr_dscr->name,cmfr);
      printf("    CPU Device ID :%4.4X\n", cdid);
      printf("    CPU Revision  :%d\n\n", crev);

      printf("*** CHIP DETECTION OVERRIDDEN ***\n\n");
      return;
    }
  else    // Matching listed CPU Chip ID
    {
	   i = 0;
       while (processor_chip->chip_id || processor_chip->instr_length)
        {
          if ( !(instrlen) ) 		// no manual IR length
          {
		    if  ( !(processor_chip->instr_length) ) { 		// no defined IR length
			  instruction_length = det_instr(0); 		// detech IR length
            } else { 		// defined IR length available, use it
              instruction_length = processor_chip->instr_length;
            }
          }
		  if ( i==0 ) {
            printf("Probing bus ... ");
          }
          test_reset(); // this doesn't hurt while improves detection
          
		  i++;
          set_instr(INSTR_IDCODE);
          id = ReadData();
          cdid = GETDEVID(id);
          crev = GETREVID(id);
          cmfr = GETMFRID(id);

          if ( (cdid == processor_chip->chip_id) &&
               (cmfr == processor_chip->chip_mfrid) &&
               (!(id == 0 || id == 0xFFFFFFFF)) )
            {
              printf("Done\n\n");

              if (instrlen)
              {
                printf("Instruction Length manually set to %d\n\n",instruction_length);
              }
              else
              {
		        if  ( !(processor_chip->instr_length) ) {
                  printf("Detected IR Length is %d bits\n\n",instruction_length);
                } else {
                  printf("Defined IR Length is %d bits\n\n",instruction_length);
                }
              }
              instrlenbk = processor_chip->instr_length;
               	
              init_code = (DWORD *)processor_chip->init_code;
              mpi_base = processor_chip->mpi_base;
              
              edbk = processor_chip->chip_endn;
               if( !force_endian ) endian = edbk;

              if (BigEndian)
              	   printf("CPU assumed running under BIG endian\n\n");
              	else
              		 printf("CPU assumed running under LITTLE endian\n\n");

              printf("CPU Chip ID: ");
              ShowData(id);
              printf("*** Found a %s manufactured %s REV %2.2d CPU ***\n\n", 
                     processor_chip->mfrdscr_idx->name, processor_chip->chip_descr, crev);
              return;
/*            } else {
              printf(">");*/
            }
          processor_chip++;
        }

    }  //auto detect

  test_reset();
  set_instr(INSTR_IDCODE);
  id = ReadData();
  cdid = GETDEVID(id);
  crev = GETREVID(id);
  cmfr = GETMFRID(id);
  printf("Done\n\n");

  printf("CPU Chip ID: ");
  ShowData(id);
  if (id == 0 || id == 0xFFFFFFFF){
	
    printf("*** Unknown or NO CPU Chip ID Detected ***\n\n");

    printf("*** Possible Causes:\n");
    printf("    1) Router/Modem is not Connected.\n");
    printf("    2) Router/Modem is not Powered On.\n");
    printf("    3) Improper JTAG Cable.\n");
    printf("    4) Unrecognized CPU Chip ID.\n");

  }else{

 	  while(cmfr_dscr->id || cmfr_dscr->name){
       if(cmfr_dscr->id == cmfr) break;
        cmfr_dscr++;
       }
     if(!(cmfr_dscr->id || cmfr_dscr->name)) {
		cmfr_dscr = cpu_mfr_list;
	 }

	   printf("    CPU Manufacturer :%s(ID=0x%3.3X)\n", cmfr_dscr->name,cmfr);
	   printf("    CPU Device ID :%4.4X\n", cdid);
	   printf("    CPU Revision  :%d\n\n", crev);
     printf("*** Detected a CPU but not in build-in list ***\n\n");

     printf("*** You can set /skipdetect let operate continue ***\n\n");
  }

	if ((cable_type == FT2232H) && ((LL1 == 0)||(LL1 > 15000))) { 	// added by Volkan K.
		printf("\nNotice: You are using FT2232H based cable without frequency (clock speed) divisor flag.\n\tMost router's CPU cannot handle highest clock speed, thus you will have to slow down the clock to make it work.\n");
	}

  chip_shutdown();;
  exit(0);
}


static void check_ejtag_features()
{
  DWORD features;

  set_instr(INSTR_IMPCODE);
  features = ReadData();

  printf("    - EJTAG IMPCODE ....... : ");
  ShowData(features);

  // EJTAG Version
  ejtag_version = (features >> 29) & 7;
  printf("    - EJTAG Version ....... : ");
  if (ejtag_version == 0)       printf("1 or 2.0\n");
  else if (ejtag_version == 1)  printf("2.5\n");
  else if (ejtag_version == 2)  printf("2.6\n");
  else if (ejtag_version == 3)  printf("3.1\n");
  else                          printf("Unknown (%d is a reserved value)\n", ejtag_version);

  // EJTAG DMA Support
  USE_DMA = !(features & (1 << 14));
  printf("    - EJTAG DMA Support ... : %s\n", USE_DMA ? "Yes" : "No");
  printf( "    - EJTAG Implementation flags:%s%s%s%s%s%s%s\n",
          (features & (1 << 28)) ? " R3k"	: " R4k",
          (features & (1 << 24)) ? " DINTsup"	: "",
          (features & (1 << 22)) ? " ASID_8"	: "",
          (features & (1 << 21)) ? " ASID_6"	: "",
          (features & (1 << 16)) ? " MIPS16"	: "",
          (features & (1 << 14)) ? " NoDMA"	: "",
          (features & (1      )) ? " MIPS64"	: " MIPS32" );

  if (force_dma)
    {
      USE_DMA = 1;
      printf("    *** DMA Mode Forced On ***\n");
    }
  if (force_nodma)
    {
      USE_DMA = 0;
      printf("    *** DMA Mode Forced Off ***\n");
    }

  printf("\n");
}


static void chip_shutdown(void)
{
  fflush(stdout);
  test_reset();
  if (!cable_intf)
  	 lpt_closeport();
  else
     cable_prop->close();
}

/**** sp_xxxxxx() is a function specific to Spansion flash ****/
static void sp_unlock_bypass(void)
{
	if (cmd_type == CMD_TYPE_AMD)
		{
     sp_exit_cmdset();    //exit any command set
     ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0xaaaaaaaa&PGMASK, PGMODE); /* unlock bypass */
     ejtag_write_x(FLASH_MEMORY_START + AMDUL2, 0x55555555&PGMASK, PGMODE);
     ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0x20202020&PGMASK, PGMODE);
     printf("\nEntered Unlock Bypass mode->\n");
    }
}

static void sp_unlock_bypass_reset(void)
{
  sp_exit_cmdset();
}

static void sp_exit_cmdset(void)
{
	if (cmd_type == CMD_TYPE_AMD)
		{

      ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0x90909090&PGMASK, PGMODE); /* exit command set */
      ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0x00000000&PGMASK, PGMODE);
    }
}

static void sflash_erase_chip(void)
{
   if (cmd_type == CMD_TYPE_AMD)
    {
   			printf("Operation will take 1 or 2 minutes. Don't break!!!\nErasing whole chip ... ");
   			sp_exit_cmdset();    //exit any command set
        ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0xaaAAaaAA&PGMASK, PGMODE); /* Unlock*/
        ejtag_write_x(FLASH_MEMORY_START + AMDUL2, 0x55555555&PGMASK, PGMODE);
        ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0x80808080&PGMASK, PGMODE);
              //Erase Chip
        ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0xaaAAaaAA&PGMASK, PGMODE);
        ejtag_write_x(FLASH_MEMORY_START + AMDUL2, 0x55555555&PGMASK, PGMODE);
        ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0x10101010&PGMASK, PGMODE);

        if(PRG_X8)
        	   sflash_poll_x8(FLASH_MEMORY_START, 0xFF);
        	else
             sflash_poll_x16(FLASH_MEMORY_START, 0xFFFF);
        printf("Done!\n\n");
	      sflash_reset();
	    }

   if (cmd_type == CMD_TYPE_SST)     //SST 39serial only support x16
    {
   			printf("Operation will take 1 or 2 minutes. Don't break!!!\nErasing whole chip ... ");
   			sflash_reset();    //exit any command set
        ejtag_write_x(FLASH_MEMORY_START + (0x5555 << 1), 0xaaAAaaAA&PGMASK, PGMODE); /* Unlock*/
        ejtag_write_x(FLASH_MEMORY_START + (0x2AAA << 1), 0x55555555&PGMASK, PGMODE);
        ejtag_write_x(FLASH_MEMORY_START + (0x5555 << 1), 0x80808080&PGMASK, PGMODE);
              //Erase Chip
        ejtag_write_x(FLASH_MEMORY_START + (0x5555 << 1), 0xaaAAaaAA&PGMASK, PGMODE);
        ejtag_write_x(FLASH_MEMORY_START + (0x2AAA << 1), 0x55555555&PGMASK, PGMODE);
        ejtag_write_x(FLASH_MEMORY_START + (0x5555 << 1), 0x10101010&PGMASK, PGMODE);

        if(PRG_X8)
        	   sflash_poll_x8(FLASH_MEMORY_START, 0xFF);
        	else
             sflash_poll_x16(FLASH_MEMORY_START, 0xFFFF);
        printf("Done!\n\n");
	      sflash_reset();
	    }
	    
	 if (cmd_type == CMD_TYPE_BCS)
    {
      if (chip_features & FULL_CHIP_ERASE)
      {	
      printf("Operation will take 1 or 2 minutes. Don't break!!!\nErasing whole chip ... ");
      sflash_reset();
      //Unlock Block
      ejtag_write_x(FLASH_MEMORY_START, 0x50505050&PGMASK, PGMODE);     // Clear Status Command
      ejtag_write_x(FLASH_MEMORY_START, 0x60606060&PGMASK, PGMODE);     // Unlock Flash Block Command
      ejtag_write_x(FLASH_MEMORY_START, 0xd0D0d0D0&PGMASK, PGMODE);     // Confirm Command

      // Wait for Unlock Completion
      if(PRG_X8)
        	   sflash_poll_x8(FLASH_MEMORY_START, STATUS_READY);
       	else
             sflash_poll_x16(FLASH_MEMORY_START, STATUS_READY);

      //Erase chip
      ejtag_write_x(FLASH_MEMORY_START, 0x50505050&PGMASK, PGMODE);     // Clear Status Command
      ejtag_write_x(FLASH_MEMORY_START, 0x30303030&PGMASK, PGMODE);     // Full Chip Erase Command
      ejtag_write_x(FLASH_MEMORY_START, 0xd0D0d0D0&PGMASK, PGMODE);     // Confirm Command

      // Wait for Erase Completion
      if(PRG_X8)
        	   sflash_poll_x8(FLASH_MEMORY_START, STATUS_READY);
       	else
             sflash_poll_x16(FLASH_MEMORY_START, STATUS_READY);

      printf("Done!\n\n");
      sflash_reset();
      } else {
      	printf("Chip not support full chip erase!!\n");
      }

    }   
}

/***  Check Flash Protection Status   ***/
static void sp_check_ppb(void)
{
	DWORD glbppb=0, lockreg=0, sppb=0, cur_block, block_addr2;
  DWORD tmp;
	
	printf("\n\nFlash Sector Protection type %1d\n\n",PPB_type);
	
  if (PPB_type==SP_PPB)
  {
    if (issue_show_ppb)
 	  { 
       // read Lock Register
       sp_exit_cmdset();    //exit any command set
       ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0xaaAAaaAA&PGMASK, PGMODE); /* Lock Register Command Set Entry */
       ejtag_write_x(FLASH_MEMORY_START + AMDUL2, 0x55555555&PGMASK, PGMODE);
       ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0x40404040&PGMASK, PGMODE);
       lockreg = ejtag_read_x(FLASH_MEMORY_START,PGMODE );
       sp_exit_cmdset();    //exit any command set
       printf("\nRead Lock Register Status: ");
              ShowData_h(lockreg);
 
       // read Global PPB Lock status
        ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0xaaAAaaAA&PGMASK, PGMODE); /* Global PPB Lock Command Set Entry */
        ejtag_write_x(FLASH_MEMORY_START + AMDUL2, 0x55555555&PGMASK, PGMODE);
        ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0x50505050&PGMASK, PGMODE);
        glbppb = ejtag_read_x(FLASH_MEMORY_START,PGMODE);
        sp_exit_cmdset();    //exit any command set
        printf("\nRead Globle PPB Lock Status: ");
               ShowData_h(glbppb);
                    
       // read Non-volatile Sector PPBs
            ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0xaaAAaaAA&PGMASK, PGMODE); /* Sector PPB Command Set Entry */
            ejtag_write_x(FLASH_MEMORY_START + AMDUL2, 0x55555555&PGMASK, PGMODE);
            ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0xc0C0c0C0&PGMASK, PGMODE);
        for (cur_block = 1;  cur_block <= block_total;  cur_block++)
         {
            block_addr2 = blocks[cur_block];
            sppb = ejtag_read_x(block_addr2,PGMODE);
            printf("Read Sector: %d (addr = %08x) with PPB:", cur_block, block_addr2);
               ShowData_h(sppb);
          }
            sp_exit_cmdset();    //exit any command set
      } //complete show PPB
      
    if (issue_clear_ppb)     //clear PPB to unprotect 
	    {
        printf("Erase all Sector PPBs...");   
        sp_exit_cmdset();    //exit any command set
        ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0xaaAAaaAA&PGMASK, PGMODE); // Sector PPB Command Set Entry 
        ejtag_write_x(FLASH_MEMORY_START + AMDUL2, 0x55555555&PGMASK, PGMODE);
        ejtag_write_x(FLASH_MEMORY_START + AMDUL1, 0xc0C0c0C0&PGMASK, PGMODE);
        ejtag_write_x(FLASH_MEMORY_START ,         0x80808080&PGMASK, PGMODE); // Erase all sector PPB 
        ejtag_write_x(FLASH_MEMORY_START ,         0x30303030&PGMASK, PGMODE);

        while ( (tmp=ejtag_read_x(FLASH_MEMORY_START,PGMODE)) != 0x1 ) 
         { if (DEBUGMSG1) DBG(("%08X, poll on PPB erase!...\n",tmp));}
        mssleep(2000);
        sp_exit_cmdset();    //exit any command set            
        printf("   Done!\n\n"); 
    
     } //complete clear PPB
  }   //end Spansion PPB protectio
}

static void g_init_cpu()
{
  address_register = 0xBFC00000;  // just here,not used
  data_register    = 0x0;          // just here,not used
  ExecuteDebugModule(init_code);
}

// initialize $1 register with 0xFF200000, PraCC vitual data register base address.
static void g_init_dreg()
{
  address_register = 0xBFC00000;  // just here,not used
  data_register    = 0x0;          // just here,not used
  ExecuteDebugModule(pracc_init_dreg);
}

DWORD rev_endian(DWORD data)
{
// swap whole word  
    return ((data << 24) & 0xff000000) |
           ((data <<  8) & 0x00ff0000) |
           ((data >>  8) & 0x0000ff00) |
           ((data >> 24) & 0x000000ff) ;
}

DWORD rev_endian_h(DWORD data)
{
// swap lower bytes  
    return  ((data << 8) & 0x0000ff00) |
            ((data >> 8) & 0x000000ff) ;
}



void ShowData(DWORD value)
{
  int i;
  for (i=0; i<32; i++)
    printf("%d", (value >> (31-i)) & 1);
  printf(" (0x%08X)\n", value);
}

void ShowData_h(DWORD value)
{
  int i;
  value = value & 0xFFFF;
  for (i=0; i<16; i++)
  printf("%d", (value >> (16-i)) & 1);
  printf(" (%04X)\n", value);
}


static void run_backup(char *filename, DWORD start, DWORD length)
{
  DWORD addr, data;
  FILE *fd;
  int counter = 0;
  int percent_complete = 0;
  char newfilename[128] = "";
  time_t start_time = time(0);
  time_t end_time, elapsed_seconds;
  int align=0;
  BYTE *overead;
  DWORD blocksize;
  DWORD blkbuffer[1300];
  int len, ilen;
//  DWORD prolen = start;
  
  struct tm* lt = localtime(&start_time);
  char time_str[15];

  sprintf(time_str, "%04d%02d%02d_%02d%02d%02d",
          lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
          lt->tm_hour, lt->tm_min, lt->tm_sec
         );

  printf("*** You Selected to Backup the %s ***\n\n",filename);

  strcpy(newfilename,filename);
  strcat(newfilename,".SAVED");
  if (issue_timestamp)
    {
      strcat(newfilename,"_");
      strcat(newfilename,time_str);
    }

  fd = fopen(newfilename, "wb" );
  if (fd<=0)
    {
      fprintf(stderr,"Could not open %s for writing\n", newfilename);
      exit(1);
    }

  printf("=========================\n");
  printf("Backup Routine Started\n");
  printf("=========================\n");

  printf("\nSaving %s to Disk...\n",newfilename);

  if (cable_intf && !safemode)
  {	

   addr = start;
   while ( addr < (start+length))
   {
     if((addr + 4*LL7)> ( start + length)) blocksize = start + length - addr;
      	else blocksize = (4*LL7);
      		
     ilen = (blocksize%4) ? (blocksize/4 + 1):(blocksize/4) ;    
     	
     ilen = cable_prop->sflash_blkread(addr, blkbuffer, ilen ); // ask N DWORD data in buffer,return actual read
     fflush(stdout);
     if ( 4*ilen < blocksize) 
     	{
     		LL7 = ilen;   // reset the LL7, if DMA xfer block can't support too much
        blocksize = 4*ilen;
      }
       //bulk read, data endian has been corrected
     fwrite( (BYTE*) blkbuffer, 1, blocksize, fd);
     
     counter += blocksize;
     percent_complete = (counter * 100 / length);
     printf("%4d%%   bytes = %d\r", percent_complete, counter);
     fflush(stdout);

    addr += blocksize;

   }
//----------
  }
  else
  {
//----------  
#define ALAL 4
    for (addr=start; addr<(start+length); addr+=ALAL)
    {
      counter += ALAL;
      percent_complete = (counter * 100 / length);
      if (!silent_mode)
        if ((addr&0xF) == 0)  printf("[%3d%% Backed Up]   %08X: ", percent_complete, addr);
      data = ejtag_read_x(addr, MIPS_WORD);  //whole word(32bits) memory operation
      if(BigEndian) data= rev_endian(data);  //correct data endian
         fwrite( (BYTE*) &data, 1, sizeof(data), fd);

      if (silent_mode)  printf("%4d%%   bytes = %d\r", percent_complete, counter);
      else              printf("%08X%c", data, (addr&0xF)==0xC?'\n':' ');

      fflush(stdout);

//      if ((addr - start) > 0x10) exit(0);
    }
  }
  fclose(fd);

  printf("Done  (%s saved to Disk OK)\n\n",newfilename);
  printf("bytes written: %d\n", counter);

  printf("=========================\n");
  printf("Backup Routine Complete\n");
  printf("=========================\n");

  time(&end_time);
  elapsed_seconds = difftime(end_time, start_time);
  printf("elapsed time: %d seconds\n", (int)elapsed_seconds);
}

static void run_flash(char *filename, DWORD start, DWORD length)
{
  DWORD addr, data ;
  FILE *fd ;
  int counter = 0;
  int percent_complete = 0;
  time_t start_time = time(0);
  time_t end_time, elapsed_seconds;
  int err=0;
  int blocksize;
  DWORD blkbuffer[200];
  int len, ilen;
  int i;
  int wflag = 0;

  printf("*** You Selected to Flash the %s ***\n\n",filename);

  fd=fopen(filename, "rb" );
  if (fd<=0)
    {
      fprintf(stderr,"Could not open %s for reading\n", filename);
      exit(1);
    }

  printf("=========================\n");
  printf("Flashing Routine Started\n");
  printf("=========================\n");

  if (issue_erase) err = sflash_erase_area(start,length);
    else 
    	if ((cmd_type == CMD_TYPE_BCS) || (cmd_type == CMD_TYPE_SCS))  //intel type flash need unlock sector protection
    		{ err = sflash_erase_area(start,length);}
    	else
    		{ check_align = 1; err = sflash_erase_area(start,length);}
  if (err) return;

  printf("\nLoading %s to Flash Memory...\n",filename);
  if ( cable_type == FT2232H) mssleep(1000);
  if (bypass)
    {
      sp_unlock_bypass();
    }
  if ( cable_type == FT2232H)  mssleep(1000);    
if (cable_intf && !safemode ) 
  {
 
   addr = start;
   
   blocksize = cable_prop->sflash_blkwrite(0, blkbuffer, 0,  0 );  // find supported max block size in x16 mode
   if (LL8 > blocksize) LL8 = blocksize;              // reset LL8 if needed
   if(PRG_X8) blocksize = LL8/2;
   
   while ( addr < (start+length))
   {

      for(i=0; i<LL8; i++ ) 
        blkbuffer[i] = 0xFFFFFFFF;  // This is in case file is shorter than expected length
      
      if((addr + 4*LL8)> ( start + length)) blocksize = start + length - addr;
      	else blocksize = (4*LL8);

      len = fread( (BYTE*) blkbuffer, 1, blocksize, fd);

      ilen = (blocksize%4) ? (blocksize/4 + 1):(blocksize/4);    

      // Erasing Flash Sets addresses to 0xFF's so we can avoid writing these (for speed)
      if (issue_erase)
        {
          for(i=0;i<LL8;i++ )
           if (blkbuffer[i] != 0xFFFFFFFF) {wflag = 1; break;}
           	
           if(wflag)  ilen = cable_prop->sflash_blkwrite(addr, blkbuffer, ilen,  prg_x8 );  //write block, get actual write
           wflag = 0;
        }
      else ilen = cable_prop->sflash_blkwrite(addr, blkbuffer, ilen,  prg_x8);   // Otherwise we gotta flash it all
      	
      counter += blocksize;
      if(counter >length) counter = length;
      percent_complete = (counter * 100 / length);
      printf("%4d%%   bytes = %d\r", percent_complete, counter);
      fflush(stdout);
      
      addr += blocksize;
//       if (addr - start >64)     
//      exit(1);
    }
//----------
  }
  else
  {
//----------  

    for (addr=start; addr<(start+length); addr+=4)
    {
      counter += 4;
      percent_complete = (counter * 100 / length);
      if (!silent_mode)
        if ((addr&0xF) == 0)  printf("[%3d%% Flashed]   %08X: ", percent_complete, addr);

      fread( (BYTE*) &data, 1,sizeof(data), fd);
      // Erasing Flash Sets addresses to 0xFF's so we can avoid writing these (for speed)
      if (issue_erase)
        {
          if (!(data == 0xFFFFFFFF))
            sflash_write_word_x(addr, data);  //send a word 32bits to write
        }
      else sflash_write_word_x(addr, data);  // Otherwise we gotta flash it all

      if (silent_mode)  printf("%4d%%   bytes = %d\r", percent_complete, counter);
      else              printf("%08X%c", data, (addr&0xF)==0xC?'\n':' ');


      fflush(stdout);
      data = 0xFFFFFFFF;  // This is in case file is shorter than expected length
//      if((addr-start)>0x10)exit(0);
    }
  }
  fclose(fd);
  printf("Done  (%s loaded into Flash Memory OK)\n\n",filename);
  sp_unlock_bypass_reset();

  printf("=========================\n");
  printf("Flashing Routine Complete\n");
  printf("=========================\n");

  time(&end_time);
  elapsed_seconds = difftime(end_time, start_time);
  printf("elapsed time: %d seconds\n", (int)elapsed_seconds);
}


static void run_erase(char *filename, DWORD start, DWORD length)
{
  time_t start_time = time(0);
  time_t end_time, elapsed_seconds;

  printf("*** You Selected to Erase the %s ***\n\n",filename);

  printf("=========================\n");
  printf("Erasing Routine Started\n");
  printf("=========================\n");

  sflash_erase_area(start,length);
  sflash_reset();

  printf("=========================\n");
  printf("Erasing Routine Complete\n");
  printf("=========================\n");

  time(&end_time);
  elapsed_seconds = difftime(end_time, start_time);
  printf("elapsed time: %d seconds\n", (int)elapsed_seconds);
}


static void identify_flash_part()
{
  flash_chip_type*   flash_chip = flash_chip_list;
  flash_area_type*   flash_area = flash_area_list;
  mfr_type* flash_mfr = cfi_mfr_list;

  DWORD topblocksize = 0;
  DWORD bottomsize = size64K;

  // Important for these to initialize to zero
  block_addr  = 0;
  block_total = 0;
  flash_size  = 0;
  cmd_type    = 0;
  PPB_type    = 0;
  strcpy(flash_part,"");

  
  while(flash_mfr->id) {
   if(flash_mfr->id == mfrid) {break;}
   	 else flash_mfr ++;
   	}

  while (flash_chip->venid || (flash_mfr == cfi_mfr_list))
    {
     if ((flash_chip->venid == venid) && (flash_chip->devid == devid))
     {
       flash_size = flash_chip->flash_size;
       cmd_type   = flash_chip->cmd_type;
       PPB_type   = flash_chip->prot_type;
       strcpy(flash_part, flash_chip->flash_part);
       
       if (flash_chip->region4_num) topblocksize = flash_chip->region4_size;
            else if (flash_chip->region3_num) topblocksize = flash_chip->region3_size;
          		 else if (flash_chip->region2_num) topblocksize = flash_chip->region2_size;
          				else if (flash_chip->region1_num) topblocksize = flash_chip->region1_size;
          topblocksize =(topblocksize >= NVRAM_LEN)? topblocksize : NVRAM_LEN;  // get flash top reserved space
          bottomsize = (flash_chip->region1_size > size64K)? flash_chip->region1_size : size64K;
          	
			if (strcasecmp(AREA_NAME,"CUSTOM")==0 )
       {
             
         FLASH_MEMORY_START = selected_window;
         if (!is_probe)
           {
            AREA_START         = selected_start;
            AREA_LENGTH        = selected_length;
            if ((AREA_START < FLASH_MEMORY_START) || 
            	  (AREA_START >= FLASH_MEMORY_START + flash_size  )  ||
             	  (AREA_START + AREA_LENGTH <= FLASH_MEMORY_START )  ||
             	  (AREA_START + AREA_LENGTH > FLASH_MEMORY_START + flash_size))  
            printf("\n\nWarning: Custom area NOT in the FLASH memory window!!!\n\n");  
            strcat(AREA_NAME,".BIN");
           }
        }
			
			 else  // non-custom
			  {
			 	
			    if (BigEndian) 		
					  {                // IF flash size>4MB, re-calculate address and length
					   if(!mpi_base)    //have gotton FLASH_MEMORY_START by reading mpi_reg
					     {if (flash_size > size4MB) FLASH_MEMORY_START = MEM_TOP - flash_size;
                else FLASH_MEMORY_START = 0x1FC00000;}
            } 
          else     //little endian 
            {  
             if(!mpi_base)
               {if (flash_size >= size8MB) FLASH_MEMORY_START = 0x1C000000;   //keep old setting for Wifi chip
                else FLASH_MEMORY_START = 0x1FC00000;}
            }	

          while (flash_area->chip_size)
            {
              if ((flash_area->chip_size == flash_size) && (strcasecmp(flash_area->area_name, AREA_NAME)==0))
                {
                 AREA_START  = FLASH_MEMORY_START + flash_area->area_start;
                 AREA_LENGTH = flash_area->area_length;
                 
                 if(BigEndian)           //BCM63x8, 8MB & 16MB
                 	 {
                   if (strcasecmp(AREA_NAME,"TFE")==0)              //one bottom block size
                      { AREA_START  = FLASH_MEMORY_START;
                        AREA_LENGTH = bottomsize; }
                        
                   else if (strcasecmp(AREA_NAME,"WHOLEFLASH")==0)
                      { AREA_START  = FLASH_MEMORY_START;
                        AREA_LENGTH = flash_size; }
                        
                   else if (strcasecmp(AREA_NAME,"NVRAM")==0)    
                      { AREA_START  = FLASH_MEMORY_START + flash_size - topblocksize; 
                        AREA_LENGTH = topblocksize; }
                   } //end of big endian
 
                 strcat(AREA_NAME,".BIN");
                 break;
                }
               flash_area++;
            } // end while

        }   //non-custom
        
          if (flash_chip->region1_num)  define_block(flash_chip->region1_num, flash_chip->region1_size);
          if (flash_chip->region2_num)  define_block(flash_chip->region2_num, flash_chip->region2_size);
          if (flash_chip->region3_num)  define_block(flash_chip->region3_num, flash_chip->region3_size);
          if (flash_chip->region4_num)  define_block(flash_chip->region4_num, flash_chip->region4_size);

				
          sflash_reset();
          if (!silent_mode)  printf("Matching Flash Chip (VenID:DevID = %04X : %04X)\n\n",venid,devid);
          
          if (selected_fc != 0)
            printf("*** Manually Selected a %s ", flash_part);
          else
            printf("*** Found a %s Flash Chip ", flash_part);

          if (strcasecmp(flash_mfr->name, "")) printf("from %s\n\n",flash_mfr->name ); 

			    if (!is_probe)
				   {
           printf("    - Flash Chip Window Start .... : %08X\n", FLASH_MEMORY_START);
           printf("    - Flash Chip Window Length ... : %08X\n", flash_size);
           printf("    - Selected Area Start ........ : %08X\n", AREA_START);
           printf("    - Selected Area Length ....... : %08X\n\n", AREA_LENGTH);
           }  
          break;
     } // match chid id IF
      flash_chip++;
    } // finish search WHILE

}

// get start address of each block, max block number
static void define_block(DWORD block_count, DWORD block_size)
{
  DWORD  i;

  if (block_addr == 0)  block_addr = FLASH_MEMORY_START;

  for (i = 1; i <= block_count; i++)
    {
      block_total++;
      blocks[block_total] = block_addr;
      block_addr = block_addr + block_size;
    }
  blocks[block_total+1] = block_addr; //get end of flash address+1
}


static void sflash_config(void)
{
  flash_chip_type*   flash_chip = flash_chip_list;
  int counter = 0;

  while (flash_chip->venid)
    {
      counter++;
      if (counter == selected_fc)
        {
          venid = flash_chip->venid;
          devid = flash_chip->devid; 
          mfrid = (venid&0xff00)? (venid>>8) : venid;
          identify_flash_part(0);
          break;
        }
      flash_chip++;
    }

  if (strcasecmp(flash_part,"")==0)
    printf("*** Unknown or NO Flash Chip Selected ***\n");

}

static void sflash_probe(void)
{
  int retries = MAX_ATTEMPTS, i;
  DWORD devid_1, devid_2, devid_3, p_id, p_adr;
  DWORD erase_blk_size[4], erase_blk_count[4], num_blk_regions;
  int bootloc;
  DWORD *tmp;
  WORD chip_ver;
  int found = 0;
  mfr_type* flash_mfr = cfi_mfr_list;

  // Use default flash window for flash probe if not customized the probe address.
  if (strcasecmp(AREA_NAME,"CUSTOM")==0)   
  	    FLASH_MEMORY_START = selected_window;	
      else  if(!mpi_base) FLASH_MEMORY_START = 0x1FC00000;
  	     
  printf("\nProbing Flash at Address: 0x%08X ...\n", FLASH_MEMORY_START);

//---- Probing flash device id ---
      sflash_reset();
   if(PRG_X8)
   	{
      ejtag_write_x(FLASH_MEMORY_START + (AMDUL1 ), 0xaaAAaaAA, MIPS_BYTE);
      ejtag_write_x(FLASH_MEMORY_START + (AMDUL2 ), 0x55555555, MIPS_BYTE);
      ejtag_write_x(FLASH_MEMORY_START + (AMDUL1 ), 0x90909090, MIPS_BYTE);
      mfrid  = ejtag_read_x(FLASH_MEMORY_START, MIPS_BYTE) & 0xFF;      //get manufacture ID
      venid  = mfrid; 
      devid_1 = ejtag_read_x(FLASH_MEMORY_START+2, MIPS_BYTE);    //get Vendor ID cycle 1
      devid = devid_1;
      if (!silent_mode)  printf("Read raw Chip ID (MfrID:DevID = %04X : %04X)\n",venid,devid);
      if ((devid & 0xFF) == 0x7E){     //maybe 3 cycles devid, 0x227E, 0x257E

        devid_2 = ejtag_read_x(FLASH_MEMORY_START+(0x1C), MIPS_BYTE);  // Vendor ID cycle 2
        devid_3 = ejtag_read_x(FLASH_MEMORY_START+(0x1E), MIPS_BYTE);  // Vendor ID cycle 3
        if (!silent_mode)  printf("Read raw sub dev ID (devid2:devid3 = %04X : %04X)\n",devid_2,devid_3);

  	        venid = ((venid & 0xFF) << 8) | 0x7E;
            devid = ((devid_2 & 0xFF) << 8) | (devid_3 & 0xFF);
            if (!silent_mode)   printf("Read final Chip ID (VenID:DevID = %04X : %04X)\n",venid,devid);
      }
    }
    else  // x16 mode
   	{
      ejtag_write_x(FLASH_MEMORY_START + (0x5555 << 1), 0x00AA00AA, MIPS_HALFWORD);
      ejtag_write_x(FLASH_MEMORY_START + (0x2AAA << 1), 0x00550055, MIPS_HALFWORD);
      ejtag_write_x(FLASH_MEMORY_START + (0x5555 << 1), 0x00900090, MIPS_HALFWORD);
      mfrid  = ejtag_read_x(FLASH_MEMORY_START, MIPS_HALFWORD) & 0xFF;      //get manufacture ID
      venid  = mfrid; 
      devid_1 = ejtag_read_x(FLASH_MEMORY_START+2, MIPS_HALFWORD);    //get Vendor ID cycle 1
      devid = devid_1;
      if (!silent_mode)  printf("Read raw Chip ID (MfrID:DevID = %04X : %04X)\n",venid,devid);
      if ((devid & 0xFF) == 0x7E){     //maybe 3 cycles devid, 0x227E, 0x257E

        devid_2 = ejtag_read_x(FLASH_MEMORY_START+(0x0E << 1), MIPS_HALFWORD);  // Vendor ID cycle 2
        devid_3 = ejtag_read_x(FLASH_MEMORY_START+(0x0F << 1), MIPS_HALFWORD);  // Vendor ID cycle 3
        if (!silent_mode)  printf("Read raw sub dev ID (devid2:devid3 = %04X : %04X)\n",devid_2,devid_3);
  
        if(((devid_2 & 0xFF00) ==(devid & 0xFF00) ) && ((devid_3 & 0xFF00) == (devid & 0xFF00) )){
  	        venid = ((venid & 0xFF) << 8) | 0x7E;
            devid = ((devid_2 & 0xFF) << 8) | (devid_3 & 0xFF);
            if (!silent_mode)   printf("Read final Chip ID (VenID:DevID = %04X : %04X)\n",venid,devid);
	       }
      }
    }    	  
   	  printf("Detected Chip ID (VenID:DevID = %04X : %04X)\n",venid,devid);
   	  sflash_reset();

//--- Getting flash geometry with CFI query ---

 if (issue_cfi_qry && cfi_qry_request()){
      cfi_read_array((WORD*)&cfi_qry_data, 0x10, sizeof(cfi_qry_data)/CFI_QRY_BUSWIDTH);

  if (LL4 == 0xFFFF) 
    {
      if(cfi_qry_data.WordWriteTimeoutTyp >= 7) 
      	 LL4 = 8;
      else if(cfi_qry_data.WordWriteTimeoutTyp < 4) 
         LL4 = (1 << cfi_qry_data.WordWriteTimeoutTyp)*1.2;
     }
    
//    printf("flash program timeout = %d\n", flsPrgTimeout);
    p_id = cfi_qry_data.P_ID[0] + (cfi_qry_data.P_ID[1]<<8 );
    p_adr = cfi_qry_data.P_ADR[0] + (cfi_qry_data.P_ADR[1] <<8);
    bootloc = BOT_BOOT; 
 /*
  Intel - no need flip erase block geometry
  SST   - most are uniform erase block, no need flip
  AMD   - Top Boot need flip
 */
	if (!silent_mode)  printf("\nFlash CommandSet = %04X\n", p_id);
	switch(p_id){

	   case P_ID_INTEL_EXT:
	   case P_ID_INTEL_STD:
//	   case P_ID_INTEL_PERFORMANCE:
	      cmd_type = CMD_TYPE_BCS;
	      if(p_adr) {
	      	cfi_read_array((WORD*) &cfi_pri_data, p_adr, sizeof(cfi_pri_data)/CFI_QRY_BUSWIDTH); 
	      	if( cfi_pri_data.pri[0]!= 'P' ||
	      		  cfi_pri_data.pri[1]!= 'R' ||
	      		  cfi_pri_data.pri[2]!= 'I' ) goto cfi_exit;
	      	chip_ver = (cfi_pri_data.MajorVersion << 8)|cfi_pri_data.MinorVersion;
          if (chip_ver <0x3130 ) goto cfi_exit;
         chip_features = cfi_pri_data.Features;
         if (!silent_mode)  printf("\nFlash Full Chip Erase Supported: %s\n", (chip_features & FULL_CHIP_ERASE)?"Yes" : "No");
        }else goto cfi_exit;
	      break;

	   case P_ID_AMD_STD:
		    cmd_type = CMD_TYPE_AMD;
	      if(p_adr) {
	      	cfi_read_array((WORD*) &cfi_amd_pri_data, p_adr, sizeof(cfi_amd_pri_data)/CFI_QRY_BUSWIDTH); 
	      	if( cfi_amd_pri_data.pri[0]!= 'P' ||
	      		  cfi_amd_pri_data.pri[1]!= 'R' ||
	      		  cfi_amd_pri_data.pri[2]!= 'I' ) goto cfi_exit;
	      	chip_ver = (cfi_amd_pri_data.MajorVersion << 8)|cfi_amd_pri_data.MinorVersion;
          if (chip_ver <0x3130 ) goto cfi_exit;  
          else if (chip_ver == 0x3130 ) bootloc = (devid_1&0x80)? TOP_BOOT : BOT_BOOT;
          else
	      	  bootloc = cfi_amd_pri_data.TopBottom;
	      	if (!silent_mode)  printf("\nFlash Boot Block Location: %s\n", (bootloc==TOP_BOOT)?"TOP" : "Non TOP");
        } else 
        	if(mfrid == MFR_SST) {bootloc = (devid_1&1)? BOT_BOOT : TOP_BOOT;}
        		else goto cfi_exit;
        if(cfi_amd_pri_data.BlkProtUnprot==0x08) flash_chip_list[0].prot_type = SP_PPB;
	      break;

     case P_ID_SST:
        cmd_type = CMD_TYPE_SST;
        break;

     default:
	     goto cfi_exit;
    }

	for(i=0; i<4;i++) {erase_blk_size[i] =0; erase_blk_count[i] = 0;}
		
  flash_chip_list[0].flash_size =((DWORD)1) << cfi_qry_data.DevSize;
  flash_chip_list[0].cmd_type   = cmd_type;
  flash_chip_list[0].venid      = venid;
  flash_chip_list[0].devid      = devid;
  
  if (!silent_mode)  printf("Flash total size: %dKB (%dMB)\n", (flash_chip_list[0].flash_size>>10),(flash_chip_list[0].flash_size>>20) );
  
  num_blk_regions = cfi_qry_data.NumEraseRegions;
  if(mfrid == MFR_SST){        //SST:  use block erase info instead of sector erase info
  	 -- num_blk_regions;       
  	 if(num_blk_regions < 4)
  	 for (i=0;i<num_blk_regions;i++){
  	   cfi_qry_data.EraseRegionInfo[i][0]=cfi_qry_data.EraseRegionInfo[i+1][0];
  	   cfi_qry_data.EraseRegionInfo[i][1]=cfi_qry_data.EraseRegionInfo[i+1][1];
  	   cfi_qry_data.EraseRegionInfo[i][2]=cfi_qry_data.EraseRegionInfo[i+1][2];
  	   cfi_qry_data.EraseRegionInfo[i][3]=cfi_qry_data.EraseRegionInfo[i+1][3];
  	  }
   }
    
    for(i=0; i < num_blk_regions; i++) {
        erase_blk_count[i] = cfi_qry_data.EraseRegionInfo[i][0] 
                           + (cfi_qry_data.EraseRegionInfo[i][1] <<8 ) + 1;
        erase_blk_size[i]  = (cfi_qry_data.EraseRegionInfo[i][2] <<8 )
                           + (cfi_qry_data.EraseRegionInfo[i][3] <<16);
    }
    
   tmp = &flash_chip_list[0].region1_num; 
  for (i=0; i < num_blk_regions; i++) {
    if((num_blk_regions >1) && (force_flip || (!force_noflip && bootloc == TOP_BOOT ))){   //Top Boot flipping
      *(tmp + 2*i)     = erase_blk_count[num_blk_regions - 1 - i];
 	    *(tmp + 2*i + 1) = erase_blk_size[num_blk_regions - 1 - i];
    } else {   
      *(tmp + 2*i)     = erase_blk_count[i];
  	  *(tmp + 2*i + 1) = erase_blk_size[i];
    }
  }

  found = 0;
  while(flash_mfr->id) {
   if(flash_mfr->id == mfrid) {found = 1; break;}
   	 else flash_mfr ++;
   	}
  if(found == 0)
  	{
  	 cfi_mfr_list[0].id = mfrid;
  	 flash_chip_list[0].venid = venid;
     flash_chip_list[0].devid = devid;
    }
   	
  if(!silent_mode) {
 	 printf("Flash has total %d erase block regions\n", num_blk_regions );
 	 for (i=0; i < num_blk_regions; i++) printf(" Region[%1d] sector count:%4d, \tper size:%3dKB\n", i, *(tmp+2*i),*(tmp+2*i+1)>>10 );
 	 printf("Complete CFI Query!!\n\n");
  }
 } // end of cfi query

cfi_exit:
  sflash_reset();      //exit CFIQRY
  identify_flash_part(found);
  if (strcasecmp(flash_part,"")==0)
      printf("*** Unknown or NO Flash Chip Detected ***\n");
   
}

static void cfi_read_array(WORD *ptr, int ofs, int length)
{
	int i;
	if(!silent_mode) printf("\n\nRead Array Starting from offset [0x%04X]\n", ofs);
	for (i=0; i<length; i++){
	 ptr[i] = ejtag_read_x(FLASH_MEMORY_START+((ofs+i) << 1), MIPS_HALFWORD)&0xFF; 
	 if(!silent_mode) printf("Array[0x%02X] = 0x%04X\n", ofs+i, ptr[i]); 
	}
}

static int cfi_qry_request()
{
	WORD  qry[3];
// 1. AMD, Intel
  ejtag_write_x(FLASH_MEMORY_START, 0xffFFffFF&PGMASK, PGMODE);    //Intel reset
  ejtag_write_x(FLASH_MEMORY_START, 0xf0F0f0F0&PGMASK, PGMODE);    //AMD reset
  ejtag_write_x(FLASH_MEMORY_START, 0xf0F0f0F0&PGMASK, PGMODE);    //Double AMD reset for ST exit CFIQRY
  ejtag_write_x(FLASH_MEMORY_START+AMDCFI, 0x98989898&PGMASK, PGMODE);  //CFIQRY,ST
	cfi_read_array((WORD*)&qry, 0x10, 3);	
   if( qry[0] == 'Q' &&
       qry[1] == 'R' &&
       qry[2] == 'Y') return 1;    //keep in CFIQRY status
		
// 2. AMD
  ejtag_write_x(FLASH_MEMORY_START, 0x00F000F0&PGMASK, PGMODE);    //AMD reset
  ejtag_write_x(FLASH_MEMORY_START, 0x00F000F0&PGMASK, PGMODE);    //Double AMD reset for ST exit CFIQRY
  ejtag_write_x(FLASH_MEMORY_START+AMDCFI, 0x98989898&PGMASK, PGMODE);  //CFIQRY
	cfi_read_array((WORD*)&qry, 0x10, 3);	
   if( qry[0] == 'Q' &&
       qry[1] == 'R' &&
       qry[2] == 'Y') return 1;    //keep in CFIQRY status

// 3. Intel
  ejtag_write_x(FLASH_MEMORY_START, 0xffFFffFF&PGMASK, PGMODE);    //Intel reset
  ejtag_write_x(FLASH_MEMORY_START, 0x98989898&PGMASK, PGMODE);  //CFIQRY
	cfi_read_array((WORD*)&qry, 0x10, 3);	
   if( qry[0] == 'Q' &&
       qry[1] == 'R' &&
       qry[2] == 'Y') return 1;    //keep in CFIQRY status

// 4. SST

  ejtag_write_x(FLASH_MEMORY_START+(0x5555 << 1), 0x00AA00AA, MIPS_HALFWORD);  //unlock
  ejtag_write_x(FLASH_MEMORY_START+(0x2AAA << 1), 0x00550055, MIPS_HALFWORD);  
  ejtag_write_x(FLASH_MEMORY_START+(0x5555 << 1), 0x00F000F0, MIPS_HALFWORD);  //reset
  ejtag_write_x(FLASH_MEMORY_START+(0x5555 << 1), 0x00AA00AA, MIPS_HALFWORD);  //unlock
  ejtag_write_x(FLASH_MEMORY_START+(0x2AAA << 1), 0x00550055, MIPS_HALFWORD);  
  ejtag_write_x(FLASH_MEMORY_START+(0x5555 << 1), 0x00980098, MIPS_HALFWORD);  //CFIQRY
	cfi_read_array((WORD*)&qry, 0x10, 3);	
   if( qry[0] == 'Q' &&
       qry[1] == 'R' &&
       qry[2] == 'Y') return 1;    //keep in CFIQRY status
		
return 0;		
}

static int sflash_erase_area(DWORD start, DWORD length)
{
  int cur_block;
  int tot_blocks;
  DWORD reg_start;
  DWORD reg_end;         
  int block_start=0;
  int block_end=0;
  int align_start=0;
  int align_end=0;
  DWORD block_addr1, block_addr2;
 
  reg_start = start;
  reg_end   = reg_start + length;   //the end +1

  tot_blocks = 0;

  for (cur_block = 1;  cur_block <= block_total;  cur_block++)
    {
      block_addr1 = blocks[cur_block];
      block_addr2 = blocks[cur_block+1];  //current block end +1
     if ((reg_start >= block_addr1) && (reg_start < block_addr2)) block_start = cur_block;
     if ((block_start>0) && (reg_end >= block_addr1) && (reg_end < block_addr2)) block_end = cur_block;	
     if (reg_start == block_addr1) align_start = 1;
     if ((length>0) && (reg_end == block_addr2)) {align_end = 1; block_end = cur_block+1;}
    }
// printf("Erase start from sect %8d to sect %8d\n", block_start, block_end);   
    
  if (block_start == 0 || block_end == 0 )
  	{ printf("***ERROR: No blocks need to operation or Window setting wrong!!\n");
  	goto go_err; }
  	
  if ((!force_align)&& ((!align_start) || (!align_end)))
  	{ printf("***ERROR: Erase or Program area doesn't align with flash sector boundary!\n"
  				   "          Modify operation area or use /forcealign option.\n");
  	goto go_err;	}

 if (!check_align)
 	{  	
  if(align_end) tot_blocks = block_end - block_start;
  	 else tot_blocks = block_end - block_start + 1;
  if (issue_erase)  printf("Total Blocks to Erase: %d\n\n", tot_blocks);
 	   else printf("Total Blocks to unlock: %d\n\n", tot_blocks);

  for (cur_block = block_start;  cur_block < block_start + tot_blocks;  cur_block++)
   {
     block_addr1 = blocks[cur_block];
     if (issue_erase) printf("Erasing block: %d (addr = %08X)...", cur_block, block_addr1);
     	 else printf("Unlocking block: %d (addr = %08X)...", cur_block, block_addr1);
     fflush(stdout);
     sflash_erase_block(block_addr1);
     mssleep(100);
     printf("Done\n");
     fflush(stdout);
    }
   } //check align
    return 0;    

go_err:
    return -1;
}


static void sflash_erase_block(DWORD addr)
{

  if (cmd_type == CMD_TYPE_AMD)
    {
      sflash_reset();
      //Unlock Block
      ejtag_write_x(FLASH_MEMORY_START+AMDUL1, 0xaaAAaaAA&PGMASK, PGMODE);
      ejtag_write_x(FLASH_MEMORY_START+AMDUL2, 0x55555555&PGMASK, PGMODE);
      ejtag_write_x(FLASH_MEMORY_START+AMDUL1, 0x80808080&PGMASK, PGMODE);

      //Erase Block
      ejtag_write_x(FLASH_MEMORY_START+AMDUL1, 0xaaAAaaAA&PGMASK, PGMODE);
      ejtag_write_x(FLASH_MEMORY_START+AMDUL2, 0x55555555&PGMASK, PGMODE);
      ejtag_write_x(addr, 0x30303030&PGMASK, PGMODE);


      // Wait for Erase Completion
        if(PRG_X8)
        	   sflash_poll_x8(addr, 0xFF);
        	else
             sflash_poll_x16(addr, 0xFFFF);
    }

  else if (cmd_type == CMD_TYPE_SST)
    {
      //Unlock Block
      ejtag_write_x(FLASH_MEMORY_START+(0x5555 << 1), 0x00AA00AA, MIPS_HALFWORD);
      ejtag_write_x(FLASH_MEMORY_START+(0x2AAA << 1), 0x00550055, MIPS_HALFWORD);
      ejtag_write_x(FLASH_MEMORY_START+(0x5555 << 1), 0x00800080, MIPS_HALFWORD);

      //Erase Block
      ejtag_write_x(FLASH_MEMORY_START+(0x5555 << 1), 0x00AA00AA, MIPS_HALFWORD);
      ejtag_write_x(FLASH_MEMORY_START+(0x2AAA << 1), 0x00550055, MIPS_HALFWORD);
      ejtag_write_x(addr, 0x00500050, MIPS_HALFWORD);

      // Wait for Erase Completion
      sflash_poll_x16(addr, 0xFFFF);

    }

  else if ((cmd_type == CMD_TYPE_BCS) || (cmd_type == CMD_TYPE_SCS))
    {
      //Unlock Block
      ejtag_write_x(addr, 0x50505050&PGMASK, PGMODE);     // Clear Status Command
      ejtag_write_x(addr, 0x60606060&PGMASK, PGMODE);     // Unlock Flash Block Command
      ejtag_write_x(addr, 0xd0D0d0D0&PGMASK, PGMODE);     // Confirm Command

      // Wait for Unlock Completion
      if(PRG_X8)             
       	   sflash_poll_x8(addr, STATUS_READY);  
	   else                                                    
	        sflash_poll_x16(addr, STATUS_READY);
	  //Erase Block
      ejtag_write_x(addr, 0x50505050&PGMASK, PGMODE);     // Clear Status Command
      ejtag_write_x(addr, 0x20202020&PGMASK, PGMODE);     // Block Erase Command
      ejtag_write_x(addr, 0xd0D0d0D0&PGMASK, PGMODE);     // Confirm Command

      // Wait for Erase Completion
      if(PRG_X8)                                                
        	   sflash_poll_x8(addr, STATUS_READY);  
       	else                                                    
             sflash_poll_x16(addr, STATUS_READY);

    }

  sflash_reset();

}


static void sflash_reset(void)
{

  if (cmd_type == CMD_TYPE_AMD)
    {
      ejtag_write_x(FLASH_MEMORY_START, 0xf0F0f0F0&PGMASK, PGMODE);    // Set array to read mode
      ejtag_write_x(FLASH_MEMORY_START, 0xf0F0f0F0&PGMASK, PGMODE);    // twice for ST flash quit from CFIQRY
    }
  else if (cmd_type == CMD_TYPE_SST)
    {
      ejtag_write_x(FLASH_MEMORY_START, 0x00F000F0, MIPS_HALFWORD);    // Set array to read mode
      ejtag_write_x(FLASH_MEMORY_START, 0x00F000F0, MIPS_HALFWORD);    // twice for ST flash quit from CFIQRY
    }  

  else if ((cmd_type == CMD_TYPE_BCS) || (cmd_type == CMD_TYPE_SCS))
    {
      ejtag_write_x(FLASH_MEMORY_START, 0x50505050&PGMASK, PGMODE);    // Clear CSR
      ejtag_write_x(FLASH_MEMORY_START, 0xffffffff&PGMASK, PGMODE);    // Set array to read mode
    }

}

static void sflash_write_word_x(DWORD addr, DWORD data)
{

  if(PGMODE == MIPS_HALFWORD)
  	{
  		sflash_write_x16((addr & (~3)), data);
  		sflash_write_x16((addr & (~3))+ 2, data);
  	}
  else if(PGMODE == MIPS_BYTE)
   	{
  		sflash_write_x8((addr & (~3)), data);
  		sflash_write_x8((addr & (~3))+ 1, data);
  		sflash_write_x8((addr & (~3))+ 2, data);
  		sflash_write_x8((addr & (~3))+ 3, data);
  	}
  else
  	{
  		printf("\n***ERROR: unsupported write data type!!! exit...\n");
      exit(1);
    }
  
}


#define LEMASK16(k)   (0xffff<<(16*(k)))
#define BEMASK16(k)   (0xffff<<(16*(1-(k))))

static void sflash_write_x16(DWORD addr, DWORD data)
{
  int k;
  DWORD odata,ldata;
  
  k = (addr & 0x2)>>1;
  
       if (BigEndian)
       	  {
    	     odata = rev_endian(data) & BEMASK16(k);
    	     ldata = odata >>(16*(1-k));
    	    }
        else
          {
           odata = data & LEMASK16(k);
           ldata = odata >>(16*k);
          }

  if( ldata == 0xffff) return; // no need to program
  if (USE_DMA == 0) odata = ldata;  //pracc
 
//  printf("write x16 to %08x , %08x, %08x\n", addr, odata, ldata); 	
     switch( cmd_type )
       {
        case CMD_TYPE_AMD:
        	if (!bypass)
        		{
             ejtag_write_x(FLASH_MEMORY_START+AMDUL1, 0x00AA00AA,MIPS_HALFWORD);
             ejtag_write_x(FLASH_MEMORY_START+AMDUL2, 0x00550055,MIPS_HALFWORD);
            }
          ejtag_write_x(FLASH_MEMORY_START+AMDUL1, 0x00A000A0,MIPS_HALFWORD);
          break;
        case CMD_TYPE_SST:
          ejtag_write_x(FLASH_MEMORY_START+(0x5555 << 1), 0x00AA00AA,MIPS_HALFWORD);
          ejtag_write_x(FLASH_MEMORY_START+(0x2AAA << 1), 0x00550055,MIPS_HALFWORD);
          ejtag_write_x(FLASH_MEMORY_START+(0x5555 << 1), 0x00A000A0,MIPS_HALFWORD);
          break;
        case CMD_TYPE_BCS:
        case CMD_TYPE_SCS:
        default:
          ejtag_write_x(addr, 0x00500050,MIPS_HALFWORD);   // Clear Status Command
          ejtag_write_x(addr, 0x00400040,MIPS_HALFWORD);   // Write Command
       }
 
          ejtag_write_x(addr, odata,MIPS_HALFWORD);

    switch( cmd_type )
       {
        case CMD_TYPE_AMD:
        	 if (bypass) { ussleep(16); break;}
        case CMD_TYPE_SST:

           sflash_poll_x16(addr, ldata);
           break;

        case CMD_TYPE_BCS:
        case CMD_TYPE_SCS:
        default:           		
            ejtag_write_x(addr, 0x00700070,MIPS_HALFWORD);   // Check Status Command
            sflash_poll_x16(addr, STATUS_READY);   
       }       
}


static void sflash_poll_x16(DWORD addr, DWORD data)
{
  int timeout = 10;   //16M need 5300 for full chip erase
  DWORD onpoll;
  if ((cmd_type == CMD_TYPE_BCS) || (cmd_type == CMD_TYPE_SCS))
    {
      // Wait Until Ready
      onpoll = ejtag_read_x(FLASH_MEMORY_START,MIPS_HALFWORD);
      while ( ( onpoll & STATUS_READY) != STATUS_READY )
      {
      	onpoll = ejtag_read_x(FLASH_MEMORY_START,MIPS_HALFWORD);
      	if((data !=0xFFFF) && (onpoll == data)) 
			break;
      	if((timeout--) && (data !=0xFFFF)) 
			break;
		if ( cable_type == FT2232H) 
			mssleep(500);
      }
    }
  else
    {
      onpoll = ejtag_read_x(addr,MIPS_HALFWORD);
      // Wait Until Ready
      while ( ( onpoll& STATUS_READY) != (data & STATUS_READY) )
      {
      	onpoll = ejtag_read_x(addr,MIPS_HALFWORD);
      	if ((data !=0xFFFF) && (onpoll == data)) break;
      	if ((timeout--) && (data !=0xFFFF)) break;
      }
    }
}



#define LEMASK8(k)    (0xff<<(8*(k)))
#define BEMASK8(k)    (0xff<<(8*(3-(k))))  
 
static void sflash_write_x8(DWORD addr, DWORD data)
{
  int k;
  DWORD odata,ldata;
  
  k = addr & 0x3;

/* spec-ed implementation, but seem not work
 *
 *      if (BigEndian)
 *      	  {
 *   	     odata = rev_endian(data) & BEMASK8(k);
 *   	     ldata = odata >> (8*(3-k));
 *   	    }
 *       else
 *         {
 *          odata = data & LEMASK8(k);
 *          ldata = odata >> (8*k);
 *         }
 */         

// a work around implementation.
   
   ldata = (data >> (8*k))&0xFF;
   odata = ldata | (ldata <<8);
   odata |= (odata<<16);
     
//     printf(" I am in x8, addr, data = %08x, %08x, %08x %08x\n", addr, data,odata,ldata);  	     
     if( ldata == 0xff) return; // no need to program     
     	
     if (USE_DMA == 0) odata = ldata;  //pracc
     	
     switch( cmd_type )
       {
        case CMD_TYPE_AMD:
        	if (!bypass)
        		{
             ejtag_write_x(FLASH_MEMORY_START+AMDUL1, 0xAAAAAAAA,MIPS_BYTE);
             ejtag_write_x(FLASH_MEMORY_START+AMDUL2, 0x55555555,MIPS_BYTE);
            }
             ejtag_write_x(FLASH_MEMORY_START+AMDUL1, 0xA0A0A0A0,MIPS_BYTE);
          break;
        case CMD_TYPE_SST:
        	return;  //SST 39 don't support x8
          break;
        case CMD_TYPE_BCS:
        case CMD_TYPE_SCS:
        default:
          ejtag_write_x(addr, 0x50505050,MIPS_BYTE);   // Clear Status Command
          ejtag_write_x(addr, 0x40404040,MIPS_BYTE);   // Write Command
       }
 
          ejtag_write_x(addr, odata,MIPS_BYTE);

    switch( cmd_type )
       {
        case CMD_TYPE_AMD:
        	 if (bypass) { ussleep(16); break;}
        
        case CMD_TYPE_SST:
           sflash_poll_x8(addr, ldata);
           break;

        case CMD_TYPE_BCS:
        case CMD_TYPE_SCS:
        default:           		
            ejtag_write_x(addr, 0x70707070,MIPS_BYTE);   // Check Status Command
            sflash_poll_x8(addr, STATUS_READY);   
       }       
}


static void sflash_poll_x8(DWORD addr, DWORD data)
{
  int timeout = 10;   //16MB need 5300 for full chip erase
  DWORD onpoll;
  if ((cmd_type == CMD_TYPE_BCS) || (cmd_type == CMD_TYPE_SCS))
    {
      // Wait Until Ready
      onpoll = ejtag_read_x(FLASH_MEMORY_START,MIPS_BYTE);
      while ( ( onpoll & STATUS_READY) != STATUS_READY )
      {
      	onpoll = ejtag_read_x(FLASH_MEMORY_START,MIPS_BYTE);
      	if((data !=0xFF) && (onpoll == data)) break;
      	if((timeout--) && (data !=0xFF)) break;
      }
    }
  else
    {
      onpoll = ejtag_read_x(addr,MIPS_BYTE);
      // Wait Until Ready
      while ( ( onpoll& STATUS_READY) != (data & STATUS_READY) )
      {
      	onpoll = ejtag_read_x(addr,MIPS_BYTE);
      	if ((data !=0xFF) && (onpoll == data)) break;
      	if ((timeout--) && (data !=0xFF)) break;
      }
    }
}


static void show_flashlist()
{

  flash_chip_type*      flash_chip = flash_chip_list;
  int counter = 0;
  printf( "    Build-in flash list\n" 
          "    -------------------\n");

   flash_chip++;  //bypass the first
  ++counter ; 
  while (flash_chip->venid)
    {
      printf("    /fc:%03d (%04X %04X) ........... %-40.40s\n", ++counter,
                      flash_chip->venid, flash_chip->devid, flash_chip->flash_part);
      flash_chip++;
    } 
   printf( "    ----------------------------------------------------------------\n"  );
   printf( "            Total %d flash in list\n\n", counter-1  );

   printf( "              1-Cycle FLASH ID (00MM XXXX)\n"
           "              3-Cycle FLASH ID (MMXX YYZZ)\n\n");

   printf( "      Use switch '/fc:XXX' if you want to manually select Flash Chip.\n"
           "      It will disable CFI auto detect and non CFI flash ID auto match.\n"
           "      use 'zjtag /showflashlist' show this list\n\n");

}

static void show_usage()
{

  flash_chip_type*      flash_chip = flash_chip_list;
  processor_chip_type*  processor_chip = processor_chip_list;
  int counter = 0;

  printf( " ABOUT: This program reads/writes flash memory on the Broadcom MIPS(LE)\n"
          "        Chip and compatible routers via EJTAG using either DMA Access\n"
          "        routines or PrAcc routines (slower/more compatible). Processor chips\n"
          "        supported in this version include the following chips:\n\n"
          "            Supported Chips\n"
          "            ---------------\n");

  while (processor_chip->chip_id || processor_chip->instr_length)
    {
      printf("            %s", processor_chip->mfrdscr_idx->name);
      printf(" %-40.40s\n", processor_chip->chip_descr);
      processor_chip++;
    }

  printf( "\n            Supported Cable Types\n"
          "            ---------------\n"
          "            ID\tCable Name\n");

  counter = 0;
  while (cable_list[counter].id !=-1)
    {
      printf("            %d\t", counter);
      printf(" %-55.55s\n", cable_list[counter].name);
      counter++;
    }

  printf( "\n\n");
  printf( " USAGE: zjtag /showflashlist\n");
  printf( " USAGE: zjtag [parameter] </noreset> </noemw> </nocwd> </nobreak></LE|BE>\n"
          "                      </notimestamp> </dma> </nodma> </noerase></initcpu>\n"
          "                      </nompi> </ejslow></waitbrk></srst><wx8>\n"          
          "                      </bypass></forcealign></showppb></clearppb></erasechip>\n"          
		  "                      </nocfi></forcenoflip></forceflip></delay:xxx>\n"
          "                      <window:XXXXXXXX><start:XXXXXXXX> </length:XXXXXXXX>\n"
          "                      <port:XXX> </instrlen:XX> </fc:XX></skipdetect>\n"
		  "                      </cablefta></cableblackcat></wiggler></cable:X>\n"
          "                      </io2></safemode></verbose></debug>\n\n"
          "            Required Parameter\n"
          "            ------------------\n"
          "            -backup:cfe\n"
          "            -backup:tfe\n"
          "            -backup:nvram\n"
          "            -backup:kernel\n"
          "            -backup:wholeflash\n"
          "            -backup:custom\n"
          "            -backup:bsp\n"
          "            -erase:cfe\n"
          "            -erase:tfe\n"
          "            -erase:nvram\n"
          "            -erase:kernel\n"
          "            -erase:wholeflash\n"
          "            -erase:custom\n"
          "            -erase:bsp\n"
          "            -flash:cfe\n"
          "            -flash:tfe\n"
          "            -flash:nvram\n"
          "            -flash:kernel\n"
          "            -flash:wholeflash\n"
          "            -flash:custom\n"
          "            -flash:bsp\n"
          "            -probeonly\n\n"

          "            Optional Switches\n"
          "            -----------------\n"
          "            /noreset ........... prevent Issuing EJTAG CPU reset\n"
          "            /noemw ............. prevent Enabling Memory Writes\n"
          "            /nocwd ............. prevent Clearing CPU Watchdog Timer\n"
          "            /nobreak ........... prevent Issuing Debug Mode JTAGBRK\n"
          "            /noerase ........... prevent Forced Erase before Flashing\n"
          "            /notimestamp ....... prevent Timestamping of Backups\n"
          "            /dma ............... force use of DMA routines\n"
          "            /srst .............. force a TAP nSRST reset on starting\n"
          "            /nodma ............. force use of PRACC routines (No DMA)\n"
          "            /ejslow............. with low speed ejtag access\n"
          "            /waitbrk............ wait until CPU enter debug mode\n"
          "            /wx8, /byte_mode ... with x8 mode program flash\n"
          "            /initcpu............ load CPU configuration code\n"
          "            /nompi.............. skip autodect flash base address with MPI Reg\n"
          "            /LE ................ force operate as Little Endian chip\n"
          "            /BE ................ force operate as Big Endian chip\n"          
          "            /window:XXXXXXXX ... custom flash window base&probe address(in HEX)\n"
          "            /start:XXXXXXXX .... custom start location (in HEX)\n"
          "            /length:XXXXXXXX ... custom length (in HEX)\n"
          "            /verbose............ scrolling display of data\n"
          "            /debug.............. print debug messages\n"
          "            /skipdetect ........ skip auto detection of CPU Chip ID\n"
          "            /instrlen:XX ....... set CPU instruction length manually\n"
          "            /cablefta .......... use TIAO All In One / 10 / 20 PIN FTA cable\n"
		  "            /cableblackcat ..... use TIAO Blackcat cable modem JTAG cable\n"
          "            /wiggler ........... use wiggler cable\n"
          "            /nocfi ............. disable CFI query flash geometry\n"
          "            /forcenoflip ....... force not flipping CFI queried flash geometry\n"
          "            /forceflip ......... force flipping CFI queried flash geometry\n"
          "            /bypass ............ unlock Spansion bypass mode & disable polling\n"
          "            /forcealign......... force erase address align with block boundary\n"         
          "            /erasechip.......... erase whole chip, only work with -probeonly \n"
          "            /clearppb........... erase Spansion PPB,only work with -probeonly\n"
          "            /showppb ........... show flash sector protection status\n"
          "                                 only work with -probeonly \n"
          "            /port:XXX........... customize parallel port(default XXX is 378)\n"
          "                                 only work in Windows version\n"
		  "            /delay:XXX.......... Delay XXX ms in DMA write mode (only for TUMPA)\n"
		  "            /L1:<divider>....... JTAG clock speed divider (only for TUMPA)\n"
#ifdef WINDOWS_VERSION
          "            /io2 ............... use alternative Parallel port access method\n"          
#endif
          "            /cable:x ........... select cable type, x = cable type ID\n"
		  "                        x = 0 (default): %-55.55s\n", cable_list[0].name);
  counter = 1;
  while (cable_list[counter].id !=-1)
    {
      printf( "                        x = %d : %-55.55s\n", counter, cable_list[counter].name);
      counter++;
    }
  printf( "            /safemode .......... use parallel cable way operate USB, SLOW!\n"
          "            /fc:XXX = Manual Flash Chip Selection,disable CFI and ID auto match\n"
          "                      use 'zjtag /showflashlist' show build-in flash list\n"
          "            \n");
#if 0     // defualt no show flash list since cfi auto-detect is used
  flash_chip++;  //bypass the first
  ++counter ; 
  while (flash_chip->venid)
    {
      printf("            /fc:%02d ............. %-40.40s\n", ++counter, flash_chip->flash_part);
      flash_chip++;
    }
#endif

  printf( "\n\n");
  printf( " NOTES: *) '-backup:', '-flash:' and '-erase:', the source filename must exist\n"
          "           as follows: CFE.BIN, NVRAM.BIN, KERNEL.BIN, WHOLEFLASH.BIN or\n"
          "           CUSTOM.BIN, BSP.BIN, TFE.BIN(64KB or 1x bottom Sector length CFE)\n\n"

          "        *) zjtag defualt with x16 mode handle Parallel Flash chip. /wx8 switch\n"
          "           to x8 mode.\n\n"

          "        *) zjtag uses CFI command set to automatically detect flash chip\n"
          "           parameters. If you have difficulty auto-detecting flash with CFI,\n"
          "           '/nocfi' convert to original flash detection method. zjtag then use\n"
          "           detected flash ID query parameters from build-in flash list. \n"
          "           particularly, you can use '/fc:XX' manually specify flash ID.\n"
          "           'zjtag /showflashlist' can print build-in flash list \n\n"

          "        *) '/forcenoflip' and '/forceflip' can help on some AMD type flash\n"
          "           detecting sector structure correctly if CFI uses.\n"
          "           'zjtag -probeonly /verbose' debug flash detection\n\n"
          
          "        *) If you have difficulty with the older bcm47xx chips or when no CFE\n"
          "           is currently active/operational you may want to try both the\n"
          "           /noreset and /nobreak command line options together.  Some bcm47xx\n"
          "           chips *may* always require both these options to function properly.\n\n"

          "        *) When using this utility, usually it is best to type the command line\n"
          "           out, then power up the router, about 0.5 second delay, hit <ENTER> \n"
          "           quickly to avoid bad CFE code lead to <CPU NOT enter Debug mode> \n"
          "           or the CPUs watchdog interfering with the EJTAG operations.\n\n"

          "        *) /bypass - enables Unlock bypass command for some AMD/Spansion type\n"
          "           flashes, it also disables polling\n\n"

          "        *) /initcpu allow load config code to initialize the CPU. This may help\n"
          "           BCM6358 prevent from some address non-accessible.\n\n"

          "        *) '-probeonly /window:xxxxxxxx /erasechip' allow choose a workable\n"
          "           sector address to erase whole chip. This may help on a bricked box\n"
          "           with bad CFE\n\n"
          
          "        *) /forcealign - enable erase sectors if the operation window is not\n"
          "           aligned with sector boundary. It's risky! but can help erase some\n"
          "           box NVRAM area whose sector size is larger than NVRAM definition\n\n" 

          "        *) /ejslow - limit parallel port clock out speed to 500KHz. This wish\n"
          "           to increase LPT port compatibility for some high clock PC.\n"
          "           For USB cable this switch can help hit higher clock\n\n"

		  "        *) /delay - Pause between DMA write actions.  Use this if you could not\n"
		  "           get a reliable write or erase result.  If not specified, default delay\n"
		  "           is 50 ms.  This option only works with TUMPA USB adapter.\n\n"
       


          " ***************************************************************************\n"
          " * Flashing the KERNEL or WHOLEFLASH will take a very long time using JTAG *\n"
          " * via this utility.  You are better off flashing the CFE & NVRAM files    *\n"
          " * & then using the normal TFTP method to flash the KERNEL via ethernet.   *\n"
          " ***************************************************************************\n\n");
}

static void chip_filllist()
{

  processor_chip_type*  processor_chip = processor_chip_list;
  cpumfr_type* cmfr_dscr = cpu_mfr_list;
  int counter = 0;

  while (processor_chip->chip_id || processor_chip->instr_length)
    {
    	while(cmfr_dscr->id || cmfr_dscr->name){
       if(cmfr_dscr->id == processor_chip->chip_mfrid) {processor_chip->mfrdscr_idx = cmfr_dscr;break;}
       cmfr_dscr++;
      }
      if(!(cmfr_dscr->id || cmfr_dscr->name)) processor_chip->mfrdscr_idx = cpu_mfr_list;
      processor_chip++;
      cmfr_dscr=cpu_mfr_list;
    }
}

#ifdef WINDOWS_VERSION

void iusleep(DWORD x)
{
  static double freq =-1.0;
  static int tm_bypass=0;
  LARGE_INTEGER tk1, tk2;
  
  if(tm_bypass) {mssleep(x%1000); return;}
  if (freq <= 0)
    {
    	if (QueryPerformanceFrequency(&tk1) ==0 ) 
        {
        	printf("Host doesn't support high presious timing!\n");
    	    tm_bypass = 1;
    	    return;
        }
       freq = dbl(tk1)/1e6;  //Mhz
//       printf("Host Timer working freq = %.3f MHz.\n", freq);
     }

  QueryPerformanceCounter(&tk1);
  do
   {
       QueryPerformanceCounter(&tk2);
   }
    while ( ((dbl(tk2)-dbl(tk1))/freq) < x);
    
//    printf(" tk1 = %f, \n", dbl(tk1));
//    printf(" tk2 = %f, \n", dbl(tk2));
//    printf(" delay time = %f us, \n",(dbl(tk2)-dbl(tk1))/freq);

}

#endif

int main2(int argc, char** argv)
{
	//
	int iii =0;
	for (  iii=1; iii<3; iii++)
	{
		int BO_INV1 = iii;
		int jjjj = 0;
		for ( jjjj = 1; jjjj<3;jjjj++)
		{
			 int BI_INV1 = jjjj;
			 printf ("xxxxxxxxxxxxxxxBO_INV1 = %X, BI_INV = %X\n", BO_INV1, BI_INV1);
			 //main1(argc, argv);
		}

	}
	return 0;
}

int main(int argc, char** argv)
{
  char choice[128];
  int j;
  int run_option =0;

  printf("\n");
  printf("        ==============================================\n");
  printf("               zJTAG EJTAG Debrick Utility v1.8 RC3\n");
  printf("        ==============================================\n\n");

 	chip_filllist();

  if (argc < 2)
    {
      show_usage();
      exit(1);
    }
     	    
  if (argc==2 && strcasecmp(argv[1],"/showflashlist")==0)
    {
      show_flashlist();
      exit(1);
    }



  strcpy(choice,argv[1]);

  // default to tumpa cable:
   cable_list[cable_id].id = cable_id;
   if(cable_list[cable_id].is_bulk)
	{
           		 USBID = cable_list[cable_id].devid;
           		 cable_intf = 1;
           		 cable_prop = &cbl_prop;
    }else{
          	 	safemode = 0;
           	 	cable_type = cable_list[cable_id].subid;
				cable_intf = 0;
    }


  if (strncasecmp(choice,"-backup:",8 )==0)
    {
      run_option = 1;
      
      if (strcasecmp(choice+8,"tfe")==0) strcpy(AREA_NAME, "TFE");
      	else if (strcasecmp(choice+8,"cfe")==0) strcpy(AREA_NAME, "CFE");
      		else if (strcasecmp(choice+8,"nvram")==0) strcpy(AREA_NAME, "NVRAM");
      			else if (strcasecmp(choice+8,"kernel")==0) strcpy(AREA_NAME, "KERNEL");
      		else if (strcasecmp(choice+8,"wholeflash")==0) strcpy(AREA_NAME, "WHOLEFLASH");
        	else if (strcasecmp(choice+8,"bsp")==0) strcpy(AREA_NAME, "BSP");
        	  else if (strcasecmp(choice+8,"custom")==0) 
        		 { strcpy(AREA_NAME, "CUSTOM");custom_options++;}
        	else run_option = 0;
    }
  else if (strncasecmp(choice,"-erase:",7 )==0)
    {
      run_option = 2;
      
      if (strcasecmp(choice+7,"tfe")==0) strcpy(AREA_NAME, "TFE");
      	else if (strcasecmp(choice+7,"cfe")==0) strcpy(AREA_NAME, "CFE");
      		else if (strcasecmp(choice+7,"nvram")==0) strcpy(AREA_NAME, "NVRAM");
      			else if (strcasecmp(choice+7,"kernel")==0) strcpy(AREA_NAME, "KERNEL");
      		else if (strcasecmp(choice+7,"wholeflash")==0) strcpy(AREA_NAME, "WHOLEFLASH");
        	else if (strcasecmp(choice+7,"bsp")==0) strcpy(AREA_NAME, "BSP");
        	  else if (strcasecmp(choice+7,"custom")==0) 
        		 { strcpy(AREA_NAME, "CUSTOM");custom_options++;}
        	else run_option = 0; 

    }
  else if (strncasecmp(choice,"-flash:",7 )==0)
    {
      run_option = 3;
      
      if (strcasecmp(choice+7,"tfe")==0) strcpy(AREA_NAME, "TFE");
      	else if (strcasecmp(choice+7,"cfe")==0) strcpy(AREA_NAME, "CFE");
      		else if (strcasecmp(choice+7,"nvram")==0) strcpy(AREA_NAME, "NVRAM");
      			else if (strcasecmp(choice+7,"kernel")==0) strcpy(AREA_NAME, "KERNEL");
      		else if (strcasecmp(choice+7,"wholeflash")==0) strcpy(AREA_NAME, "WHOLEFLASH");
        	else if (strcasecmp(choice+7,"bsp")==0) strcpy(AREA_NAME, "BSP");
        	  else if (strcasecmp(choice+7,"custom")==0) 
        		 { strcpy(AREA_NAME, "CUSTOM");custom_options++;}
        	else run_option = 0; 
        		
    }
   else if (strcasecmp(choice,"-probeonly")==0)
    {
      run_option = 4;
      is_probe = 1;
    } 
    

  if (run_option == 0)
    {
      show_usage();
      printf("\n*** ERROR - Invalid [option] specified ***\n\n");
      exit(1);
    }

  if (argc > 2)
    {
      j = 2;
      while (j < argc)
        {
          strcpy(choice,argv[j]);
       /* **     CPU    ** */
          if (strcasecmp(choice,"/noreset")==0)              issue_reset = 0;
          else if (strcasecmp(choice,"/noemw")==0)           issue_enable_mw = 0;
          else if (strcasecmp(choice,"/nocwd")==0)           issue_watchdog = 0;
          else if (strcasecmp(choice,"/nobreak")==0)         issue_break = 0;
        else if (strcasecmp(choice,"/LE")==0)               {force_endian = 1; endian = __LE;}
        else if (strcasecmp(choice,"/BE")==0)               {force_endian = 1; endian = __BE;}
          else if (strcasecmp(choice,"/initcpu")==0)         issue_initcpu=1;
          else if (strcasecmp(choice,"/nompi")==0)           no_mpidetect=1;	
          else if (strcasecmp(choice,"/skipdetect")==0)      skipdetect = 1;
          else if (strncasecmp(choice,"/instrlen:",10)==0)   instrlen = strtoul(((char *)choice + 10),NULL,10);
 
       /* **    Ejtag Access   ** */
          else if (strcasecmp(choice,"/noerase")==0)         issue_erase = 0;
        else if (strcasecmp(choice,"/dma")==0)              {force_dma = 1; force_nodma = 0; }
          else if (strcasecmp(choice,"/nodma")==0)          {force_nodma = 1; force_dma = 0; }
        	else if (strcasecmp(choice,"/ejslow")==0)          ejtag_speed = 1;
        	else if ((strcasecmp(choice,"/wx8")==0) || (strcasecmp(choice,"/byte_mode")==0))            prg_x8 = 1;
        	else if (issue_break && strcasecmp(choice,"/waitbrk")==0)    wait_break = 1;
          else if (strcasecmp(choice,"/safemode")==0)        safemode = 1;
          else if (strcasecmp(choice,"/srst")==0)           issue_srst = 1;
          	
	        		
       /* **    Flash   ** */
          else if (strcasecmp(choice,"/nocfi")==0) 	         issue_cfi_qry =0;
       	  else if (strcasecmp(choice,"/forcenoflip")==0) 	   force_noflip =1;
       	  else if (strcasecmp(choice,"/forceflip")==0) 	   { force_noflip =0; force_flip=1; }
          else if (strcasecmp(choice,"/forcealign")==0) 	   force_align =1;
          else if (strncasecmp(choice,"/fc:",4)==0)        {  selected_fc = strtoul(((char *)choice + 4),NULL,10);
          	                                                issue_cfi_qry =0; }
          else if (strcasecmp(choice,"/bypass")==0)          bypass = 1;
          else if (strcasecmp(choice,"/erasechip")==0)     { if (run_option ==4) issue_chip_erase = 1; } 
          else if (strcasecmp(choice,"/clearppb")==0)      { if (run_option ==4) issue_clear_ppb = 1;  }	
 				  else if (strcasecmp(choice,"/showppb")==0)       { if (run_option ==4) issue_show_ppb = 1;   } 	
 						
       /* **    File   ** */
          else if (strcasecmp(choice,"/notimestamp")==0)     issue_timestamp = 0;	
          else if (strncasecmp(choice,"/window:",8)==0)
            {
              selected_window = strtoul(((char *)choice + 8),NULL,16);
              custom_options++;
              if (run_option == 4) {strcpy(AREA_NAME, "CUSTOM"); 
                                  selected_window &= 0xFFFFF000;} 
            }
          else if (strncasecmp(choice,"/start:",7)==0)
            {
              selected_start  = strtoul(((char *)choice + 7),NULL,16);
              custom_options++;
            }
          else if (strncasecmp(choice,"/length:",8)==0)
            {
              selected_length = strtoul(((char *)choice + 8),NULL,16);
              custom_options++;
            }

        /* **    test only   ** */    
//        else if (strncasecmp(choice,"/ad:",4)==0)        {  ad = strtoul(((char *)choice + 4),NULL,10) & 0x3;}
//        else if (strncasecmp(choice,"/sz:",4)==0)        {  sz = strtoul(((char *)choice + 4),NULL,10) & 0x3;}


            
        /* **    Others   ** */  
          else if (strcasecmp(choice,"/verbose")==0)         silent_mode = 0;
          else if (strcasecmp(choice,"/debug")==0) {        DEBUGMSG1 = 1; DEBUGMSG = 1; }
		  else if (strcasecmp(choice,"/cable:dlc5")==0)
		  {
				  cable_type = XILINX;
				  cable_intf = 0;
				  printf("cable=dlc5, cabletype=%d\n", cable_type);
		  }
          else if ((strcasecmp(choice,"/wiggler")==0) || (strcasecmp(choice,"/cable:wiggler")==0))
		  {
				  cable_type = WIGGLER;
				  cable_intf = 0;
				  printf("cable=wiggler, cabletype=%d\n", cable_type);
		  }
		  else if ((strcasecmp(choice,"/cablefta")==0) || (strcasecmp(choice,"/diygadgetfta")==0))
		  {
				  cable_type = XILINX;
				  cable_intf = 0;
				  printf("cable=fta, cabletype=%d\n", cable_type);
		  }
		  else if ((strcasecmp(choice,"/cableblackcat")==0) || (strcasecmp(choice,"/diygadgetblackcat")==0))
		  {
				  cable_type = BLACKCAT;
				  cable_intf = 0;
				  printf("cable=blackcat, cabletype=%d\n", cable_type);
		  }
          else if (strncasecmp(choice,"/port:",6)==0)
            {
              lpt_port = strtoul(((char *)choice + 6),NULL,16);
            }

#ifdef WINDOWS_VERSION   // ---- Compiler Specific Code ----            
          else if (strcasecmp(choice,"/io2")==0)            is_os64 = 1;  
#endif

          else if (strncasecmp(choice,"/cable:",7)==0)
           {
           
           	 int kk = 0;
             cable_id = strtoul(((char *)choice + 7),NULL,10);
             while(cable_list[kk].id != -1) {kk++;}
             //printf("\nkk,cable_id = %d,%d\n",kk,cable_id);
             if( kk <= cable_id)
            	{
            		printf("\n*** ERROR - /cable: selction is wrong ***\n");
            		exit(1);
            	}	 
           
             cable_list[cable_id].id = cable_id;
             if(cable_list[cable_id].is_bulk)
           	 {
           		 USBID = cable_list[cable_id].devid;
           		 cable_intf = 1;
           		 cable_prop = &cbl_prop;
				 printf("cableid=%d, cabletype=%d\n", cable_id, cable_type);
           	 }else{
           	 	safemode = 0;
				cable_intf = 0;
           	 	cable_type = cable_list[cable_id].subid;
				printf("cableid=%d, cabletype=%d\n", cable_id, cable_type);
           	 }
           }
		  // added by tumpa
		   else if (strncasecmp(choice,"/delay:",7)==0)
		   {
				delay_in_ms = strtoul(((char *)choice + 7),NULL,10);
				printf("\n*** INFO - sleep <%d> (ms) per internal command ***\n",delay_in_ms);
		   }
           else if(cable_intf)
           	{
           		if( getusb_setting(choice))
                   {
                     printf("\n*** WARNING - Invalid option <%s> specified ***\n",choice);
                     exit(1);
                    }
            }
           else
          	 printf("\n*** WARNING - Invalid option <%s> specified ***\n",choice);
         
          j++;
        }
       printf("\n");   
          	
    }

  if (strcasecmp(AREA_NAME,"CUSTOM")==0)
    {
      if ((custom_options != 0) && (custom_options != 4) && (run_option != 4 ))
        {
          show_usage();
          printf("\n*** ERROR - 'CUSTOM' also requires '/window' '/start' and '/length' options ***\n\n");
          exit(1);
        }
    }

  if(force_nodma) safemode = 1;
  if (!cable_intf)
  {
    lpt_openport();
  
  }
  else
   { 
     if(!(cable_list[cable_id].init)((void*)cable_prop))
     	{printf("\n***ERROR*** USB TAP device has not been initialized correctly!\n"); 
     	exit(1);}
   	 printf("\nUSB TAP device has been initialized. Please confirm VREF signal connected!\n"); 	
#ifdef WINDOWS_VERSION
   	 printf("Press any key to continue... ONCE target board is powered on!\n\n"); 	
   	 getch();
#else
   	 printf("Press any <Enter> key to continue... ONCE target board is powered on!\n\n"); 	
     getchar();
#endif
    }

  // ----------------------------------
  // Reset TAP State Machine to RTI
  // ----------------------------------
  test_reset();
  
  if(issue_srst && !cable_intf )
  	{
  		lpt_srst();
  		test_reset();
    }
  // ----------------------------------
  // Detect CPU
  // ----------------------------------

  chip_detect();

  // ----------------------------------
  // Find Implemented EJTAG Features
  // ----------------------------------
  check_ejtag_features();

  // ----------------------------------
  // Reset State Machine For Good Measure
  // ----------------------------------
  test_reset();

  // ----------------------------------
  // Reset Processor and Peripherals
  // ----------------------------------
  printf("Issuing Processor / Peripheral Reset ... ");
  if (issue_reset)
    {
      set_instr(INSTR_EJTAGBOOT);   //take a debug exception after reset
      mssleep(1);
 
      set_instr(INSTR_CONTROL);
      ctrl_reg = ReadWriteData(PRRST | PERRST);
      ctrl_reg = ReadWriteData(0x0);
      ctrl_reg = ReadWriteData(0x0);
      printf("Done\n");
    }
  else printf("Skipped\n");


  // ----------------------------------
  // Enable Memory Writes
  // ----------------------------------
  // Always skip for EJTAG versions 2.5 and 2.6 since they do not support DMA transactions.
  // Memory Protection bit only applies to EJTAG 2.0 based chips.
  if (ejtag_version != 0)  issue_enable_mw = 0;
  printf("Enabling Memory Writes ... ");
  if (issue_enable_mw)
    {
      // Clear Memory Protection Bit in DCR
      ejtag_dma_write_x(0xff300000, (ejtag_dma_read_x(0xff300000,MIPS_WORD) & ~(1<<2)),MIPS_WORD );
      printf("Done\n");
    }
  else printf("Skipped\n");

  // ----------------------------------
  // Put into EJTAG Debug Mode
  // ----------------------------------
  printf("Halting Processor ... ");
  if (issue_break)
    {
 dtry:
      set_instr(INSTR_CONTROL);
      ctrl_reg = ReadWriteData(PRACC | PROBEN | SETDEV | JTAGBRK );
      mssleep(10);
      if (ReadWriteData(PRACC | PROBEN | SETDEV) & BRKST)
        { printf("<Processor Entered Debug Mode!> ... ");
          if (!USE_DMA) g_init_dreg();
        }
      else
        { printf("<Processor did NOT enter Debug Mode!> ... ");
        	if(wait_break){printf("\n"); goto dtry;}
        }		
      printf("Done\n");
    }
  else printf("Skipped\n");

        	
  // ----------------------------------
  // Clear Watchdog
  // ----------------------------------
  printf("Clearing Watchdog ... ");
  if (issue_watchdog)
    {
      ejtag_write_x(0xb8000080,0,MIPS_WORD);
      printf("Done\n");
    }
  else printf("Skipped\n");

  // ----------------------------------
  // Initialize Processor Configuration
  // ----------------------------------
  printf("Loading CPU Configuration Code ... ");
  if (issue_initcpu) 
    {
      g_init_cpu();
      printf("Done\n");
    }
  else printf("Skipped\n");

  // ----------------------------------
  // Detect Flash Window Base
  // ----------------------------------

  
  if (mpi_base && !no_mpidetect)
    {
    	DWORD mpi_reg = 0;
      printf("Detecting Flash Base Address...\n");
      mpi_reg = ejtag_dma_read_x(mpi_base,MIPS_WORD);
      printf("Read MPI register value : %08X\n" , mpi_reg  );
      FLASH_MEMORY_START = mpi_reg & 0x1FFFFF00;
      printf("MPI register show Flash Access Base Addr : %08X\n" , FLASH_MEMORY_START);
    }
  if (no_mpidetect) {mpi_base = 0; }

  // ----------------------------------
  // Flash Chip Detection
  // ----------------------------------
  if (selected_fc > 1)
    sflash_config();
  else
    sflash_probe();
  test_reset();
  // ----------------------------------
  // Execute Requested Operation
  // ----------------------------------
  if ((flash_size > 0) && (AREA_LENGTH > 0))
    {
      if (run_option == 1 )  run_backup(AREA_NAME, AREA_START, AREA_LENGTH);
      if (run_option == 2 )  run_erase(AREA_NAME, AREA_START, AREA_LENGTH);
      if (run_option == 3 )  run_flash(AREA_NAME, AREA_START, AREA_LENGTH);
    }
     
   if ((flash_size > 0) && (run_option == 4 )) 
   	{
     	  // In probe whole chip erase can be done at customed address.
      	// Check the Spansion advanced sector protect
    	 if (issue_chip_erase) sflash_erase_chip();
    	 if (issue_show_ppb || issue_clear_ppb) sp_check_ppb(); 
   	 	 
    }
    

  printf("\n\n *** REQUESTED OPERATION IS COMPLETE ***\n\n");

  chip_shutdown();

  return 0;
}

static int getusb_setting(char* choice)
{

          		if( strncasecmp(choice,"/L1:",4)==0 )
              {
                
                 if(strncasecmp(choice,"/L1:0x",6)==0)LL1 = strtoul(((char *)choice + 6),NULL,16) & 0xFFFF;
                 else if(strncasecmp(choice,"/L1:0X",6)==0)LL1 = strtoul(((char *)choice + 6),NULL,16) & 0xFFFF;
                 else if(strncasecmp(choice,"/L1:",4)==0)LL1 = strtoul(((char *)choice + 4),NULL,10) & 0xFFFF;
                 	
              }
              else if ( (strncasecmp(choice,"/L2:",4)==0) )
              {
              LL2 =(BYTE)( strtoul(((char *)choice + 4),NULL,10) & 0xFF);
              }
              else if ( (strncasecmp(choice,"/L3:",4)==0) )
              {
              LL3 = strtoul(((char *)choice + 4),NULL,10);
              if (LL3 >48) LL3 = 48;
              set_LL3 = 1;
              }
              else if ((strncasecmp(choice,"/L4:",4)==0) )
              {
              LL4 = strtoul(((char *)choice + 4),NULL,10);
              if (LL4 > 128) LL4 = 128;
              flsPrgTimeout = LL4;
              }
              else if ((strncasecmp(choice,"/L5:",4)==0) )
              {
              LL5 = strtoul(((char *)choice + 4),NULL,10);
              LL5 &= 0xFFFF;
              
              }
              else if ( (strncasecmp(choice,"/L6:",4)==0) )
              {
              LL6 = strtoul(((char *)choice + 4),NULL,10);
              if (LL6 <16) LL6 = 16;
              if (LL6 >400) LL6 = 400;
              }
              else if ((strncasecmp(choice,"/L7:",4)==0) )
              {
              LL7 = strtoul(((char *)choice + 4),NULL,10);
              if (LL7 <1) LL7 = 1;
              }
              else if ((strncasecmp(choice,"/L8:",4)==0) )
              {
              LL8 = strtoul(((char *)choice + 4),NULL,10);
              if (LL8 <1) LL8 = 1;
              }
              else if ((strncasecmp(choice,"/L9:",4)==0) )
              {
              LL9 = strtoul(((char *)choice + 4),NULL,10);
              LL9 &= 0xF;
              }
              else if ((strncasecmp(choice,"/ID:",4)==0) )
              {
              USBID = strtoul(((char *)choice + 4),NULL,16);
              }
              else if ((strncasecmp(choice,"/G1:",4)==0) )
              {
              GG1 =(BYTE)( strtoul(((char *)choice + 4),NULL,16) & 0xFF);
              }
              else if ((strncasecmp(choice,"/G2:",4)==0) )
              {
              GG2 =(BYTE)( strtoul(((char *)choice + 4),NULL,16) & 0xFF);
              }
              else if ( (strcasecmp(choice,"/showgpio")==0) )
              {
              showgpio = 1;
              }
              else return 1;   //invalid opt
          return 0;
}

// **************************************************************************
// End of File
// **************************************************************************
