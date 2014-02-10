/*
 * created by hugebird @ chinadsl.net 11/7/2009 rev1.8c
 *
 * Common Flash Interface suport data
 * Refer to Kernel mtd/cfi.h
 *
 * This code is covered by the GPL.
 *  This program is copyright (C) 2009 hugebird
*/




#define CFI_QRY_OK  1
#define CFI_QRY_ERR 0


/* define manufacture ID */
#define MFR_AMD	      0x0001      //AMD Spansion
#define MFR_FUJITSU   0x0004
#define MFR_NEC       0x0010
#define MFR_EON_LEG   0x001C
#define MFR_ATMEL     0x001F
#define MFR_ST        0x0020      //ST Nymonyx
#define MFR_EON       0x007F
#define MFR_INTEL     0x0089
#define MFR_TOSHIBA   0x0098
#define MFR_PMC       0x009D
#define MFR_HYUNDAI   0x00AD
#define MFR_SHARP     0x00B0
#define MFR_SST       0x00BF
#define MFR_MACRONIX  0x00C2      //MXIC
#define MFR_WINBOND   0x00DA
#define MFR_SAMSUNG   0x00EC
#define MFR_UNKNOW    0x00AA     //unregisted manufacturer, but have CFI structure


typedef struct _mfr_type {
	unsigned short  id;
	char *  name;
	} mfr_type;
	
static mfr_type cfi_mfr_list[]={
 {MFR_UNKNOW    ,"Unknow"      },
 {MFR_AMD	      ,"AMD/Spansion"},
 {MFR_FUJITSU   ,"Fujitshu"    },
 {MFR_NEC       ,"NEC"         },
 {MFR_EON_LEG   ,"EON(alt)"    },
 {MFR_ATMEL     ,"Atmel"       },
 {MFR_ST        ,"ST/Numonyx"  },
 {MFR_EON       ,"EON"         },
 {MFR_INTEL     ,"Intel"       },
 {MFR_TOSHIBA   ,"Toshiba"     },
 {MFR_PMC       ,"PMC"         },
 {MFR_HYUNDAI   ,"Hyundai"     },
 {MFR_SHARP     ,"Sharp"       },
 {MFR_SST       ,"SST"         },
 {MFR_MACRONIX  ,"Macronix"    },
 {MFR_WINBOND   ,"Winbond"     },
 {MFR_SAMSUNG   ,"Samsung"     },
 {0,0}};

/* define command set ID */
#define P_ID_NONE               0x0000
#define P_ID_INTEL_EXT          0x0001
#define P_ID_AMD_STD            0x0002
#define P_ID_INTEL_STD          0x0003
#define P_ID_AMD_EXT            0x0004
#define P_ID_WINBOND            0x0006
#define P_ID_ST_ADV             0x0020
#define P_ID_MITSUBISHI_STD     0x0100
#define P_ID_MITSUBISHI_EXT     0x0101
#define P_ID_SST_PAGE           0x0102
#define P_ID_INTEL_PERFORMANCE  0x0200
#define P_ID_INTEL_DATA         0x0210
#define P_ID_RESERVED           0xffff
#define P_ID_SST                0x0701

/* AMD flsh boot location */
#define BOT_BOOT 2
#define TOP_BOOT 3
#define UBOT_BOOT 4
#define UTOP_BOOT 5
 


#define CFI_QRY_BUSWIDTH sizeof(unsigned short)




/* Basic Query Structure */
typedef struct _cfi_qry_type {
	unsigned short  qry[3];
	unsigned short  P_ID[2];
	unsigned short  P_ADR[2];
	unsigned short  A_ID[2];
	unsigned short  A_ADR[2];
	unsigned short  VccMin;
	unsigned short  VccMax;
	unsigned short  VppMin;
	unsigned short  VppMax;
	unsigned short  WordWriteTimeoutTyp;
	unsigned short  BufWriteTimeoutTyp;
	unsigned short  BlockEraseTimeoutTyp;
	unsigned short  ChipEraseTimeoutTyp;
	unsigned short  WordWriteTimeoutMax;
	unsigned short  BufWriteTimeoutMax;
	unsigned short  BlockEraseTimeoutMax;
	unsigned short  ChipEraseTimeoutMax;
	unsigned short  DevSize;
	unsigned short  InterfaceDesc[2];
	unsigned short  MaxBufWriteSize[2];
	unsigned short  NumEraseRegions;
	unsigned short  EraseRegionInfo[4][4]; 
} cfi_qry_type ;

/* Extended Query Structure for both PRI and ALT */

typedef struct _cfi_pri_type {
	unsigned short  pri[3];
	unsigned short  MajorVersion;
	unsigned short  MinorVersion;
	unsigned short  Features;        //for Intel 
} cfi_pri_type;

#define FULL_CHIP_ERASE  0x1           // intel chip feature

/* Vendor-Specific PRI for AMD/Fujitsu Extended Command Set (0x0002) */

typedef struct _cfi_pri_amdstd_type {
	unsigned short  pri[3];
	unsigned short  MajorVersion;
	unsigned short  MinorVersion;
	unsigned short  SiliconRevision; /* bits 1-0: Address Sensitive Unlock */
	unsigned short  EraseSuspend;
	unsigned short  BlkProt;
	unsigned short  TmpBlkUnprotect;
	unsigned short  BlkProtUnprot;
	unsigned short  SimultaneousOps;
	unsigned short  BurstMode;
	unsigned short  PageMode;
	unsigned short  VppMin;
	unsigned short  VppMax;
	unsigned short  TopBottom;
} cfi_pri_amdstd_type;

/* Vendor-Specific PRI for Atmel chips (command set 0x0002) */

typedef struct _cfi_pri_atmel_type {
	unsigned short pri[3];
	unsigned short MajorVersion;
	unsigned short MinorVersion;
	unsigned short Features;
	unsigned short BottomBoot;
	unsigned short BurstMode;
	unsigned short PageMode;
} cfi_pri_atmel_type;


/* define functions prototype */
static void cfi_read_array(unsigned short *ptr, int ofs, int length);
static int cfi_qry_request();

/*  End of line */
