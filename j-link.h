/*
 * created by hugebird @ chinadsl.net 1/5/2010 rev1.9a
 *
 *
 * J-Link JTAG application in bit-wise mode for Brjtag.
 * Copyright (C) 2010 Hugebird
 *
 * This code is covered by the GPL v2.
 *
 *
 *
 */

 
#if ( defined(_MSC_VER)  )
#define WINDOWS_VERSION
#endif        

#ifdef WINDOWS_VERSION
  #include "windows.h"

#else 

  #ifndef MY_TYPE
    #define MY_TYPE
    typedef uint32_t     DWORD;
    typedef uint16_t     WORD;
    typedef unsigned char     BYTE;
    typedef int               bool;
  #endif

#endif 

#ifdef BJLINK

typedef enum _JtagStates {
	                JTAG_TLR  = 0, 
	                JTAG_RTI  = 1, 
	                JTAG_PDR  = 2, 
	                JTAG_PIR  = 3, 
	                JTAG_SDR  = 4, 
	                JTAG_SIR  = 5, 
	                Undefined = 6} JtagStates;

#define MAX_JTAG_STATES 6

static const BYTE JTAGST_FromTo[MAX_JTAG_STATES][MAX_JTAG_STATES] = {
                            /*    0TLR    1RTI    2PDR    3PIR    4SDR    5SIR   */
       /* from TLR 0-> */       {'\x01', '\x00', '\x0A', '\x16', '\x02', '\x06'}, 
       /* from RTI 1-> */       {'\x07', '\x00', '\x05', '\x0B', '\x01', '\x03'},
       /* from PDR 2-> */       {'\x1F', '\x03', '\x00', '\x2F', '\x01', '\x0F'}, 
       /* from PIR 3-> */       {'\x1F', '\x03', '\x17', '\x00', '\x07', '\x01'}, 
       /* from SDR 4-> */       {'\x1F', '\x03', '\x01', '\x2F', '\x00', '\x0F'}, 
       /* from SIR 5-> */       {'\x1F', '\x03', '\x17', '\x01', '\x07', '\x00'} }; 

static const BYTE TMSCLK_JTAGST_FromTo[MAX_JTAG_STATES][MAX_JTAG_STATES] = {            
                            /*   0TLR    1RTI    2PDR    3PIR    4SDR    5SIR   */ 
       /* from TLR 0-> */        {1,      1,      5,      6,      4,      5},  
       /* from RTI 1-> */        {3,      1,      4,      5,      3,      4},  
       /* from PDR 2-> */        {5,      3,      1,      7,      2,      6},  
       /* from PIR 3-> */        {5,      3,      6,      1,      5,      2},  
       /* from SDR 4-> */        {5,      3,      2,      7,      0,      6},  
       /* from SIR 5-> */        {5,      3,      6,      2,      5,      0}   };

#define JSTPATH_EXR_TO_RTI  0x01
#define JSTPLEN_EXR_TO_RTI  4

// j-link Protocal Command
// Get system information functions
#define EMU_CMD_VERSION		      0x01
#define EMU_CMD_GET_SPEED		    0xC0
#define EMU_CMD_GET_CAPS		    0xE8
#define EMU_CMD_GET_CAPS_EX	    0xED
#define EMU_CMD_GET_SPEED		    0xC0
#define EMU_CMD_GET_HW_VERSION  0xF0

// Get state information functions
#define EMU_CMD_GET_STATE		    0x07
#define EMU_CMD_GET_HW_INFO     0xC1
#define EMU_CMD_GET_COUNTERS    0xC2

// JTAG & Hardware functions
#define EMU_CMD_RESET_TRST      0x02
#define EMU_CMD_SET_SPEED		    0x05
#define EMU_CMD_HW_CLOCK		    0xC8  //Generates one clock and retrieves data from TDI
#define EMU_CMD_HW_TMS0			    0xC9  //Clear TMS
#define EMU_CMD_HW_TMS1			    0xCA  //Sets TMS
#define EMU_CMD_HW_DATA0		    0xCB  //Clears TDI signal
#define EMU_CMD_HW_DATA1	    	0xCC  //Sets TDI signal
#define EMU_CMD_HW_JTAG 	    	0xCD  //It receives data for TDI and TMS and sends TDO data back
                                      //xfer No Bits, obsolete
#define EMU_CMD_HW_JTAG2	    	0xCE  //It receives data for TDI and TMS and sends TDO data back
                                      //xfer No Bits, ver >=5, obsolete
#define EMU_CMD_HW_JTAG3	    	0xCF  //It receives data for TDI and TMS and sends TDO data back
                                      //xfer No Bits. ver >=5
#define EMU_CMD_HW_JTAG_WRITE		      0xD5 //It receives data for TDI and TMS. No TDO data is sent back
                                           // via the EMU_CMD_HW_JTAG_GET_RESULT get xfer status
#define EMU_CMD_HW_JTAG_GET_RESULT		0xCF
#define EMU_CMD_HW_SELECT_IF          0xC7 //JTAG:0, SWD:1, RESET:0xFF
                                           //JTAG: ifreset->ifjtag->reset1->trst1->speed
                                           
// Target functions
#define EMU_CMD_HW_RESET0		    0xDC
#define EMU_CMD_HW_RESET1	    	0xDD
#define EMU_CMD_HW_TRST0		    0xDE
#define EMU_CMD_HW_TRST1		    0xDF

// Retrieves capabilities of the emulator, flag bit mask
#define EMU_CAP_GET_HW_VERSION		  (1 <<  1)
#define EMU_CAP_SPEED_INFO    		  (1 <<  9)
#define EMU_CAP_GET_HW_INFO	   		  (1 << 12)
#define EMU_CAP_SELECT_IF 	   		  (1 << 17)
#define EMU_CAP_GET_COUNTERS 	   	  (1 << 19)
#define EMU_CAP_REGISTER    	   	  (1 << 27)

// Speed *KHz
#define JL_SPEED_12M   12000
#define JL_SPEED_1M    1000

// usb endpoint
#define JL_EP1  	0x81     //for read,  v3,v4 also use for write
#define JL_EP2		0x02     //for write, ver >=5

// USB Output/Input Buffer byte length
#define MAX_NUM_BYTES_USB_BUFFER      4096
#define JL_MAX_BUFFER_SIZE  3000    //v8 has enough room to keep 3000bytes, eg total out buffer 3000*2+4 = 6004 bytes
                                    // speced in/out total buffer len is 2048bytes, total 4+1000*2 = 2004 bytes.
                                    // tms_delay sequence need limit to 10 bytes.

// TX/RX data buffer
#define MAX_DATA_BUFFER_SIZE 65536
#define MAX_DATA_BLOCK_SIZE  20000


#define MAX_DMA_BLOCK_SIZE   400   //Max L9 DMA xfer block size for 65536 bytes buffer.
#define DMA_BLOCK_SIZE       256   //L6 defult DMA xfer block size
#define DMA_POLLING            2   //L3 defult polling retry time on DMA xfering
#define FLASH_POLLING         16   //L4 defult polling retry time on flash writing


// port scan data type
#define TAP_RD        0x80   // need return data
#define TAP_IR         1
#define TAP_DATA       2
#define TAP_TMS_DELAY  3
#define TAP_TMS_MOVE   4

// DMA xfer type
#define XFER_TX		1		// blcok xfer, add cmd to queue,    internal flag
#define XFER_RX   2   // block xfer, get data from queue, internal flag

#define XFER_ADD  3   // add DMA xfer block to queue, for caller
#define XFER_QRY  4   // find next read data block,fetch out, for caller
#define XFER_NOOP 5   // no operate, return free dma xfer block count

#define DMA_WR   1    // dma write operate
#define DMA_RD   2    // dma read operate



//--------------private-------------------------------------------

static int JL_scan_oneclk (int tms, int tdi);
static int JL_scan_blk (int BlkIndex);
static int JL_scan_flush (void);
static int JL_scan_xfer (void);
static void JL_Wait(int n);
static void JL_ResetToRTI (void);
static void JL_AddBlkToTxDataBuffer(DWORD data, int type, int bitlen, int Is_clear);
static void JL_ADDJTAGStateMoveTo(JtagStates JtagST2);
static void JL_simplecmd ( BYTE cmd );
static void JL_setspeed ( WORD khz );
static void JL_seleJtag (void);
static int JL_getstate(void);
static DWORD JL_getversion(void);
static DWORD JL_getcaps(void);
static DWORD bitmask32(int len);
static DWORD getDWORD(BYTE * pAddr);
static void JL_sflash_write_x16(DWORD, DWORD, int*);
static void JL_sflash_write_x8(DWORD, DWORD, int*);
static void fill_cable_prop(void *p);
static void JL_purgebuffer(void);
static void JL_getconfig(void);

//--------------------pub-------------------------------------
int jlinit(void *);
void jlclose(void);
void jltest_reset(void);
DWORD jlset_instr(DWORD);
DWORD jldet_instr(void);
DWORD jlReadWriteData(DWORD);
DWORD jlReadData(void);
void jlWriteData(DWORD);
DWORD jl_dma_blkfetch(DWORD, DWORD, int, int, int*);
void jl_sflash_write_word(DWORD, DWORD, int);
int jl_sflash_blkread(DWORD, DWORD*, int);
int jl_sflash_blkwrite(DWORD, DWORD*, int, int);

void jl_dma_rdtraining(DWORD,DWORD);
void jl_dma_wrtraining(DWORD,DWORD);

#endif


