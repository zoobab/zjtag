/*
 * created by hugebird @ chinadsl.net 1/5/2010 rev1.9a
 *
 *
 * FTDI FT2232D JTAG application in MPSSE mode for Brjtag.
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

#ifdef FTDIXXX
 
typedef DWORD FTC_HANDLE;
typedef DWORD FTC_STATUS;


typedef enum _JtagStates {
	                JTAG_TLR  = 0, 
	                JTAG_RTI  = 1, 
	                JTAG_PDR  = 2, 
	                JTAG_PIR  = 3, 
	                JTAG_SDR  = 4, 
	                JTAG_SIR  = 5, 
	                Undefined = 6} JtagStates;

#define MAX_JTAG_STATES 6
#define NO_TAILBIT      0


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

//  TMS shift max limit to 7 tcks

 
// MPSSE Data Shifting Commands bit setting
#define FTC_WR_CLKN     0x01    //bit0, Write -ve
#define FTC_BIT         0x02    //bit1, 1 bit mode, 0 byte mode
#define FTC_RD_CLKN     0x04    //bit2, Read -ve
#define FTC_LSB         0x08    //bit3, 1 LSB Shift 1st, 0 MSB Shift 1st
#define FTC_WR          0x10    //bit4, Write TDI/DO
#define FTC_RD          0x20    //bit5, Read  TDO/DI
#define FTC_TMS         0x40    //bit6, Write TDI/DO tail bit, Write TMS xN clk


// EJTAG require
//  LSB shift first
//  TCK    initial state low
//  TDI/DO clock out on -ve edge  
//  TDO/DI clock in  on +ve edge 
//  TMS    clock out on -ve edge, TMS initial state high

// MPSSE Data Shift Command
#define DSC_BYTES_OUTN_LSB     0x19
#define DSC_BITS_OUTN_LSB      0x1B

#define DSC_BYTES_IN_LSB       0x28
#define DSC_BITS_IN_LSB        0x2A

#define DSC_BYTES_OUTN_IN_LSB  0x39
#define DSC_BITS_OUTN_IN_LSB   0x3B

#define DSC_TMSN_LSB           0x4B
#define DSC_TMSN_IN_LSB        0x6B

#define DSC_SET_GPIOL_BITS     0x80
#define DSC_SET_GPIOH_BITS     0x82
#define DSC_GET_GPIOL_BITS     0x81
#define DSC_GET_GPIOH_BITS     0x83

#define DSC_SET_CLK_FREQ       0x86
#define DSC_SEND_FDBK_IMMD     0x87

#define DSC_LOOPBACK_ON        0x84
#define DSC_LOOPBACK_OFF       0x85

// disable divide by 5
#define DSC_DISABLE_DIVIDE_5_CMD   0x8A

// bit and mask definitions for JTAG signals
#define FT_TCK         0
#define FT_TDI         1
#define FT_TDO         2
#define FT_TMS         3
#define FT_MASK_TDO     (1 << FT_TDO)
#define FT_MASK_TDI     (1 << FT_TDI)
#define FT_MASK_TCK     (1 << FT_TCK)
#define FT_MASK_TMS     (1 << FT_TMS)


#define FT_JTAG_nOE        4
// OpenMoko GPIO definition
#define FT_Moko_nTRST_nOE  0
#define FT_Moko_nTRST_OUT  1
#define FT_Moko_nSRST_nOE  2
#define FT_Moko_nSRST_OUT  3

#define FT_Moko_EMU_IN     4
#define FT_Moko_SRST_IN    7

#define FT_DIR_Moko_nTRST_nOE (1 << FT_Moko_nTRST_nOE)
#define FT_DIR_Moko_nTRST_OUT (1 << FT_Moko_nTRST_OUT)
#define FT_DIR_Moko_nSRST_nOE (1 << FT_Moko_nSRST_nOE)
#define FT_DIR_Moko_nSRST_OUT (1 << FT_Moko_nSRST_OUT)


// FT2322 USB Output/Input Buffer byte length
#define MAX_NUM_BYTES_USB_WRITE      65536
#define MAX_NUM_BYTES_USB_WRITE_READ 65536
#define MAX_NUM_BYTES_USB_READ       65536

// TX/RX data buffer
#define MAX_DATA_BLK_SIZE   16384  //Tx Data blk
#define MAX_DSC_BLOCK_SIZE  8000  //Max DSC block reg count, 9B/IR or 16B/DWORD, 8K enough for 64KB buffer
#define MAX_DMA_BLOCK_SIZE  1100   //Max DMA xfer block size, x10 Data blk per DMA blk
#define DMA_BLOCK_SIZE      1100   //L6 defult DMA xfer block size, real used DMA blk size
#define DMA_POLLING            2   //L3 defult polling retry time on DMA xfering
#define FLASH_POLLING         16   //L4 defult polling retry time on flash writing
#define FT_LATENCY             2   //L2 default latency timer in ms


// port scan data type
#define TAP_WO     0x10
#define TAP_RO     0x20
#define TAP_RW     0x30

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

//---------------------pub-----------------------------------------
int ftinit(void *);
void ftclose(void);
void fttest_reset(void);
DWORD ftset_instr(DWORD);
DWORD ftdet_instr(void);
DWORD ftReadWriteData(DWORD);
DWORD ftReadData(void);
void ftWriteData(DWORD);
DWORD ft_dma_blkfetch(DWORD, DWORD, int, int, int*);
void ft_sflash_write_word(DWORD, DWORD,int);
int ft_sflash_blkread(DWORD, DWORD*, int);
int ft_sflash_blkwrite(DWORD, DWORD*, int,int);
void ft_dma_rdtraining(DWORD,DWORD);
void ft_dma_wrtraining(DWORD,DWORD);

//------------------private-----------------------------------------
static DWORD bitmask32(int);
static void FTDI_AddByteToOutputBuffer(DWORD, int);
static void FTDI_AddNBytesToOutputBuffer(BYTE* , DWORD , int);
static DWORD FTDI_AddDSCBlkToOutBuffer(int, int, BYTE*, DWORD, DWORD);
static void FTDI_CalcBlkBytesToRead(DWORD, DWORD* , DWORD* );
static DWORD FTDI_MoveJTAGStateTo(JtagStates, DWORD, int);
static FT_STATUS FTDI_WriteReadWithDevice( int , DWORD , DWORD* );
static int FTDI_scan_blk_post(int, int,BYTE*, DWORD, int, DWORD,int);
static int FTDI_scan_flush(int);
static void FTDI_scan_blk (int );
static int FTDI_scan_xfer (void);
static void FTDI_CleanFlushIndex(void);
static void FTDI_AddBlkToTXDataBuffer(DWORD, int,int,int);
static DWORD getDWORD(BYTE *);
static void FTDI_AddBlkDelay(int);	
static void FTDI_WaitInRTI(int);
static void FTDI_WaitInTLR(int);
static void FTDI_ClockOutTMS(DWORD, DWORD, int);
static void ft_sflash_write_x16(DWORD, DWORD, int*);
static void ft_sflash_write_x8(DWORD, DWORD, int*);
static void fill_cable_prop(void *p);
static void ft_purgebuffer(void);
static void ft_getconfig(void);

#endif

