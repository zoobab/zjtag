/*
 * created by hugebird @ chinadsl.net 17/9/2010 Brjtag rev1.9m
 *
 *
 * Brjtag application for HID-BRJTAG v1.xx MCU firmware.
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
    typedef unsigned uint32_t     DWORD;
    typedef unsigned uint16_t     WORD;
    typedef unsigned char     BYTE;
    typedef int               bool;
  #endif

#endif 

#ifdef BUSBASP

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

// flsh cmd id
#define AMD16_I1    0
#define AMD16_I2    1
#define AMD16_I3    2
#define AMD8_I1     3
#define AMD8_I2     4
#define AMD8_I3     5
#define SST16_I1    6
#define SST16_I2    7
#define SST16_I3    8
#define INTL16_I1   9
#define INTL16_I2   10
#define INTL8_I1    11
#define INTL8_I2    12


//---------------------------------------------------------
//       _____             _______            _______
//      | DUT |___________|TAP/MCU|__________|PC/HOST|
//       -----    JTAG     -------     USB    -------
//        BCM               MCU FW            BRJTAG
//---------------------------------------------------------

/*     MCU Processing State Machine      */
#define ST_MCU_IDLE     0x01     //ideal, can accept any cmd
#define ST_MCU_XFER     0x02     //on USB data xfer, not accept any new cmd
#define ST_MCU_BUSY     0x03     //on JTAG process
#define ST_MCU_RDY      0x04     //DMA Read/write complete, data reply buffer ready

//   idle -> xfer -> busy -> ready -> xfer -> idle


/*      Host <-> MCU  COMMAND ID rq->bRequest   */
#define REQ_MCU_HWVER          0x11        //EPIN,get MCU firmware version
#define REQ_MCU_RESET          0x12        //EPIN,reset tap to idle
#define REQ_MCU_SETSPD         0x13        //EPIN,set jtag speed
#define REQ_MCU_GetSTT         0x14        //EPIN,get MCU current state
#define REQ_MCU_CMDSEQ         0x21        //EPOUT,send cmd sequence to mcu
#define REQ_MCU_BITSEQ         0x22        //EPOUT,send bit sequence to mcu
#define REQ_MCU_GetDAT         0x23        //EPIN,get back sequence response data

//cmd sequence format (total length <= 201 bytes)
//[cmd id 0][cmd data 0][cmd id 1][cmd data 1] ... 
//
// Command ID:
#define CMD_TAP_DELAY           0x01        //pause DUT <-> TAP 
//In:  [ 0:id ][1 : us]              out:none
#define CMD_TAP_CLKIO           0x02        //clock shift in/out
//In: [0: id][1:bit len][2,3,4,5]    out:[0,..., (len+7)>>3]
#define CMD_TAP_DETIR           0x03        //IR shift/scan a instruction and return
//In: [0:id][1:ir bit len][2,3,4,5 ir data]     out:[0,1,2,3]
#define CMD_TAP_SETIR            0x04        //IR shift/scan a instruction
//In: [0:id][1:ir bit len][2,3,4,5 ir data]     out:none
#define CMD_TAP_DR32            0x05        //DR shift/scan a 32b Data
//In: [0:id][1,2,3,4 ir data]        out:[0,1,2,3]
#define CMD_TAP_DMAREAD        0x06        //DMA Read a 32b data
//In: [0:1d][1:type][2,3,4,5:ADDR]  out:[0,1,2,3]
#define CMD_TAP_DMAWRITE        0x07        //DMA write a 32b data
//In: [0:1d][1:type][2,3,4,5:ADDR][6,7,8,9:data]
#define CMD_TAP_DMABLKRD32        0x08        //DMA Read a data block
//In: [0:1d][1:type][2,3,4,5:ADDR][6:len, mode in type]  out:[len in data mode]
#define CMD_TAP_FLSHBLKWR         0x09        //flash blk write
//In: <cmd head><data 0 seq><data 1 seq>...<data 7 seq>
//<cmd head>: <[0:1d][1:type][2,3,4,5:BASE ADDR][6:len][7,8,9,10:WR Target ADDR]>
//<data x seq>:<0:I1><1:I2><2:I3><3:D><4,5,6,7:data><8:L><9:us>
//I: 0x80~0x8F, flash unlock cmd, I1~I3 for AMD is 0xAA, 0x55, 0xA0.
//         if open bypass mode, only I3 used 0xA0 at datax seq offset 0
//D: 0x00, follow 4 bytes are data, whatever the write type(x8,x16)
//L: 0xFF, means flash program waiting, 
//         waiting time is give in follow us bytes
//for a word program, maximum write step is 5, (I1,I2,I3,D,L)
//for bypass mode, write step is 3, (I3,D,L)
//
//<out>: None
//
//
//type:
#define CMDT_MOD        0x03      //op type mask, word, halfw, byte,tripw
#define CMDT_END       (1<<2)     //endian mask
#define CMDT_DMA       (1<<3)     //1:DMA       2:Pracc
#define CMDT_IRLEN(x)  ((x &0x0F)<<4)      //ir len mask
//
//****************************************************************************
//****************************************************************************
//Bit sequence scan: bit by bit scan TDI/TMS to DUT and return TDO(len<=800 bits)
//<=202 bytes)
//
//In Byte:  [ 0,1 ]  [ 2,3,4...xxx][xxx+1,...,2xxx-1]
//          bit LEN      TDI             TMS
//
//Out Byte:  [0,1,2,3,4...xxx-2] 
//                   TDO 
//****************************************************************************

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


static void u_getconfig(void);
static int  u_get_mcustate(void);
static void u_get_mcuversion(void);
static void u_set_speed(int cy);
static void u_buf_write_x16(DWORD addr, DWORD data);
static void u_buf_write_x8(DWORD addr, DWORD data);
static void u_buf_delayus(DWORD us);
static void u_buf_write(DWORD data);
static void u_bufcmd_delayus(DWORD us);
static void u_buf_write_I(DWORD id);
static void fill_cable_prop(void *p);

//--------------------pub-------------------------------------
int uinit(void *);
void uclose(void);
void utest_reset(void);
DWORD udet_instr(void);
DWORD uset_instr( DWORD instr);
DWORD uReadWriteData(DWORD data);
DWORD uReadData(void);
void uWriteData(DWORD data);
DWORD uejtag_dma_read_x(DWORD addr, int mode);
void uejtag_dma_write_x(DWORD addr, DWORD data, int mode);
int u_sflash_blkread(DWORD Inaddr, DWORD* pbuff, int len);
int u_sflash_blkwrite(DWORD Inaddr, DWORD* pbuff, int len, int flpg_x8);

#endif






