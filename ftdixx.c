/*
 * created by hugebird @ chinadsl.net 5/5/2010 rev1.9c
 *
 *
 * FTDI FT2232D JTAG application in MPSSE mode for Brjtag.
 * Copyright (C) 2010 Hugebird
 *
 * This code is covered by the GPL v2.
 *
 *
 */

 

#define FTDIXXX

#if ( defined(_MSC_VER)  )
#define WINDOWS_VERSION
#else
#include "WinTypes.h"
#endif        

#include "zjtag.h"

#ifdef WINDOWS_VERSION
#include "ftd2xx_w.h"
#else
#include "ftd2xx.h"
#endif

#include "ftdixx.h"

#define CHECK_WRITE 1 // whether to check the write date inline

#define DEBUGMSG   0 
#define DEBUGINPUT 0        // check input buffer data
#define DEBUGRX    0        // check flushing read data.
#define DEBUGRXBUF 0        // check total read data.
#define DEBUGTXBUF 0        // check total TX data.
#define DEBUGPUSH  0        // add byte
#define DEBUGFLUS  0        // scan blk
#define DEBUGDSC   0        // check ouput buffer dsc data	//DSC: data shift command for FT2232
#define DEBUGFETCH 0        // check dma fetch 
#define DEBUGBLK   0        // check dma blk, dsc blk, output buffer usage.
#define DEBUGTRAINING 0

#if (DEBUGMSG || DEBUGINPUT ||DEBUGRX || DEBUGRXBUF || DEBUGTXBUF ||DEBUGPUSH    \
     || DEBUGFLUS || DEBUGDSC || DEBUGFETCH || DEBUGBLK || DEBUGTRAINING )
#define DBG(x)     printf x
#else
#define DBG(x)
#endif


static FT_HANDLE ftHandle;             // Handle of the FTDI device
static FT_STATUS ftStatus;             // Result of each D2XX fn call
static DWORD dwNumDevs = 0;                // The number of devices
static FT_DEVICE_LIST_INFO_NODE *devInfo;
static DWORD dwClockDivisor = 0x00;     // Value of clock divisor, SCL Frequency = 12/((1+x)*2) (MHz) = 1Mhz
static DWORD dwClockDivisorH = 0x05DB;  // Value of clock divisor, SCL Frequency = 60/((1+0x05DB)*2) (MHz) = 1Mhz

static BYTE  *OutputBuffer;             //Output buffer, to device
static BYTE  *InputBuffer;              //Input buffer, from device
static DWORD *BitsToXferReg;            //RW/RO data blk bit length for a output DSC block, pending read same bits
static DWORD *TMSclkXferReg;            //RW/ROdata blk xfer TMS clock length for a DSC block
static DWORD *RxDataBuffer;             //Rx Data buffer, get from Input buffer, to App
//static DWORD *RxDataRdBlkSize;          //keep read len, for future use
static DWORD *TxDataBuffer;             //TX Data buffer, get from App, to output buffer
static DWORD *TxDataBlkType;            //Tx data type
static DWORD *TxDataBlkBits;            //Tx data bit len


static DWORD gTxFlushingBlkIndex = 0;      //flushing blk index
static DWORD gFlushingBytelen   = 0;       //flushing output buffer len, for each output buffer flushing , byte len to send out
static DWORD gInputBufferToRead = 0;       //flushing input buffer len, for each output buffer flushing return data
static DWORD BitsToXferRegIndex = 0;       //Ro/RW DSC blk count for one flushing, to decode the input buffer
static DWORD gRxDataBufferIndex = 0;       //total read data byte index
static DWORD gRxDataBlkReadIndex = 0;      //total Data blk read index
static DWORD gTxDataBlkIndex = 0;          //total Data blk in Tx data buffer
static DWORD gNextLocOnDataRead = 0;       //Index for next data read in DMA fetch
static DWORD gDSClen = 0;
  
static JtagStates CurJtagST = Undefined ;     //save current JTAG stae;

static int lowspeed = 0;
#define BigEndian		(endian == __BE)

///////////////////////////////extern global////////////////////////////////////////////////////
extern int instruction_length;
extern int endian;
extern DWORD FLASH_MEMORY_START;
extern DWORD cmd_type;
extern int safemode;
extern int showgpio;
extern int silent_mode;
extern int bypass;
extern int flsPrgTimeout;
extern int ejtag_speed;

extern DWORD LL1;
extern BYTE  LL2;
extern DWORD LL3;
extern int set_LL3;
extern DWORD LL4;
extern DWORD LL5;
extern DWORD LL6;
extern DWORD LL7;
extern DWORD LL8;
extern DWORD LL9;
extern DWORD GG1;
extern DWORD GG2;
extern DWORD USBID;

//////////////D2XX fns call ///////////////////////////////////////////////////

#ifdef WINDOWS_VERSION

#define FTfunc(x) (_stdcall *x)

typedef FT_STATUS FTfunc( tFT_Open)(	int deviceNumber,	FT_HANDLE *pHandle	);
typedef FT_STATUS FTfunc( tFT_Close)(    FT_HANDLE ftHandle    );
typedef FT_STATUS FTfunc( tFT_Read)(    FT_HANDLE ftHandle,    LPVOID lpBuffer,
    DWORD dwBytesToRead,    LPDWORD lpBytesReturned    );
typedef FT_STATUS FTfunc( tFT_Write)(    FT_HANDLE ftHandle,    LPVOID lpBuffer,
    DWORD dwBytesToWrite,    LPDWORD lpBytesWritten    );
typedef FT_STATUS FTfunc( tFT_Purge)(    FT_HANDLE ftHandle,	ULONG Mask	);
typedef FT_STATUS FTfunc( tFT_CreateDeviceInfoList)(	LPDWORD lpdwNumDevs	);
typedef FT_STATUS FTfunc( tFT_GetDeviceInfoList)(	FT_DEVICE_LIST_INFO_NODE *pDest,	LPDWORD lpdwNumDevs	);
typedef FT_STATUS FTfunc( tFT_ResetDevice)(    FT_HANDLE ftHandle	);
typedef FT_STATUS FTfunc( tFT_SetUSBParameters)(    FT_HANDLE ftHandle,
    ULONG ulInTransferSize,    ULONG ulOutTransferSize	);    
typedef FT_STATUS FTfunc( tFT_SetChars)(    FT_HANDLE ftHandle,
	UCHAR EventChar,	UCHAR EventCharEnabled,	UCHAR ErrorChar,	UCHAR ErrorCharEnabled    );
typedef FT_STATUS FTfunc( tFT_SetTimeouts)(    FT_HANDLE ftHandle,	ULONG ReadTimeout,	ULONG WriteTimeout	);
typedef FT_STATUS FTfunc( tFT_SetLatencyTimer)(    FT_HANDLE ftHandle,    UCHAR ucLatency    );
typedef FT_STATUS FTfunc( tFT_SetBitMode)(    FT_HANDLE ftHandle,    UCHAR ucMask,	UCHAR ucEnable    );
typedef FT_STATUS FTfunc( tFT_SetDivisor)(    FT_HANDLE ftHandle,	USHORT Divisor );


static tFT_Open _FT_Open;
static tFT_Close _FT_Close;
static tFT_Read _FT_Read;
static tFT_Write _FT_Write;
static tFT_Purge _FT_Purge;
static tFT_CreateDeviceInfoList _FT_CreateDeviceInfoList;
static tFT_GetDeviceInfoList _FT_GetDeviceInfoList;
static tFT_ResetDevice _FT_ResetDevice;
static tFT_SetUSBParameters _FT_SetUSBParameters;
static tFT_SetChars _FT_SetChars;
static tFT_SetTimeouts _FT_SetTimeouts;
static tFT_SetLatencyTimer _FT_SetLatencyTimer;
static tFT_SetBitMode _FT_SetBitMode;
static tFT_SetDivisor _FT_SetDivisor;

HINSTANCE ftDll = NULL;
static void loadftdll(void);

#else

#define  _FT_Open  FT_Open
#define  _FT_Close FT_Close
#define  _FT_Read  FT_Read
#define  _FT_Write FT_Write
#define  _FT_Purge FT_Purge
#define  _FT_CreateDeviceInfoList FT_CreateDeviceInfoList
#define  _FT_GetDeviceInfoList  FT_GetDeviceInfoList
#define  _FT_ResetDevice     FT_ResetDevice
#define  _FT_SetUSBParameters   FT_SetUSBParameters
#define  _FT_SetChars  FT_SetChars
#define  _FT_SetTimeouts FT_SetTimeouts
#define  _FT_SetLatencyTimer  FT_SetLatencyTimer
#define  _FT_SetBitMode  FT_SetBitMode
#define  _FT_SetDivisor  FT_SetDivisor

#endif

/////////////////////////////////////////////////////////////////////////////



/////////////////////////FT2232 Dxxx Driver fn call/////////////////
// wait us in RTI, for delay purpose; 
// max add 106 bytes to output buffer 
// limit less than 128us

static void FTDI_WaitLongInRTI(int us)
{
	int i;
	int nCount;
	int nBytes, nBits;
	

  if( us >128) us =128;
	nCount = (us * LL1)/1000;
  if(nCount <0) nCount = 1;
	nBytes = nCount >> 3;
	nBits = nCount & 7;
	
	if (CurJtagST != JTAG_RTI) FTDI_MoveJTAGStateTo(JTAG_RTI, 0, 0);
	if (nBytes)
		{	
	   FTDI_AddByteToOutputBuffer(DSC_BYTES_OUTN_LSB, 0);
     FTDI_AddByteToOutputBuffer((nBytes -1), 0);
	   for (i=0;i<=nBytes;i++)  FTDI_AddByteToOutputBuffer(0, 0);  // push N + 1 bytes
	  }
	 if(nBits)  FTDI_WaitInRTI(nBits);
}


// wait 1-7 tcks in RTI, for delay purpose;
// max add 3 bytes to output buffer 
static void FTDI_WaitInRTI(int n)
{
	int i;
	if (n< 1) n = 1;
	if (n >7) n = 7;          //max allow delay 128 tcks
	if (CurJtagST != JTAG_RTI) FTDI_MoveJTAGStateTo(JTAG_RTI, 0, 0);
	FTDI_AddByteToOutputBuffer(DSC_BITS_OUTN_LSB, 0);
  FTDI_AddByteToOutputBuffer(n, 0);
	FTDI_AddByteToOutputBuffer(0, 0);
}

// wait Nx 7tcks, for delay purpose;
// limit per TMS shift in a DSC block <= 7 tcks
static void FTDI_WaitTckMute(int n)
{
	int i;
	if (n< 1) n = 1;
	for (i=1;i<n;i++)  
	{
   FTDI_AddByteToOutputBuffer(DSC_SET_GPIOL_BITS, 0);  //GPIO low
   FTDI_AddByteToOutputBuffer((0x8 | ((GG1&0xF )<<4)), 0);  //0xC8
   FTDI_AddByteToOutputBuffer((0xB | ( GG2&0xF0)    ), 0);  //0x1B
  }
}
static void FTDI_WaitInTLR(int n)
{
	int i;
	if (n< 1) n = 1;
	for (i=1;i<n;i++)  FTDI_ClockOutTMS('\x7F', 7, 0);
  CurJtagST = JTAG_TLR;
}

// clock out TMS and 1 bit TDI
static void FTDI_ClockOutTMS(DWORD vTMS, DWORD nTMSclks, int Is_Read)
{
  if (nTMSclks > 7) nTMSclks = 7;
  if ((nTMSclks >= 1) && (nTMSclks <= 7))
  {
    if (Is_Read)
      FTDI_AddByteToOutputBuffer(DSC_TMSN_IN_LSB, 0);
    else
      FTDI_AddByteToOutputBuffer(DSC_TMSN_LSB, 0);

    FTDI_AddByteToOutputBuffer(((nTMSclks - 1) & 0xFF), 0);
    FTDI_AddByteToOutputBuffer((vTMS & 0xFF), 0);
  }
}


// Move Jtag state to State2, with 1x TDI bit. 
// returns the required tck length to change Jtag state by shift TMS
static DWORD FTDI_MoveJTAGStateTo(JtagStates JtagST2, DWORD Tailbit, int Is_Read)
{
  DWORD nTMSclks = 0;

  if (DEBUGDSC) 
  	{
  		DBG((" ---->We are in JTAG ST move, from %d to %d\n", CurJtagST, JtagST2 ));
    }
  
  if (CurJtagST == Undefined)
  {
    FTDI_ClockOutTMS('\x7F', 7, 0);
    CurJtagST = JTAG_TLR;
  }
	
  nTMSclks = TMSCLK_JTAGST_FromTo[CurJtagST][JtagST2];
  FTDI_ClockOutTMS((JTAGST_FromTo[CurJtagST][JtagST2] | (Tailbit << 7)), nTMSclks, Is_Read);

  CurJtagST = JtagST2;
  
  if (DEBUGDSC) DBG((" ---->Post ST mvoe.cur JTAG ST: %d -  TMS clk count: %d\n", CurJtagST,nTMSclks ));
  return nTMSclks;
}

// Write output buffer to usb port
// read answer to input buffer from usb port
static FT_STATUS FTDI_WriteReadWithDevice( int optype, DWORD BytesToRead, DWORD* BytesHaveRead)
{
  FT_STATUS Status = FT_OK;
  int i;
  DWORD HaveSent=0;

   if(gFlushingBytelen >0)
   {
     if (DEBUGBLK) 
     	   DBG(("--->>> output buffer byte len, dsc blk len = %d, %d\n\n",
     	                   gFlushingBytelen,gDSClen ));	
     gDSClen = 0;

     if (DEBUGDSC) 
     	{
     		 for (i = 0; i< gFlushingBytelen;i++) 
     		   DBG((">>>>> output buffer [%d] = %02X\n", i, OutputBuffer[i]));
     	}

     Status = _FT_Write(ftHandle, OutputBuffer, gFlushingBytelen, &HaveSent);
     if((HaveSent != gFlushingBytelen)||(Status != FT_OK)) 
  	  {
  		 printf("FT Write data to FT2232 error occur!\n");
  		 return Status;  //no read data
  	  }
  	  
     gFlushingBytelen = 0;
   }

   if( BytesToRead == 0) {*BytesHaveRead = 0; return Status;}
   	
    if ((optype != TAP_WO) && FT_SUCCESS(Status) && (BytesToRead>0))
    {
     Status =  _FT_Read(ftHandle, InputBuffer, BytesToRead, BytesHaveRead);
    }
    
    if((*BytesHaveRead != BytesToRead)||(Status != FT_OK)) 
  	{
  		 printf("FT Read data from FT2232 error occur!\n");
  		 return Status;
  	}
  	
   if(DEBUGINPUT) 
 	  { 
 		 DBG((">>>-->>Total %d bytes data read to Input buffer, read %d\n", BytesToRead, *BytesHaveRead));
 		 for (i=0; i< *BytesHaveRead;i++) 
 		   DBG(("Input buffer pending to decode [%d] = 0x%02X\n", i,InputBuffer[i]));
    }
 
  return Status;
}


///////////////////////////TAP scan out/in Between APP and USB Port/////////////////////////////////////////////
//     ___          _________________           ___________          ___________________      ___ 
//    |   |        |                 |         |           |        |    (queue)        |    |   |
//    |   |  Tx    | TX Data Buffer  |         | Scheduler |        | USB Output Buffer |    | 2 |
//    | A |------> |_________________| ------->|           |------> |___________________|--->| 2 |
//    | P |         _________________          |   scan    |         ___________________     | 3 |
//    | P |<------ |                 | <-------|   xfer    |<------ |     (queue)       |<---| 2 |
//    |   |  Rx    | RX Data buffer  |         |   Flush   |        | USB Input Buffer  |    |   |
//    |___|        | ________________|         |___________|        | __________________|    |___|
//
//    DWORD              BYTE                     Add DSC                  BYTE
/////////////////////////////////////////////////////////////////////////////////////////////////////////                                                                                                

// push a BYTE Tx data to output buffer
static void FTDI_AddByteToOutputBuffer(DWORD Data, int Is_clear)
{
  if (Is_clear) gFlushingBytelen = 0;

  OutputBuffer[gFlushingBytelen] = (BYTE)(Data & 0xFF);
  if (DEBUGPUSH) DBG(("--->Add byte to output buffer[%d] = 0x%02X\n",gFlushingBytelen, OutputBuffer[gFlushingBytelen]));
  gFlushingBytelen ++;
}

// push N x BYTE Tx data to output buffer
static void FTDI_AddNBytesToOutputBuffer(BYTE* pData, DWORD len, int Is_clear)
{
  if (Is_clear) gFlushingBytelen = 0;         //clear Tx data buffer
  memcpy((void *)(OutputBuffer + gFlushingBytelen), (void *)pData, (unsigned int) len);    
  gFlushingBytelen += len;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// calulate how many byte pending to read from usb port when exchange X bit with device
static void FTDI_CalcBlkBytesToRead(DWORD BitsToRead, DWORD* BytesNeedRead, DWORD* RemaingBitsNeedRead)
{
  DWORD N = 0;
  DWORD M = 0;

  BitsToRead --;                    // remove the tail-bit count for TMS mode
  N = ( (BitsToRead + 7)>>3 ) + 1;  // round up bytes to read, add one for TMS mode 
  M = 8 - (BitsToRead & 7);         // bit mode count M

  *BytesNeedRead = N;
  *RemaingBitsNeedRead = M;
}


// Add DSC for a Tx data block, push the DSC xfer block in output buffer
// after xfer, jtag move to St2 state. 
// is_instr, 1: Instr xfer, or 0: Data xfer
// optype: RW/RO/WO
static DWORD FTDI_AddDSCBlkToOutBuffer(int is_Instr, int optype,BYTE* buff, DWORD NumBitsToXfer,DWORD JtagST2)
                                   
{
  DWORD nDSCBits = 0;
  DWORD nBytes = 0;
  DWORD i = 0;             // Tx Data Buffer Index
  DWORD nBits = 0;
  DWORD Tailbit = 0;
  DWORD nTailBitIndex = 0;
  DWORD nTMSclks = 0;
  DWORD DSC_bytemode;
  DWORD DSC_bitmode;
  int   DSC_tmsin;         //in TMS mode, enable TDO input?
  
  if (DEBUGDSC) DBG(("<<<<<<<<<<<<<<<<<<in add dsc cmd>>>>>>>>>>>>>>>>>>>\n"));
  if (optype == TAP_WO)
  	 {DSC_bytemode = DSC_BYTES_OUTN_LSB; DSC_bitmode = DSC_BITS_OUTN_LSB; DSC_tmsin = 0;}
  
  else if (optype == TAP_RO)
  	{DSC_bytemode = DSC_BYTES_IN_LSB; DSC_bitmode = DSC_BITS_IN_LSB; DSC_tmsin = 1;}
  
  else if (optype == TAP_RW)
  	{DSC_bytemode = DSC_BYTES_OUTN_IN_LSB; DSC_bitmode = DSC_BITS_OUTN_IN_LSB; DSC_tmsin = 1;}
  else 
  	return 0;
  gDSClen++;
  
  nDSCBits = (NumBitsToXfer - 1);     // bit - 1,  for tail bit transfer with TMS mode 

  if (is_Instr !=0 )
  	FTDI_MoveJTAGStateTo(JTAG_SIR, 0, 0);        // IR shift
  else 
    FTDI_MoveJTAGStateTo(JTAG_SDR, 0, 0);        // DR shift
    

  nBytes = nDSCBits >> 3;    // 1.get byte mode count

  if (nBytes > 0)
  {

    nBytes --;              // DSC cmd length count from 0

    // data scan out on -ve, scan in on +ve clk LSB
    FTDI_AddByteToOutputBuffer(DSC_bytemode, 0);             //byte mode, Write + Read
    FTDI_AddByteToOutputBuffer((nBytes & 0xFF), 0);          //push byte length low
    FTDI_AddByteToOutputBuffer(((nBytes>>8) & 0xFF), 0);     //push byte length high 

    nBytes ++;              // get back the bytes count
    if (optype != TAP_RO)
    { 
       FTDI_AddNBytesToOutputBuffer(buff,nBytes ,0); //push N bytes data
    }
  }


  nBits = (nDSCBits & 7);   // 2.get bits need to send in bit mode

 if (nBits > 0)
  {
    nBits --;                           // DSC cmd length count from 0

    // data scan out on -ve, scan in +ve clk LSB
    FTDI_AddByteToOutputBuffer(DSC_bitmode, 0);
    FTDI_AddByteToOutputBuffer((nBits & 0xFF), 0);
    if (optype != TAP_RO)
        FTDI_AddByteToOutputBuffer(buff[nBytes], 0);
  }
  
                            // 3.get tail bit TDI in TMS bit mode
 if (optype == TAP_RO)
	  Tailbit = 0;          //Read only
 else
  {
   
    Tailbit = buff[nBytes];
    nTailBitIndex = NumBitsToXfer & 7;

    if (nTailBitIndex == 0)
      Tailbit  >>= 7;    // tail bit at bit7, shift to bit0, TDI out from bit0 in TMS mode
    else
      Tailbit >>= (nTailBitIndex - 1);  // shift tail bit to bit0,
  }
  
   // tail bit xfer with TMS state change, Tailbit for TDI output, enable tmsin for TDO input
    nTMSclks = FTDI_MoveJTAGStateTo( (JtagStates)JtagST2, Tailbit, DSC_tmsin);

  return nTMSclks;
}

// push delay block to tx data queue, n in us
static void FTDI_AddBlkDelay(int n)	
{
  if (n > 512) n = 512;
	while( n >= 128)
	{
	 FTDI_AddBlkToTXDataBuffer(0,TAP_TMS_DELAY, 128, 0);
	 n -= 128;
  }
  if (n)
	  FTDI_AddBlkToTXDataBuffer(0,TAP_TMS_DELAY, n, 0);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// scan ouput buffer data to FT2232, scan data from FT2232 into input bufer
// Decode input buffer, filling to RX Data buffer
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int FTDI_scan_flush(int Is_clear)
{
  FT_STATUS Status = FT_OK;
  DWORD M = 0;      // Remaining Bit length, shift with bit mode
  DWORD K = 0;      // TMS clock count for  tail bit shift and TMS state change, in TMS mode
  DWORD N = 0;      // total byte count need read, include bit mode and TMS mode.
  DWORD i = 0;      // read block index
  DWORD j = 0;      // input buffer read out index
  DWORD d = 0;      // decoded data len in DWORD saved in RX data buffer
  DWORD p;          // data blk len in input buffer before decoding
  DWORD xbits;      // xfer bit length for current block
  BYTE Tailbit = 0;
  DWORD tmpBytesRead = 0;
  int ii;

  if (Is_clear) {gRxDataBufferIndex = 0; } //clean Rx data buffer
 	                                         //keep for simple cmd
  FTDI_AddByteToOutputBuffer(DSC_SEND_FDBK_IMMD, 0);             // get rx at once after tx
  Status = FTDI_WriteReadWithDevice(TAP_RW, gInputBufferToRead, &tmpBytesRead);  //Write and Read
  if((Status != FT_OK)) return (int)Status;
 
  while ( j < gInputBufferToRead)  //decode all Ro/RW data in input buffer to Rx data buffer 
    {
      xbits = BitsToXferReg[i]; 
      K = TMSclkXferReg[i++];     // need point next block after taking saved record
      if (xbits == 0) continue;

      FTDI_CalcBlkBytesToRead(xbits, &N, &M);  //calculate N bytes to read, M bit to shift
      p = N;

//      if(DEBUGRX)  DBG((" block[%d] need read %d bits, %d tms\n", i, xbits, K));

      // adjust last 2 bytes
      if (M < 8)     //bit mode read & TMS bit mode read

      {
       Tailbit = InputBuffer[j + N - 1] << (K - 1);   //TMS bit, left shift to bit7
       Tailbit = Tailbit & 0x80;              // keep bit7 only
       InputBuffer[j + N - 2] = (InputBuffer[j + N - 2]>> M) | (Tailbit >> (M - 1));
       N --; // actual data byte len

      }
      else // M=8, no bit mode shift occur,  only TMS bit mode read
      {

       Tailbit = InputBuffer[j + N - 1] << (K - 1);
       InputBuffer[j + N - 1] = Tailbit >> 7;
      }

      d = ((N+3)>>2);  //read d x DWORDs
      RxDataBuffer[gRxDataBufferIndex + d - 1] = 0; //clean last DWORD
//      RxDataRdBlkSize[gRxDataBufferIndex] = d;      //keep data blk len in DWORD
      memcpy((void *)(RxDataBuffer+gRxDataBufferIndex), (void *)(InputBuffer+j), (unsigned int) N);
      
      if(DEBUGRX) 
  	     { 
 		      for (ii=0; ii< d;ii++) 
 		      DBG(("decode Rx data [%d] = 0x%08X,0x%08X\n", gRxDataBufferIndex+ii,*((DWORD *)(InputBuffer+j)),RxDataBuffer[gRxDataBufferIndex+ii] ));
         }
         
      j += p;                   // point to next pending decoded input blk
      gRxDataBufferIndex += d;  // decoded data save to Rx buffer 
    }

   gInputBufferToRead = 0;  //clear input buffer
   BitsToXferRegIndex = 0;  //clear block xfer reg
      
   return (int)Status;
}

///////////////////////////////////////////////////////////////////////////
//translate a data blk to dsc blk, fill out output buffer. 
//Not scan out to FT2232 immediately, just add to queue.
///////////////////////////////////////////////////////////////////////////
static int FTDI_scan_blk_post(int is_Instr, int optype, BYTE *buff, DWORD BitsToXfer, int do_flush, 
                                       DWORD JtagST2,int Is_clear)
{
  FT_STATUS Status = FT_OK;
  DWORD M = 0;      // Remaining Bit length, shift with bit mode
  DWORD K = 0;      // TMS clock count for  tail bit shift and TMS state change, in TMS mode
  DWORD N = 0;      // total byte length need read, include bit mode and TMS mode.
  BYTE Tailbit = 0;

  if (BitsToXfer<=0) return 0; 
  if (Is_clear)    //keep for simple cmd add data
  	{
  		FTDI_CleanFlushIndex();
  	} 

// translate raw data to Data Shift Command, return TMS clk count for tail bit
   K = FTDI_AddDSCBlkToOutBuffer(is_Instr, optype, buff, BitsToXfer, JtagST2);

  if ( (optype == TAP_WO) && do_flush )
  	     FTDI_AddByteToOutputBuffer(DSC_SEND_FDBK_IMMD, 0);  // let data shift out at once
  if( (gFlushingBytelen % 3948)==0) FTDI_AddByteToOutputBuffer(DSC_SEND_FDBK_IMMD, 0);
  
  if (optype != TAP_WO)  //record RO/RW blk xfer bits len, TMS clk info for later read out decoding
  	{  
       FTDI_CalcBlkBytesToRead(BitsToXfer, &N, &M);  //calculate N bytes to read, M bit to shift
       
       BitsToXferReg[BitsToXferRegIndex] = BitsToXfer;     //save data blk bit len.
 	     TMSclkXferReg[BitsToXferRegIndex] = K;              //save TMS clk shift count,
	     BitsToXferRegIndex ++;
	     gInputBufferToRead += N;                            //total bytes pending to read

      if (DEBUGPUSH)  
      	DBG(("push data block [%d] (L,Ltms)= %d, %d\n", BitsToXferRegIndex, BitsToXfer,K));
    }
    
  return Status;
}

///////////////////////////////////////////////

static void FTDI_scan_blk (int BlkIndex )
{

   int ii,flag;
   JtagStates NxtJtagST;
   DWORD blktype,blkbitlen,data;

    	 blktype    = TxDataBlkType[gTxFlushingBlkIndex]; 
    	 blkbitlen  = TxDataBlkBits[gTxFlushingBlkIndex];
    	 data       = TxDataBuffer [gTxFlushingBlkIndex];

   if (DEBUGFLUS)
   	 {
   	  DBG(("->Add (T,L,V) to output buffer, 0x%02X,%d,0x%08X\n",blktype,blkbitlen,data));
   	  DBG(("  cur JTAG ST = %d\n",(int)CurJtagST));
   	  DBG(("  cur gFlushingBytelen = %d\n", gFlushingBytelen));
   	 }
   	   	 
      switch( blktype & 0xF )
      {
        case TAP_TMS_DELAY:
        	 FTDI_WaitInTLR(1);
        	 FTDI_WaitLongInRTI(blkbitlen);
           FTDI_WaitInTLR(1);
           break;
            
        case TAP_TMS_MOVE:
        	 NxtJtagST = (JtagStates)data;
           FTDI_MoveJTAGStateTo(NxtJtagST, 0, 0);
           break;
             
        case TAP_IR:
           data &=  bitmask32(blkbitlen);
           FTDI_scan_blk_post(1, blktype&0xF0,(BYTE*)&data, blkbitlen, 0,JTAG_RTI,0); 
           break;      

        case TAP_DATA:
 				default:       // 32b
            
           data &=  bitmask32(blkbitlen);
           FTDI_scan_blk_post(0, blktype&0xF0,(BYTE*)&data, blkbitlen, 0,JTAG_RTI,0); 
           
       }

       gTxFlushingBlkIndex++;     // go next block
            
    if (DEBUGFLUS)
   	  {
   	     DBG(("  post JTAG ST2 = %d\n",(int)CurJtagST));
   	     DBG(("  post gFlushingBytelen = %d\n", gFlushingBytelen));
   	  }

}


static int FTDI_scan_xfer (void)
{
    
    DWORD blktype;
    int blkbitlen;
    int ii;
    int st = 0;
    int bk =0;

    if (gTxDataBlkIndex == 0) return 0;   //total blk to flush
    FTDI_CleanFlushIndex();

//    	 gTxFlushingBlkIndex = 0;      //flushing blk index
//    	 gFlushingBytelen = 0;         //flushing output buffer len 
//    	 gInputBufferToRead = 0;       //flushing input buffer len
//    	 gRxDataBufferIndex = 0;       //total read data byte index
//    	 gRxDataBlkReadIndex = 0;      //total Data blk read index

    if(DEBUGTXBUF)
   	{
   		 DBG((" We have total %d blks to flush\n",gTxDataBlkIndex));
   	   for(ii =0; ii<gTxDataBlkIndex; ii++)
   	   DBG(("--> tx data buffer to flush, index [%8d],(T,L,V)= 0x%02X,%2d,0x%08X\n-> ",
              ii, TxDataBlkType[ii], TxDataBlkBits[ii], TxDataBuffer[ii] ));
    } 

    while (	gTxFlushingBlkIndex < gTxDataBlkIndex)
    {
   	 
   	  if(DEBUGTXBUF) DBG((" In Flushing at location %d, with total bytes %d\n",gTxFlushingBlkIndex, gTxDataBlkIndex));
   		
      if (gFlushingBytelen >= LL5 )
     	  { 
     	   if(DEBUGTXBUF) DBG((" start Flushing ,%d bytes pending flush in output buffer\n",gFlushingBytelen));	
         st |= FTDI_scan_flush(0);
         if(DEBUGTXBUF) DBG((" end Flushing ,at location %d, out buffer byte len %d\n",gTxFlushingBlkIndex,gFlushingBytelen));
        } 
      else
        FTDI_scan_blk ( gTxFlushingBlkIndex );   //add one data block to output buffer
     }

     if (gFlushingBytelen >0)
     	{ 
       st |= FTDI_scan_flush(0);
       if(DEBUGTXBUF) DBG((" final Flushing ,at location %d\n",gTxFlushingBlkIndex));
      } 
      
   if(DEBUGRXBUF)
   	{
   	   for(ii =0; ii<gRxDataBufferIndex; ii++)
   	    DBG(("====> RxDataBuffer[%d] = %08X \n",ii, RxDataBuffer[ii] ));
   	    DBG(("\n\n\n"));
    } 
     gTxDataBlkIndex = 0;
     return st;  // st = 1 if error occur
}


static void FTDI_CleanFlushIndex(void)
{
  gTxFlushingBlkIndex = 0;      //flushing blk index
  gFlushingBytelen   = 0;       //flushing output buffer len, for each output buffer flushing , byte len to send out
  gInputBufferToRead = 0;       //flushing input buffer len, for each output buffer flushing return data
	BitsToXferRegIndex = 0;       //Ro/RW DSC blk count for one flushing, to decode the input buffer
  gRxDataBufferIndex = 0;       //total read data byte index
  gRxDataBlkReadIndex = 0;      //total Data blk read index
    	 
	gNextLocOnDataRead  = 0;      //DMA xfer read loc index

}


/////////////////////////////////////////////////////////////////////////////////////////////////
//push a N-bits data which stored as 32bit DWORD format to Tx buffer in byte-wise 
// N <=32
//interface between APP and Tx data buffer
//
// Assume Host Pc is little endian, Lowest byte push in buffer at lowest addr
/////////////////////////////////////////////////////////////////////////////////////////////////
// Queue/Bulk R/W
// Add blk to Tx Data buffer -> scan xfer -> scan blk ->scan blk post-> scan flush -> post process Rx Data buffer
//
// Simple cmd
// Add byte to output buffer -> scan flush -> post process Rx data buffer
//
static void FTDI_AddBlkToTXDataBuffer(DWORD data,int type, int bitlen, int Is_clear)
{
  if (Is_clear) {gTxDataBlkIndex = 0;}

  TxDataBuffer [gTxDataBlkIndex] = data;   // currently, only support 32b DWORD blk
  TxDataBlkType[gTxDataBlkIndex] = type;
  TxDataBlkBits[gTxDataBlkIndex] = bitlen;
  
    if (DEBUGTXBUF) 
  	{   		
      DBG(("-- add to Tx buffer, index [%08d],(T,L,V)= 0x%02X,%2d,0x%08X\n-> ",
              gTxDataBlkIndex, type, bitlen, TxDataBuffer[gTxDataBlkIndex] ));
    }
  
  gTxDataBlkIndex ++;

}

static DWORD bitmask32(int len)
{
 
  return ((len==32)? -1 : (1 << len) - 1 ) ;
}


///////////////////load d2xx dll dynamicly///////////////////////////

#ifdef  WINDOWS_VERSION

static void loadftdll(void)
{
  
  
  	if(ftDll == NULL) ftDll = LoadLibrary ("ftd2xx.dll");
    if(ftDll == NULL)
    {
        printf("Couldn't load ftd2xx DLL library\n");
        exit (1);
    }

  _FT_Open = (tFT_Open) GetProcAddress (ftDll, "FT_Open");

  _FT_Close = (tFT_Close) GetProcAddress (ftDll, "FT_Close");

  _FT_Read = (tFT_Read) GetProcAddress (ftDll, "FT_Read");

  _FT_Write = (tFT_Write) GetProcAddress (ftDll, "FT_Write");

  _FT_Purge = (tFT_Purge) GetProcAddress (ftDll, "FT_Purge");
 
  _FT_CreateDeviceInfoList = (tFT_CreateDeviceInfoList) GetProcAddress (ftDll, "FT_CreateDeviceInfoList");

  _FT_GetDeviceInfoList = (tFT_GetDeviceInfoList) GetProcAddress (ftDll, "FT_GetDeviceInfoList");
    
  _FT_ResetDevice = (tFT_ResetDevice) GetProcAddress (ftDll, "FT_ResetDevice");
 
  _FT_SetUSBParameters = (tFT_SetUSBParameters) GetProcAddress (ftDll, "FT_SetUSBParameters");

  _FT_SetChars = (tFT_SetChars) GetProcAddress (ftDll, "FT_SetChars");

  _FT_SetTimeouts = (tFT_SetTimeouts) GetProcAddress (ftDll, "FT_SetTimeouts");

  _FT_SetLatencyTimer = (tFT_SetLatencyTimer) GetProcAddress (ftDll, "FT_SetLatencyTimer");

  _FT_SetBitMode = (tFT_SetBitMode) GetProcAddress (ftDll, "FT_SetBitMode");
  
  _FT_SetDivisor = (tFT_SetDivisor) GetProcAddress (ftDll, "FT_SetDivisor");
  
  
    if( !_FT_Open || !_FT_Close ||  !_FT_Read || !_FT_Write || !_FT_Purge || !_FT_CreateDeviceInfoList || 
    	  !_FT_GetDeviceInfoList || !_FT_ResetDevice || !_FT_SetUSBParameters || !_FT_SetChars || 
    	  !_FT_SetTimeouts || !_FT_SetLatencyTimer || !_FT_SetBitMode || !_FT_SetDivisor )
    {
        printf("ftd2xx func link error\n");
        exit (1);
    }
  
}

#endif


static void ft_getconfig(void)
{
// performance fine tune parameters
//L1: FT2232 clk divisor>>> 0 ~ 0xFFFF  6MHz - 91.553Hz, default 0; freq = 6/(1+n)
//L2: FT2232 USB read latency>>> 2 ~ 255ms, default 4ms for 16KB buffer
//L3: DMA xfer Polling timeout>>> 0 ~ 30 us, defualt 1 us
//L4: FLASH Write Polling timeout>>> 1 ~ 127 us, defualt 16 us
//L5: USB buffer
//L6: Allowed DMA Bulk xfer DSC Block length>>>
//L7: Block Size on FLASH DWORD Read in x32Bits Mode>>>
//L8: Block Size on FLASH DWORD Write in x16Bits Mode>>>
//L9: Block Size on FLASH DWORD Write in x8Bits Mode>>>
//	printf(" LL1[%d],LL2[%d],LL3[%d],LL4[%d],LL5[%d],LL6[%d],LL7[%d],LL8[%d],LL9[%d]\n",LL1,LL2,LL3,LL4,LL5,LL6,LL7,LL8,LL9);

  if(!LL2) LL2 = FT_LATENCY;
  if(LL3==0 || LL3 ==0xFFFF) LL3 = DMA_POLLING;
  if(LL4==0xFFFF) LL4 = FLASH_POLLING;
  if(!LL5) LL5 = MAX_NUM_BYTES_USB_WRITE_READ - 400;     //400 for final delay block reserve
  if (LL5 >= (MAX_NUM_BYTES_USB_WRITE_READ - 400)) LL5 = MAX_NUM_BYTES_USB_WRITE_READ - 400;
  if (LL5 <4096) LL5 = 4096;
  if(!LL6) LL6 = DMA_BLOCK_SIZE;    
  if(!LL7) LL7 = 640;   //256->25418/22530 80%
  if(!LL8) LL8 = 80;    //32->31010/24386 80%
  if(LL9 ==1)   // safe mode
  	{ LL1 = 59;
  		LL2 = 10;
  		LL4 = 128;
  		LL8 = 2;
  		LL7 = 64;
  		LL3 = 48;
  	}
  else if(LL9 == 2)     //risk read mode
  	{ LL1 = 0;
  		LL2 = 2;
  		LL3 = 1;
  		LL4 = 8;
  		LL7 = 640;
  		LL8 = 80;
  	}
  	
  if(!GG1)  GG1 = 0x14;   //GPIOLo, value 0x4, dir 0x1
  if(!GG2)  GG2 = 0xFF;   //GPIOHi  value 0xF, dir 0xF
  //if(!USBID) USBID = 0x14575118;	//OpenMoko v2
  if(!USBID) USBID = 0x04038A98;	//TIAO TUMPA
//	printf(" LL1[%d],LL2[%d],LL3[%d],LL4[%d],LL5[%d],LL6[%d],LL7[%d],LL8[%d],LL9[%d]\n",LL1,LL2,LL3,LL4,LL5,LL6,LL7,LL8,LL9);

}

static void fill_cable_prop(void *p)
{
   cable_prop_type *pcbl = (cable_prop_type*)p;
   
     pcbl->feature = 0;
     pcbl->close = ftclose;
     pcbl->test_reset = fttest_reset;       
     pcbl->det_instr = ftdet_instr;
     pcbl->set_instr = ftset_instr;
     pcbl->ReadWriteData = ftReadWriteData;
     pcbl->ReadData = ftReadData;
     pcbl->WriteData = ftWriteData;
     

     pcbl->ejtag_dma_read_x = 0;
     pcbl->ejtag_dma_write_x = 0;

     pcbl->ejtag_pracc_read_x = 0;
     pcbl->ejtag_pracc_write_x = 0;

     pcbl->sflash_blkread = ft_sflash_blkread;
     pcbl->sflash_blkwrite = ft_sflash_blkwrite;
}

////////////////////// iNIT  FT2232D device /////////////////////////////////////////////////////
int ftinit(void *p)
{
 DWORD NumBytesToRead,NumBytesRead;
 int i;
 int ii=1;
 
 fill_cable_prop(p);
 ft_getconfig();
 
 RxDataBuffer  = (DWORD*) malloc(4*MAX_DATA_BLK_SIZE);
 TxDataBuffer  = (DWORD*) malloc(4*MAX_DATA_BLK_SIZE);
 TxDataBlkBits = (DWORD*) malloc(4*MAX_DATA_BLK_SIZE);
 TxDataBlkType = (DWORD*) malloc(4*MAX_DATA_BLK_SIZE);
 
 BitsToXferReg = (DWORD*)malloc(4*MAX_DSC_BLOCK_SIZE); 
 TMSclkXferReg = (DWORD*)malloc(4*MAX_DSC_BLOCK_SIZE);
 
 OutputBuffer  = (BYTE*) malloc(MAX_NUM_BYTES_USB_WRITE_READ);
 InputBuffer   = (BYTE*) malloc(MAX_NUM_BYTES_USB_WRITE_READ);

 
 if (  (BitsToXferReg == NULL) || (TMSclkXferReg == NULL)
 	   ||(OutputBuffer == NULL)  || (InputBuffer == NULL)
 	   ||(TxDataBlkBits == NULL) || (TxDataBlkType == NULL)
 	   ||(RxDataBuffer == NULL)  || (TxDataBuffer == NULL)  )
    { printf("Allocate Buffer error!!\n"); exit(1); }


#ifdef WINDOWS_VERSION
 ftDll = NULL;   
 loadftdll();
#endif 

  if (_FT_CreateDeviceInfoList(&dwNumDevs)!= FT_OK) // get number of FTDI device
   { printf("Error in getting FTDI devices count!\n"); exit(1); }

  if (dwNumDevs < 1) // Exit if we don't see any
   { printf("There are no FTDI devices installed\n");  exit(1); }

  devInfo = (FT_DEVICE_LIST_INFO_NODE*)malloc(sizeof(FT_DEVICE_LIST_INFO_NODE)*dwNumDevs); // get the device information list 
   if ((devInfo != NULL) && (_FT_GetDeviceInfoList(devInfo,&dwNumDevs) == FT_OK) )
	  {
	  	for (i = 0; i < dwNumDevs; i++) 
			 {

				 printf("Dev %d:\n",i);
				 printf(" Flags=0x%x\n",devInfo[i].Flags); 
				 printf(" Type=0x%x\n",devInfo[i].Type); 
				 printf(" ID=0x%x\n",devInfo[i].ID); 
				 printf(" LocId=0x%x\n",devInfo[i].LocId); 
				 printf(" SerialNumber=%s\n",devInfo[i].SerialNumber); 
				 printf(" Description=%s\n",devInfo[i].Description); 
				 printf(" ftHandle=0x%x\n",devInfo[i].ftHandle); 

	  		if(devInfo[i].ID == USBID && strstr(devInfo[i].Description," A")>0) 
					{ 
						ftStatus = _FT_Open(i, &ftHandle); 

/*						if ( ( strnicmp("TI", devInfo[i].SerialNumber, 2) != 0 ) || ( strnicmp("TIAO ", devInfo[i].Description, 5) != 0 ))
						{
							printf("TIAO USB Multi-Protocol Adapter not found\n"); exit(1);
						}
						*/
						ii=0;
						break;
					}				 
			 }
		}
  if(devInfo != NULL) free(devInfo);
  if (ii || ftStatus != FT_OK)
   { printf("FTDI device not find or open Failed with error %d\n", ftStatus); exit(1); }

ftStatus |= _FT_ResetDevice(ftHandle);                         //Reset USB device
ftStatus |= _FT_Purge(ftHandle, FT_PURGE_TX | FT_PURGE_RX);    //purge buffer
ftStatus |= _FT_SetUSBParameters(ftHandle, MAX_NUM_BYTES_USB_WRITE_READ, MAX_NUM_BYTES_USB_WRITE_READ);
                                                               //Set USB request Input buffer to 64K
ftStatus |= _FT_SetChars(ftHandle, 0, 0, 0, 0);                //Disable event and error characters
ftStatus |= _FT_SetTimeouts(ftHandle, 0, 5000);                //Sets the read and write timeouts in milliseconds, 0: infinite
ftStatus |= _FT_SetLatencyTimer(ftHandle, LL2);         //Set the latency timer to 1mS (default is 16mS)
ftStatus |= _FT_SetBitMode(ftHandle, 0x0, 0x00);               //Reset controller
mssleep(20);
ftStatus |= _FT_SetBitMode(ftHandle, 0xB, 0x02);               //Enable MPSSE mode
ftStatus |= _FT_ResetDevice(ftHandle); 
ftStatus |= _FT_Purge(ftHandle, FT_PURGE_TX | FT_PURGE_RX);

  if (ftStatus != FT_OK)
   { printf("Enable FTDI device MPSSE mode error %d\n", ftStatus); exit(1); }

// Configure the MPSSE GPIO initial settings for JTAG
// - GPIO low byte
//  Pin    Sig     Dir      Init State
// AD0   TCK/SK output 1   low  0    // Tck=0
// AD1   TDI/DO output 1   low  0    // TDI=0
// AD2   TDO/DI input  0   low  0    // TDO input
// AD3   TMS/CS output 1   high 1    // TMS=1
//
// AD4   GPIOL0 output 1   low  0    // nOE
// AD5   GPIOL1 input  0   low  0
// AD6   GPIOL2 input  0   high 1
// AD7   GPIOL3 input  0   high 1
//
// - GPIO high byte
// AC0   GPIOH0 output 1    1       //nTrst
// AC1   GPIOH1 output 1    1       //nSRST
// AC2   GPIOH2 output 1    0       //nTrst_OE
// AC3   GPIOH3 output 1    0       //nSrst_OE
////////////////////////////////////////////////////////

// Configure the MPSSE GPIO initial settings for JTAG
// - GPIO low byte
//  Pin    Sig     Dir      Init State     OpenMoko       JtagKey
//                      B         8
// ADBUS0 TCK/SK output 1    low  0       Tck=0
// ADBUS1 TDI/DO output 1    low  0       TDI=0
// ADBUS2 TDO/DI input  0    low  0       TDO input
// ADBUS3 TMS/CS output 1    high 1       TMS=1
//
//                      1         4
// ADBUS4 GPIOL0 output 1    low  0                        O nOE
// ADBUS5 GPIOL1 input  0    low  0                        I nVref_in
// ADBUS6 GPIOL2 input  0    high 1       I VCC3           I SRST_in
// ADBUS7 GPIOL3 input  0    high 0       I SRST_in
//
// - GPIO high byte     F       F
// ACBUS0 GPIOH0 output 1       1         O nTrst_OE       O  nTrst
// ACBUS1 GPIOH1 output 1       1         O nTrst          O  nSRST
// ACBUS2 GPIOH2 output 1       1         O nSrst_OE       O  nTrst_OE
// ACBUS3 GPIOH3 output 1       1         O nSrst          O  nSrst_OE




  mssleep(20);

  FTDI_AddByteToOutputBuffer(DSC_LOOPBACK_OFF, 1);    //Close loop back
  FTDI_AddByteToOutputBuffer(DSC_SEND_FDBK_IMMD, 0); 

  FTDI_AddByteToOutputBuffer(DSC_SET_GPIOL_BITS, 0);  //GPIO low
  FTDI_AddByteToOutputBuffer((0x8 | ( GG1<<4) ), 0);   //0x48 val
  FTDI_AddByteToOutputBuffer((0xB | ( GG1&0xF0) ), 0); //0x1B dir
  FTDI_AddByteToOutputBuffer(DSC_SEND_FDBK_IMMD, 0); 

  FTDI_AddByteToOutputBuffer(DSC_SET_GPIOH_BITS, 0);  //GPIO high
  FTDI_AddByteToOutputBuffer((GG2&0xF), 0);            //0x0F val
  FTDI_AddByteToOutputBuffer((GG2>>4 ), 0);            //0x0F dir
  FTDI_AddByteToOutputBuffer(DSC_SEND_FDBK_IMMD, 0); 
  ftStatus |= FTDI_WriteReadWithDevice( TAP_WO, 0, &NumBytesRead);
  if (ftStatus != FT_OK)
   { printf("FTDI device set GPIO state error %d\n", ftStatus); exit(1); }

  FTDI_AddByteToOutputBuffer(DSC_DISABLE_DIVIDE_5_CMD, 0);  //Disable Clk Divide by 5, so the frquence can be as high as 30Mhz
  FTDI_AddByteToOutputBuffer(DSC_SET_CLK_FREQ, 0);  //Setup I/O clk frequency
  FTDI_AddByteToOutputBuffer((LL1 & 0xFF), 0);
  FTDI_AddByteToOutputBuffer((LL1 >> 8)&0xFF, 0);
  ftStatus = FTDI_WriteReadWithDevice( TAP_WO, 0, &NumBytesRead);
 
//  ftStatus = _FT_SetDivisor( ftHandle, (WORD)(LL1&0xFFFF));
  if (ftStatus != FT_OK)
   { printf("FTDI device set clock frequency error %d\n", ftStatus); exit(1); }   

  LL1 = 30000/(LL1+1); //convert to khz
  printf(" Set I/O speed to %d KHz\n",LL1);
  if((LL1 < 100)||ejtag_speed) lowspeed = 1 ;

  if(showgpio)
   {
    FTDI_AddByteToOutputBuffer(DSC_GET_GPIOL_BITS, 1);
    FTDI_AddByteToOutputBuffer(DSC_SEND_FDBK_IMMD, 0); 
    ftStatus = FTDI_WriteReadWithDevice( TAP_RO, 1, &NumBytesRead);
    printf(" Read GPIO Low %X\n", InputBuffer[0]);
  
    FTDI_AddByteToOutputBuffer(DSC_GET_GPIOH_BITS, 1);
    FTDI_AddByteToOutputBuffer(DSC_SEND_FDBK_IMMD, 0); 
    ftStatus = FTDI_WriteReadWithDevice( TAP_RO, 1, &NumBytesRead);
    printf(" Read GPIO High %X\n", InputBuffer[0]);
  }

  ft_purgebuffer();
  mssleep(50); // Wait for all the USB stuff to complete and work
  
  if(ftStatus == FT_OK)
  	 return 1;
  	else 
  	 return 0;
} 

void ftclose(void)
{
	_FT_Purge(ftHandle, FT_PURGE_TX | FT_PURGE_RX);
	_FT_Close(ftHandle);
	
	free(InputBuffer);
	free(OutputBuffer);
	free(TMSclkXferReg);
	free(BitsToXferReg);
	free(TxDataBlkType);
	free(TxDataBlkBits);
	free(TxDataBuffer);
	free(RxDataBuffer);

#ifdef WINDOWS_VERSION	
  if (ftDll != NULL) FreeLibrary(ftDll);
#endif
	
}

////////////////////////////////////////////////////////////////////////////////////////////////
// Follow are brjtag common access methods.
// 
////////////////////////////////////////////////////////////////////////////////////////////////
void fttest_reset(void)
{
	DWORD tmpBytesRead ;
  if (CurJtagST == JTAG_RTI){
    FTDI_MoveJTAGStateTo(JTAG_TLR, 0, 0);
  }
  FTDI_MoveJTAGStateTo(JTAG_RTI, 0, 0);
  FTDI_AddByteToOutputBuffer(DSC_SEND_FDBK_IMMD, 0);
	FTDI_WriteReadWithDevice(TAP_WO, 0, &tmpBytesRead);
}


DWORD ftset_instr(DWORD instr)
{
  FT_STATUS Status;
  DWORD dd;

  FTDI_AddBlkToTXDataBuffer(instr, TAP_IR|TAP_RW, instruction_length,1);
  Status = FTDI_scan_xfer();
  if (Status != FT_OK)
   { printf("FT set_instr Failed with error %d\n", Status); 
    return 0;
    }
  else
  	{
	   dd = RxDataBuffer[0];
	   if (DEBUGMSG) DBG((">>>FT set_instr data =0x%08X\n", dd));
  	 return dd;
  	}
}

DWORD ftdet_instr(void)
{
  FT_STATUS Status;
  DWORD dd;

  FTDI_AddBlkToTXDataBuffer(0xFFFFFFFF, TAP_IR|TAP_RW, 32,1);
  Status = FTDI_scan_xfer();
  if (Status != FT_OK)
   { printf("FT det_instr Failed with error %d\n", Status); 
   	return 0;
   	}
  else
  	{
	   dd = RxDataBuffer[0];
	   if (DEBUGMSG) DBG((">>>FT det_instr data =0x%08X\n", dd));
  	 return dd;
  	}
}


// RW 32bits
DWORD ftReadWriteData(DWORD data)
{
  FT_STATUS Status;
  DWORD dd;
  if (DEBUGMSG) DBG((">>>FT RW/WR data =0x%08X\n", data));
  
  FTDI_AddBlkToTXDataBuffer(data, TAP_DATA|TAP_RW, 32,1);
  Status = FTDI_scan_xfer();
  if (Status != FT_OK)
   { printf("FT RW_Data Failed with error %d\n", Status); 
   	return 0xFFFFFFFF;
   	}
  else
    {
    dd = RxDataBuffer[0];
  	if (DEBUGMSG) DBG((">>>FT RW/Rd data =0x%08X\n\n", dd));
  	return dd;
  	}
}

// RO 32bits
DWORD ftReadData(void)
{
  FT_STATUS Status;
  DWORD dd = 0;
  
  FTDI_AddBlkToTXDataBuffer(0, TAP_DATA|TAP_RO, 32,1);
  Status = FTDI_scan_xfer();
  if (Status != FT_OK)
   { printf("FT Rd_Data Failed with error %d\n", Status);
   	return 0xFFFFFFFF; }
  else
    {
    	dd = RxDataBuffer[0];
    	if (DEBUGMSG)  DBG((">>>FT Rd data =0x%08X\n", dd));
  	  return dd;
  	}
}

// WO 32bits
void ftWriteData(DWORD data)
{
  FT_STATUS Status;
  
  FTDI_AddBlkToTXDataBuffer(data, TAP_DATA|TAP_WO, 32,1);
  Status = FTDI_scan_xfer();
  if (Status != FT_OK)
   { printf("FT Wt_Data Failed with error %d\n", Status); }
}


// fetch a DWORD from a BYTE-wise buffer
static DWORD getDWORD(BYTE * pAddr)
{
// DWORD* x;
// x = (DWORD *) pAddr;
//  return   *x;
  return (*((DWORD *) pAddr));
}

static void ft_purgebuffer(void)
{
	_FT_Purge(ftHandle, FT_PURGE_TX | FT_PURGE_RX);
	
  gTxDataBlkIndex = 0;
	FTDI_CleanFlushIndex();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// complete a DMA xfer with target CPU. data length <= 32bits
// Block/Bulk xfer to improve performance
//
// opdir : XFER_ADD: add DMA block to output buffer queue, 
//         XFER_QRY: fetch out data, flush buffer if needed.
//         XFER_NOOP: return cur internal dir
// dmadir: DMA_WR : DMA Write operate, 
//         DMA_RD : DMA Read operate
// mode  : DMA xfer data type: WORD(32bits), HALFWORD(16bits) or BYTE(8bits)
//     
// opdir |  Internal Dir  |   Action
//_______|________________|__________________________________________________________________________________________
//       |     Tx         | return available DMA xfer blcok count, return opdir = XFER_ADD
//  NOOP |________________|__________________________________________________________________________________________
//       |     Rx         | return 0, return opdir = XFER_QRY
//_______|________________|__________________________________________________________________________________________
//       |     Tx         | Add a xfer block to output buffr,return available DMA xfer blcok count opdir=XFER_ADD
//       |                | if buffer full, internal dir change to RX, opdir = XFER_QRY
//  ADD  |________________|__________________________________________________________________________________________
//       |     Rx         | return -1, add not success, opdir = XFER_QRY
//_______|________________|__________________________________________________________________________________________
//       |     Tx         | if Tx queue empty, return -1, fetch not success. opdir = XFER_ADD
//       |                | else force internal dir to Rx, flush output queue, do below (QRY+Rx) action
//  QRY  |________________|__________________________________________________________________________________________
//       |     Rx         | return read DWORD value, opdir set to XFER_RX
//       |                | if buffer empty, internal dir change to TX, opdir = XFER_ADD
//       |                |
//_______|________________|__________________________________________________________________________________________
//
///////////////////////////////////////////////////////////////////////////////////////////////////

#define CCT_CT   2
DWORD ft_dma_blkfetch(DWORD addr, DWORD txdata, int mode, int dmadir, int* opdir)
{
  DWORD rxdata;
  static int RWdir = XFER_TX;   //Internal queue op dir
                                //XFER_Tx: add app DMA data to output queue,
                                //XFER_Rx: read answer from RX data queue
  static int BlockIndex = 0;    //how many DMA xfer block add to queue
  static int BlockDirReg[MAX_DMA_BLOCK_SIZE];  //dma xfer direction, max keep 420 DMA xfer block for 64KB buffer
  static int fetchIndex =0;     //data read out index;
  int haveread=0;               //have read flag
  int Status;
  int i;
  DWORD cct;                    //control register value for DMA xfer

  set_LL3 = 0;      // here set_LL3 is alway equal 0, I don't want to polling control register value


  if (DEBUGFETCH)  
  	DBG(("\n\n>>>>> ftdi dma blk fetch start...\n\n"));
  	
  if(*opdir == XFER_NOOP)	
  	{
  		if( RWdir == XFER_TX )
  			{
  				*opdir = XFER_ADD;
  				return (LL6 - BlockIndex);
  			}
  		else
  			{
  				*opdir = XFER_QRY;
  				return 0;
  			}
  	}

 if (*opdir == XFER_ADD)  // ADD to queue op
	{
     if( RWdir == XFER_TX )
     	{	 
      if (DEBUGFETCH) DBG(("add queue dir TX\n\n"));

       //R/W
       //1  set_instr(INSTR_ADDRESS);	
       FTDI_AddBlkToTXDataBuffer(INSTR_ADDRESS, TAP_IR|TAP_WO, instruction_length, (BlockIndex == 0)?1:0); //WO
    
	   // added by tumpa
	   //FTDI_AddBlkDelay(100);
       //2   WriteData(addr);
       FTDI_AddBlkToTXDataBuffer(addr, TAP_DATA|TAP_WO, 32, 0);
    // added by tumpa
	   //FTDI_AddBlkDelay(100);
       if(dmadir == DMA_WR)
       	{
         //X/3.  set_instr(INSTR_DATA);
         FTDI_AddBlkToTXDataBuffer(INSTR_DATA, TAP_IR|TAP_WO, instruction_length,0);
    // added by tumpa
	   //FTDI_AddBlkDelay(100);
         //X/4.  WritedData(data);
         FTDI_AddBlkToTXDataBuffer(txdata, TAP_DATA|TAP_WO, 32, 0);
		 // added by tumpa
	   //FTDI_AddBlkDelay(100);
        }
       
       // Initiate DMA Read & set DSTRT
       //3/5    set_instr(INSTR_CONTROL);
       FTDI_AddBlkToTXDataBuffer(INSTR_CONTROL, TAP_IR|TAP_WO, instruction_length,0);
    // added by tumpa
	   //FTDI_AddBlkDelay(100);
       //4/6.  WriteData(DMAACC | DRWN | DMA_WORD | DSTRT | PROBEN | PRACC);
       FTDI_AddBlkToTXDataBuffer((DMAACC | ((dmadir == DMA_RD)?DRWN:0) | DMASZ(mode) | DSTRT | PROBEN | PRACC), TAP_DATA|TAP_WO, 32, 0);
    // added by tumpa
	   //FTDI_AddBlkDelay(100);
       // Wait for DSTRT to Clear
       //5/7.  flag = ReadWriteData(DMAACC | PROBEN | PRACC)& DSTRT;
     if (set_LL3)
      {
        for(i = 0; i<CCT_CT;i++)
         {
         FTDI_AddBlkDelay(LL3);
         FTDI_AddBlkToTXDataBuffer((DMAACC | PROBEN | PRACC), TAP_DATA|TAP_RW, 32, 0);
         }
      }
      else
      	{	FTDI_AddBlkDelay((dmadir == DMA_WR)?LL3+11:LL3);
      		if(lowspeed) FTDI_AddBlkDelay(24);   // add 24us latency
      		}
    
       if(dmadir == DMA_RD)
       	{
         //6/X.  set_instr(INSTR_DATA);
         FTDI_AddBlkToTXDataBuffer(INSTR_DATA, TAP_IR|TAP_WO, instruction_length,0);
    
		     // added by tumpa
	   //FTDI_AddBlkDelay(100);

         //7/X.  data = ReadData();
         FTDI_AddBlkToTXDataBuffer(0, TAP_DATA|TAP_RO, 32, 0);

		     // added by tumpa
	   //FTDI_AddBlkDelay(100);

         //8/X   set_instr(INSTR_CONTROL);
         FTDI_AddBlkToTXDataBuffer(INSTR_CONTROL, TAP_IR|TAP_WO, instruction_length,0);
		     // added by tumpa
	   //FTDI_AddBlkDelay(100);

	   }
    
       //9/8. (ReadWriteData(PROBEN | PRACC) & DERR)
       if (set_LL3)
        {
          for(i = 0; i<CCT_CT;i++)
          {
           FTDI_AddBlkToTXDataBuffer((DMAACC | PRACC), TAP_DATA|TAP_RW, 32, 0);
          }
         }
      FTDI_AddBlkDelay(100);
        if (DEBUGFETCH)
      	  DBG(("add current block DMA opdir [%d] = %d\n\n",BlockIndex, dmadir));
        
        BlockDirReg[BlockIndex] = dmadir;  //save current dma xfer dir, DMA write or DMA read
        BlockIndex ++;                      //next block index
 	 	  
       //check block reg queue is full?  if full, flush buffer, but not fetch out
       if(BlockIndex >= LL6)     //block xfer reg queue full, invert to RX
         {RWdir = XFER_RX;                  //next xfer is need fetch data; 
          fetchIndex = 0;                   //xfer block read direction index clean
          gNextLocOnDataRead = 0;           //read positon clean
          *opdir = XFER_QRY;                //indicate op invert to fetch from queue
          FTDI_AddByteToOutputBuffer(DSC_SEND_FDBK_IMMD, 0);    // flush the output queue
          Status = FTDI_scan_xfer();         // transmit input buffer into Rx Data buffer
          if (DEBUGFETCH) DBG(("DMA xfer block count = %d\n\n",BlockIndex));
          if (Status )
           { printf("FT DMA block Read flush buffer error %d\n", Status); }
          
          if (DEBUGFETCH)   
           	{
             DBG(("add queue full, invert to RX\n\n"));
             for (i=0; i< BlockIndex;i++) 
             DBG((" dma opdir after add queue[%d]= %d\n", i, BlockDirReg[i]));
            }
          return 0;    // return no room for next add queue
         } 
       else
       	 return (LL6 - BlockIndex); // return available dma block         
      }                // finish add xfer block to ouput buffer queue
     else              // if ( RWdir == XFER_RX )
     {
       *opdir = XFER_QRY;
       return -1;        //return add not success, all input buffer not empty
     }
    
  }  
 else    //if (*opdir == XFER_QRY)           // fetch data from queue
  {
     if( RWdir == XFER_TX  )            //force flush the ouput queue, and flip to Rx opdir, fetch first
  	 	{ 
  	 	 if (BlockIndex <=0)
  	 	 	{
  	 	 		*opdir = XFER_ADD;
  	 	 		 return -1;   // no data flush, return
  	 	 	}
  	 	
  	 	 RWdir = XFER_RX; 
//  	 	 FTDI_AddByteToOutputBuffer(DSC_SEND_FDBK_IMMD, 0);   
       
       Status = FTDI_scan_xfer();
       if (DEBUGBLK) DBG(("DMA xfer block count = %d\n\n",BlockIndex));
       if (Status )
         { printf("FT DMA block Read flush buffer error %d\n", Status); }
       
       fetchIndex = 0;               //xfer block read index clean
       gNextLocOnDataRead = 0;       //read data positon clean

       if (DEBUGFETCH) 
 	      { 
 	        DBG(("force flsh queue, invert to RX\n\n"));
 	      	for (i=0; i< BlockIndex;i++) 
 	        DBG((" dma opdir [%d]= %d\n", i, BlockDirReg[i]));   }
  	 	}   // if( RWdir == XFER_TX  )   force read out; and continue get first readable data
  	
       
          //  (XFER_RX) start fetch out --------------------------------------------------------
    
         if (DEBUGFETCH)
        	{   DBG(("pop queue RWdir RX\n\n"));
              DBG(("Curent data queue read location = %d\n\n",gNextLocOnDataRead));
              DBG(("pop queue dmadir [%d]= %d\n\n",fetchIndex, dmadir));
          }

       rxdata = 0;    
       haveread = 0; //data not fetchout
       do
        {
         dmadir = BlockDirReg[fetchIndex]; // get this dma dir
          if(haveread && (dmadir == DMA_RD)) break;    // data have read, second readable found. exit
          	                                           // guarantee next call can fetch a data successfully.
     if (set_LL3)
      {
         //1. check DMA start
#ifdef DEBUGFETCH
          for(i = 0; i<CCT_CT;i++)
           {
#else           		
           {
             	 gNextLocOnDataRead += (CCT_CT - 1);     // only read the final stable state
#endif
             cct = *( RxDataBuffer + gNextLocOnDataRead );	
             gNextLocOnDataRead ++;
             if ( (cct & DSTRT) && (!silent_mode) ) {printf("\n>>>DMA NOT start... %08X, index %d\n",cct,BlockIndex ); }
             if (DEBUGFETCH)
             	{ 
             		if( cct & DSTRT ) DBG((">>>DMA NOT start at %d ...\n", i));
             		DBG(("1.ctrl reg value[%d]= %08X\n",i,cct));
                DBG(("next queue read location = %d\n\n",gNextLocOnDataRead));
              }
           }
     } // if (LL3)
          if(dmadir == DMA_RD)
       	   {
   	         //2. read data
             rxdata = *( RxDataBuffer + gNextLocOnDataRead );
             gNextLocOnDataRead ++;
             haveread = 1;     //data read out;
             dmadir = DMA_WR; //search next readable data, but not fetch
             if (DEBUGFETCH)
             	{ DBG(("===fetch data value= %08X\n",rxdata));
                DBG(("next queue read location = %d\n\n",gNextLocOnDataRead));
              }
            }
        
          //3. check DMA error
      if (set_LL3)
      {
#ifdef DEBUGFETCH
           for(i = 0; i<CCT_CT;i++)
            {
#else           		
            {
             	 gNextLocOnDataRead += (CCT_CT - 1);     // only read the final stable state
#endif

             cct = *( RxDataBuffer + gNextLocOnDataRead );	
             gNextLocOnDataRead ++;
             if ( (cct & DERR) && (!silent_mode) ) {printf("\n>>>DMA error occur... %08X, index %d\n",cct,BlockIndex );}
             if (DEBUGFETCH)
             	{ 
             		if(cct & DERR) DBG((">>>DMA error occur %d ...\n", i));
             		DBG(("2.ctrl reg value[%d]= %08X\n",i,cct));
                DBG(("next queue read location = %d\n\n",gNextLocOnDataRead));
              }
           }
       } //if (LL3)    
            BlockIndex --;
            fetchIndex ++;
            if (DEBUGFETCH)  DBG(("next block location = %d\n\n",BlockIndex));
            if(BlockIndex <=0) 
    	         {
    	         	RWdir = XFER_TX;
    	         *opdir = XFER_ADD;      //indicat op change to ADD queue
    		        gNextLocOnDataRead = 0;
    		        break;                 //exit the while loop
    		       }
    		        
    	  } while ( dmadir == DMA_WR);   // search next readable Data
      
      return rxdata;
                                     // ( XFER_RX  )
    
 	}   //if (*opdir == XFER_QRY) 
  
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//
//FLASH Block Read, read N DWORDs
//
///////////////////////////////////////////////////////////////////////////////////////////////////

int ft_sflash_blkread(DWORD Inaddr, DWORD* pbuff, int len)
{
	DWORD data_lo, data_hi, data_poll;
  DWORD blocksize;
  int xfer_op = XFER_NOOP;  
  int dmadir = DMA_RD;
  int i, ii;
  int errflag=1;
  DWORD data, addr;
  int freebuffer, ilen;
//  static int xxx = 0;
  
  
//  printf("\n\n\n\n\n\n\n\n\n------------------------> enter ft_flash_blkread, len = %d \n",len);
  
    freebuffer = ft_dma_blkfetch(0, 0, 0, 0, &xfer_op);
    ilen = freebuffer / ( 1 );  // can xfer so many dma block, guarentee buffrr not overflow.
    len = (len > ilen) ? ilen : len;

   //ADD read command to queue
   for(ii = 0; ii < len; ii ++)
   {
   	addr = Inaddr + 4*ii;
   	xfer_op =XFER_ADD;
   	ft_dma_blkfetch(addr, 0, MIPS_WORD, DMA_RD, &xfer_op);
   }
   
   //Fetch data back
   for(ii = 0; ii < len; ii ++)
   {
   	xfer_op =XFER_QRY;
   	data = ft_dma_blkfetch(0, 0, MIPS_WORD, DMA_RD, &xfer_op);
   	if (BigEndian) data = rev_endian(data);
   	pbuff[ii] = data;
   }   


//if((xxx++) >1) exit(1);

   return len;

}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//FLASH a DWORD WRITE
//
///////////////////////////////////////////////////////////////////////////////////////////////////
#define POLL_CT  1
void ft_sflash_write_word(DWORD addr, DWORD data, int flpg_x8)
{
  DWORD data_lo, data_hi, data_poll;
  DWORD blocksize;
  int xfer_op;  
  int dmadir = DMA_WR;
  int i;
  int errflag=1;
  
// printf("\n\n\n\n\n\n\n\n\n------------------------> enter ft_sflash_write_word \n");

    xfer_op = XFER_ADD;
    data_poll = 0;
          
    if (flpg_x8) 
    	{  
         ft_sflash_write_x8((addr& (~3)), data, &xfer_op);
         FTDI_AddBlkDelay(LL4);

         ft_sflash_write_x8((addr & (~3))+ 1, data, &xfer_op);
         FTDI_AddBlkDelay(LL4);

         ft_sflash_write_x8((addr & (~3))+ 2, data, &xfer_op);
         FTDI_AddBlkDelay(LL4);

         ft_sflash_write_x8((addr & (~3))+ 3, data, &xfer_op);
         FTDI_AddBlkDelay(LL4);


      } else {
      	
      	ft_sflash_write_x16((addr& (~3)), data, &xfer_op);
         FTDI_AddBlkDelay(LL4);

         ft_sflash_write_x16((addr& (~3))+2, data, &xfer_op);
         FTDI_AddBlkDelay(LL4);
      }  
          
          //polling 
        if ( (cmd_type != CMD_TYPE_AMD) || !bypass )
        {
          for(i =0; i< POLL_CT ;i++)
          {ft_dma_blkfetch(addr, 0, MIPS_WORD, DMA_RD, &xfer_op);
//            FTDI_AddBlkDelay(1);
          }

          //check polling data
          errflag = 1;
          for(i =0; i< POLL_CT ;i++)
          { 
          	xfer_op = XFER_QRY;
          	data_poll = ft_dma_blkfetch(0, 0, MIPS_WORD, DMA_RD, &xfer_op);
          	if (BigEndian) data_poll = rev_endian(data_poll);
          	if(data_poll == data) errflag = 0;
 //         	printf(" fetch data_poll[%d] = %08X\n", i,data_poll);
          }       
          if (errflag) printf(" \ndma write not correctly !!\n");
        }
        else
        {
           xfer_op = XFER_QRY;
           ft_dma_blkfetch(0, 0, MIPS_WORD, DMA_RD, &xfer_op);
         }         
        //  exit(1);

}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//FLASH Block WRITE, write N DWORDs
//
///////////////////////////////////////////////////////////////////////////////////////////////////

int ft_sflash_blkwrite(DWORD Inaddr, DWORD* pbuff, int len, int flpg_x8)
{
	DWORD data_lo, data_hi, data_poll;
  DWORD blocksize;
  int xfer_op = XFER_NOOP;  
  int dmadir = DMA_WR;
  int i, ii;
  int errflag=1;
  DWORD data, addr;
  int freebuffer, ilen;
  
//  printf("\n\n\n\n\n\n\n\n\n------------------------> enter ft_flash_blkwrite, len = %d \n",len);

    freebuffer = ft_dma_blkfetch(0, 0, 0, 0, &xfer_op);

    ilen = freebuffer / ( 9 + 2);  // can xfer so many dma block, guarentee buffer not overflow.

    if(len == 0) return ilen;        // answer back the caller query result for max supported block size
    len = (len > ilen) ? ilen : len;
 
//   printf("flash bulk write len = %d\n", len);   	
   for(ii = 0; ii < len; ii ++)
   {
     data = pbuff[ii];
     addr = Inaddr + 4*ii;
     xfer_op = XFER_ADD;

    if (flpg_x8) 
    	{  
         ft_sflash_write_x8((addr& (~3)), data, &xfer_op);
         FTDI_AddBlkDelay(LL4);

         ft_sflash_write_x8((addr & (~3))+ 1, data, &xfer_op);
         FTDI_AddBlkDelay(LL4);

         ft_sflash_write_x8((addr & (~3))+ 2, data, &xfer_op);
         FTDI_AddBlkDelay(LL4);

         ft_sflash_write_x8((addr & (~3))+ 3, data, &xfer_op);
         FTDI_AddBlkDelay(LL4);

      } else {
      	
      	ft_sflash_write_x16((addr& (~3)), data, &xfer_op);
         FTDI_AddBlkDelay(LL4);

         ft_sflash_write_x16((addr& (~3))+2, data, &xfer_op);
         FTDI_AddBlkDelay(LL4);
      }   
          //polling 
     if ( CHECK_WRITE && (  (cmd_type != CMD_TYPE_AMD) || !bypass ) )
       {	
         for(i = 0; i < POLL_CT ;i++)
             {
             	ft_dma_blkfetch(addr, 0, MIPS_WORD, DMA_RD, &xfer_op);
//              FTDI_AddBlkDelay(1);       
             }
       }
	  }
	   
	    // check polling data
	    if ( CHECK_WRITE && ( (cmd_type != CMD_TYPE_AMD) || !bypass ) )
          {
	    
            for(ii = 0; ii < len; ii ++)
             {
   	          data = *(pbuff + ii);
               addr = Inaddr + 4*ii;

               errflag = 1;
               for(i =0; i< POLL_CT ;i++)
                { 
          	      xfer_op = XFER_QRY;
          	      data_poll = ft_dma_blkfetch(0, 0, MIPS_WORD, DMA_RD, &xfer_op);
//          	      printf("data_poll = %08X\n",data_poll);
          	      if (BigEndian) data_poll = rev_endian(data_poll);
          	      if(data_poll == data) errflag = 0;
                 }       
               if (errflag) printf("\n dma write not correctly !!\n %08X - %08X\n",data,data_poll);

               // 	if (errflag)exit(1);
              }
           }
           else
           	{
           		xfer_op = XFER_QRY;
          	  ft_dma_blkfetch(0, 0, MIPS_WORD, DMA_RD, &xfer_op);
           	}
//           	exit(1);
     return len ;
}


#define LEMASK16(k)   (0xffff<<(16*(k)))
#define BEMASK16(k)   (0xffff<<(16*(1-(k))))

static void ft_sflash_write_x16(DWORD addr, DWORD data, int *xfer_op)
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
     switch( cmd_type )
       {
        case CMD_TYPE_AMD:
        	if (!bypass)
        		{
              ft_dma_blkfetch(FLASH_MEMORY_START+(0x555 << 1), 0x00AA00AA, MIPS_HALFWORD, DMA_WR, xfer_op);
              ft_dma_blkfetch(FLASH_MEMORY_START+(0x2AA << 1), 0x00550055, MIPS_HALFWORD, DMA_WR, xfer_op);
            }
          ft_dma_blkfetch(FLASH_MEMORY_START+(0x555 << 1), 0x00A000A0, MIPS_HALFWORD, DMA_WR, xfer_op);
          break;
        case CMD_TYPE_SST:
          ft_dma_blkfetch(FLASH_MEMORY_START+(0x5555 << 1), 0x00AA00AA, MIPS_HALFWORD, DMA_WR, xfer_op);
          ft_dma_blkfetch(FLASH_MEMORY_START+(0x2AAA << 1), 0x00550055, MIPS_HALFWORD, DMA_WR, xfer_op);
          ft_dma_blkfetch(FLASH_MEMORY_START+(0x5555 << 1), 0x00A000A0, MIPS_HALFWORD, DMA_WR, xfer_op);
          break;
        case CMD_TYPE_BCS:
        case CMD_TYPE_SCS:
        default:
          ft_dma_blkfetch(addr, 0x00500050, MIPS_HALFWORD, DMA_WR, xfer_op);
          ft_dma_blkfetch(addr, 0x00400040, MIPS_HALFWORD, DMA_WR, xfer_op);
       }
 
          ft_dma_blkfetch(addr, odata, MIPS_HALFWORD, DMA_WR, xfer_op);
}

#define LEMASK8(k)    (0xff<<(8*(k)))
#define BEMASK8(k)    (0xff<<(8*(3-(k))))

static void ft_sflash_write_x8(DWORD addr, DWORD data, int *xfer_op)
{
  int k;
  DWORD odata,ldata;
  
  k = addr & 0x3;
  
 /*
       if (BigEndian)
       	  {
    	     odata = rev_endian(data) & BEMASK8(k);
    	     ldata = odata >> (8*(3-k));
    	    }
        else
          {
           odata = data & LEMASK8(k);
           ldata = odata >> (8*k);
          }
   */

    ldata = (data >> (8*k))&0xFF;
    odata = ldata | (ldata <<8);
    odata |= (odata<<16);
   
//     if( ldata == 0xff) return; // no need to program
     switch( cmd_type )
       {
        case CMD_TYPE_AMD:
        	if (!bypass)
        		{
              ft_dma_blkfetch(FLASH_MEMORY_START+(0xAAA), 0xAAAAAAAA, MIPS_BYTE, DMA_WR, xfer_op);
              ft_dma_blkfetch(FLASH_MEMORY_START+(0x555), 0x55555555, MIPS_BYTE, DMA_WR, xfer_op);
            }
           ft_dma_blkfetch(FLASH_MEMORY_START+(0xAAA), 0xA0A0A0A0,MIPS_BYTE, DMA_WR, xfer_op);
           break;
        case CMD_TYPE_SST:
        	return;  //SST 39 don't support x8
          break;
        case CMD_TYPE_BCS:
        case CMD_TYPE_SCS:
        default:
        	ft_dma_blkfetch(addr, 0x50505050,MIPS_BYTE, DMA_WR, xfer_op);
          ft_dma_blkfetch(addr, 0x40404040,MIPS_BYTE, DMA_WR, xfer_op);
       }
 
          ft_dma_blkfetch(addr, odata, MIPS_BYTE, DMA_WR, xfer_op);

}




