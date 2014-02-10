/*
 * created by hugebird @ chinadsl.net 5/5/2010 rev1.9c
 *
 *
 * J-Link JTAG application in bit-wise mode for Brjtag.
 * Copyright (C) 2010 Hugebird
 *
 * This code is covered by the GPL v2.
 *
 *
 */

 
 

#if ( defined(_MSC_VER)  )
#define WINDOWS_VERSION
#endif        

#define BJLINK


#include "zjtag.h" 
#include "j-link.h"    
#include "libusb.h"


#define DEBUGFLUSH 0
#define DEBUGTXBUF 0
#define DEBUGRXBUF 0
#define DEBUGINBUF 0
#define DEBUGOTBUF 0
#define DEBUGBLK   0
#define DEBUGMSG   0 
#define DEBUGFETCH 0        // check dma fetch 
#define DEBUGTRAINING 0

#if (DEBUGFLUSH || DEBUGTXBUF ||DEBUGRXBUF || DEBUGINBUF || DEBUGOTBUF     \
     || DEBUGBLK || DEBUGMSG || DEBUGFETCH || DEBUGTRAINING )
#define DBG(x)     printf x
#else
#define DBG(x)
#endif


static BYTE *OutputBuffer;           //Output buffer, to device
static DWORD gFlushingBitlen;        //output buffer total bit length
static DWORD gTxFlushingIndex;       //on flushing data block num in output buffer.
static BYTE *pTMSbuffer;
static BYTE *pTDIbuffer;

static DWORD *TxDataBuffer;          //Tx data buffer 
static DWORD gTxDataBlkIndex;        //tx data buffer total block num.
static int *TxDataBlkType;
static int *TxDataBlkBits;

static BYTE  *InputBuffer;           //input buffer, from device
static DWORD gInputBufferIndex;      //input buffer blk index
static int *InputBlkReadLoc;         //read data location in input buffer

static DWORD *RxDataBuffer;           // Rx buffer
static DWORD gRxDataBufferIndex;     // Rx data blk num

static int jlspeed;

static JtagStates CurJtagST = Undefined;     //save current JTAG state;
static JtagStates NxtJtagST = JTAG_TLR;      //next state 
static BYTE cmdbuf[200];
static int JTAGRW_VER = 3;       // ver>=5 use JTAG3, <5 use JTAG2 RW cmd


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

//////////////////////////////////////////////////////////////////////////////////////////////
//
//   Tx Data buffer : all data block in DWORD(32bits), max block length can >> usb buffer
//                    data belong to: ir, addr, data, ctrl_reg, tms sequence for delay
//   Tx data block need regist: block type: TAP_WR(return data will be discarded)
//                                          TAP_RD(return data need be recovered to RX buffer
//                              block length: ir_len, data_len, addr_len, tms_len
//
//   Output buffer: USB in/out buffer length, when Tx data buffer flushing, data flow will be packed into 
//                 multiple output packages with each equal to buffer length. 
//                 each packages is an EMU_CMD_HW_JTAGx cmd.
//                  
//   Input buffer: return data from TAP when EMU_CMD_HW_JTAGx cmd send to TAP.
//                 the TAP_RD type data will be recover to Rx data buffer
//
//   Rx Data buffer: store the return data
//      -          -          -         -          -          -          -         -
//   Tx Data buffer : [B1]     [B2]        [B3]      [B4]      [B5]        [B6]
//   blk type:        [WR:IR]  [WR:tms_dl] [WR:add]  [RD:data] [WR:IR]     [RD:CtrReg]
//   blk bit len:     [7]      [20]        [32]      [32]      [7]         [32]
//
//                      Flushing 
//   Output buffer:    [JTGx CMD][Num Bits][TMS bit flow][TDI bit flow]
//                      usb_bulk_write->
//                      ->usb_bulk_read
//   Input buffer:     [TDO bit flow]
//
//                        Recovering
//   Rx Data  buffer:  [RB4]  [RB6]
//
//////////////////////////////////////////////////////////////////////////////////////////// 



// add 1 bit to output buffer
static int JL_scan_oneclk (int tms, int tdi)
{
    int byte, bit;

    byte = (gFlushingBitlen >> 3);   //byte offset
    bit  = gFlushingBitlen & 7;      //bit offset
    
    if (bit == 0)
        {
          pTMSbuffer[byte] = 0;
          pTDIbuffer[byte] = 0;
        }

    if (tms)
    	
        pTMSbuffer[byte] |= (1 << bit);
        
    if (tdi)
    	
        pTDIbuffer[byte] |= (1 << bit);

    gFlushingBitlen++;
    
    if(DEBUGBLK) 
    	DBG(("---->Add 1x clk to %d @ [%d.%d],(tms, tdi)= 0x%02X,0x%02X\n",gFlushingBitlen-1,
    	                   byte,bit,pTMSbuffer[byte],pTDIbuffer[byte]));
    
    if ((gFlushingBitlen >> 3) < LL5) 
    	 return 0;
    else
    	 return 1;  //output buffer full
}

// add 1 block data to output buffer
static int JL_scan_blk (int BlkIndex )
{
    int blktype, blkbitlen, tmsplen, tmsplen2, padlen=0;
    int ii,flag;
    DWORD data, *p;
    BYTE tmspath, tmspath2;
    int btms, btdi;
    int st_buf = 0;

    	 blktype    = TxDataBlkType[gTxFlushingIndex]; 
    	 blkbitlen  = TxDataBlkBits[gTxFlushingIndex];
    	 data       = TxDataBuffer [gTxFlushingIndex];

   if (DEBUGBLK)
   	 {
   	  DBG(("->Add (T,L,V) to output buffer, 0x%02X,%d,0x%08X\n",blktype,blkbitlen,data));
   	  DBG(("  cur JTAG ST = %d\n",(int)CurJtagST));
   	  DBG(("  cur gFlushingBitlen = %d\n", gFlushingBitlen));
   	 }
   	   	 
      switch( blktype & 0xF )
      {
        case TAP_TMS_DELAY:
        	 if (blkbitlen<=0) return 0;
        	 if (CurJtagST != JTAG_RTI) //first move to RTI
        	 	{
        	 	 tmspath =        JTAGST_FromTo[CurJtagST][JTAG_RTI];
             tmsplen = TMSCLK_JTAGST_FromTo[CurJtagST][JTAG_RTI];
            CurJtagST = JTAG_RTI;
        	 	 for (ii = 0; ii < tmsplen; ii++)
              {
             	btms = tmspath & (1<<ii);
              JL_scan_oneclk( btms, 0 );
              }     	
        	 	}
           if (blkbitlen>48) blkbitlen = 48;  //4us in 12MHz
           for (ii = 0; ii < blkbitlen; ii++)
            st_buf |= JL_scan_oneclk( 0, 0 );  //wait in RTI
            gTxFlushingIndex++;
    if (DEBUGBLK)
   	  {
   	     DBG(("  post JTAG ST2 = %d\n",(int)CurJtagST));
   	     DBG(("  post gFlushingBitlen = %d\n", gFlushingBitlen));
   	  }         
            return st_buf;
            break;
            
        case TAP_TMS_MOVE:
        	 NxtJtagST = (JtagStates)data;
        	 tmspath   =        JTAGST_FromTo[CurJtagST][NxtJtagST];
        	 tmsplen   = TMSCLK_JTAGST_FromTo[CurJtagST][NxtJtagST];
        	 	for (ii = 0; ii < tmsplen; ii++)
              {
             	 btms = tmspath & (1<<ii);
               st_buf |= JL_scan_oneclk( btms, 0 );
              }   
            CurJtagST = NxtJtagST;
            gTxFlushingIndex++;
   if (DEBUGBLK)
   	  {
   	     DBG(("  post JTAG ST2 = %d\n",(int)CurJtagST));
   	     DBG(("  post gFlushingBitlen = %d\n", gFlushingBitlen));
   	  }        
            return st_buf;
            break;    
            
        case TAP_IR:
            tmspath =        JTAGST_FromTo[CurJtagST][JTAG_SIR];
            tmsplen = TMSCLK_JTAGST_FromTo[CurJtagST][JTAG_SIR];
            tmspath2 = JSTPATH_EXR_TO_RTI;
            tmsplen2 = JSTPLEN_EXR_TO_RTI;
            if(blkbitlen == 32 ) flag =1;
            	else flag = 0;
            break;      

        case TAP_DATA:
 				default:       // 32bits
    
            tmspath =        JTAGST_FromTo[CurJtagST][JTAG_SDR];
            tmsplen = TMSCLK_JTAGST_FromTo[CurJtagST][JTAG_SDR];
            tmspath2 = JSTPATH_EXR_TO_RTI;
            tmsplen2 = JSTPLEN_EXR_TO_RTI; 
            flag = 1;  //align to a new byte
           
       }  //switch()
      
           if (flag)
           	{ //32bits, add padding bits
              // RD data need start from a new byte for easy read
            	 padlen = 8 - ((gFlushingBitlen + tmsplen) & 7);   // padding length
      	       if (padlen < 8)
      	     	 {
    	  	       for(ii=0; ii<padlen; ii++)
    	  	       JL_scan_oneclk( (CurJtagST == JTAG_TLR), 0 );  // RTI or TLR
    	 	       }
    	 	    } 

   if (DEBUGBLK)
   	 {
   	 	if(flag)DBG((" padding with len %1d\n",padlen));
   	  DBG((" path1,len1 0x%02X,%1d\n",tmspath,tmsplen));
   	  DBG((" path2,len2 0x%02X,%1d\n",tmspath2,tmsplen2));
   	 }

           for (ii = 0; ii < tmsplen; ii++)
            {
             	btms = tmspath & (1<<ii);
              JL_scan_oneclk( btms, 0 );  // JTAG ST move
            }     
        
           if (flag)
           	{  
     		      // add padding align to a new byte for easy access
            	if(blktype & TAP_RD) 
           			 InputBlkReadLoc [gInputBufferIndex++] = (gFlushingBitlen >> 3); //save data offset in buffer
            	 p = (DWORD*)(pTDIbuffer + (gFlushingBitlen >> 3));	
           	  *p = data;
           	   p = (DWORD*)(pTMSbuffer + (gFlushingBitlen >> 3));	
           	  *p = (1<<31);    //ExR
           	   gFlushingBitlen += 32;
           	}
           	else
           	{
           	   if (blkbitlen >32) blkbitlen = 32;  
           	   for (ii = 0; ii < blkbitlen; ii++)
                {
             	    btdi = data & (1<<ii);
                  JL_scan_oneclk( ii == (blkbitlen-1), btdi);  // shift data out
                }     
           	}   	 
    	   	 
            for (ii = 0; ii < tmsplen2; ii++)
              {
              	btms = tmspath2 & (1<<ii);
                st_buf = JL_scan_oneclk( btms, 0 );  // JTAG ST move to RTI
              }      	 

            CurJtagST = JTAG_RTI;
            gTxFlushingIndex++;     // go next block
            
    if (DEBUGBLK)
   	  {
   	     DBG(("  post JTAG ST2 = %d\n",(int)CurJtagST));
   	     DBG(("  post gFlushingBitlen = %d\n", gFlushingBitlen));
   	  }
            return st_buf;          // return 1 if output buffer full
}

// perform an USB transaction with TAP
static int JL_scan_flush (void)
{      
	     DWORD bytelen, *p;
	     int ii, ilen, jlen, klen;
	     int st = 0;
	     
       bytelen = (gFlushingBitlen + 7) >> 3;     //total bytes
	     OutputBuffer[0] = (JTAGRW_VER == 2)?EMU_CMD_HW_JTAG2:EMU_CMD_HW_JTAG3;
	     OutputBuffer[1] = 0;
	     OutputBuffer[2] = gFlushingBitlen & 0xFF;
	     OutputBuffer[3] = (gFlushingBitlen >> 8) & 0xFF;
    
       memcpy((void *)(OutputBuffer+4+bytelen), (void *)pTDIbuffer, bytelen);   //copy TDI to output buffer
       
       if(DEBUGOTBUF)
   	   {
   	     if(DEBUGTXBUF) DBG((" -->Flushing total bit %d\n",gFlushingBitlen));
   	     for(ii =0; ii<bytelen*2+4; ii++)
   	      DBG(("====> OutputBuffer[%d] = 0x%02X \n",ii,OutputBuffer[ii] ));
       } 
       
       ilen = libusb_bulk_write(OutputBuffer, bytelen*2+4);
        if (ilen != (bytelen*2+4)) st = 1;
//        ussleep(10000);
        ussleep(bytelen*8000/LL1);
        	
       jlen = libusb_bulk_read(InputBuffer, LL5 + 10);
        if (jlen != bytelen) st = 1;
        	
        if (JTAGRW_VER == 3)
        	{
            cmdbuf[0] = 0;
            klen = libusb_bulk_read(cmdbuf, 200);
            if ( (klen != 1) || cmdbuf[0] ) st = 1;
           }
        	
       for (ii = 0 ; ii <gInputBufferIndex; ii++)
         {
         	p = (DWORD *)(InputBuffer + InputBlkReadLoc[ii]);
          RxDataBuffer[gRxDataBufferIndex++] = *p;  //recover RD data
         }
        
//       if(DEBUGINBUF||DEBUGFLUSH)
        if(st)
       	{ printf("====>usb write[%d], usb read[%d],(st=%d)\n",ilen,jlen,st );
       		if (JTAGRW_VER == 3) printf("====>v3 return state[%d],(%d)\n",klen,cmdbuf[0] );
       		printf("\n\nUSB Port R/W error occur! Please reduce buffer size with /L5:1000 and try again.\n");
       		libusb_close();
       		exit(1);
       	}
       if(DEBUGINBUF)
   	   {
   	   	 
   	     for(ii =0; ii<bytelen; ii++)
   	      DBG(("====> InputBuffer[%d] = 0x%02X \n",ii,InputBuffer[ii] ));
   	     for(ii =0; ii<gInputBufferIndex; ii++)
   	      DBG(("====> Read Data [%d] @ %d = %08X\n",ii,InputBlkReadLoc[ii], *((DWORD *)(InputBuffer + InputBlkReadLoc[ii]))));
       } 
       
//       printf("index: 0x%08x, 0x%08x, 0x%08x\n",	gTxFlushingIndex,	gRxDataBufferIndex,	gTxDataBlkIndex);
        
       gFlushingBitlen   = 0;
       gInputBufferIndex = 0;
       return st;  // st = 1 if usb xfer error occur
}
           
// scan all data in Tx data buffer out to TAP, return data saved in Rx data buffer
static int JL_scan_xfer (void)
{
    
    DWORD blktype;
    int blkbitlen;
    int ii;
    int st = 0;
    int bk =0;

    if (gTxDataBlkIndex == 0) return 0;
    	
    	 gTxFlushingIndex = 0;
    	 gRxDataBufferIndex = 0;
    	 gFlushingBitlen = 0;

    if(DEBUGTXBUF)
   	{
   		 DBG((" We have total %d blks to flush\n",gTxDataBlkIndex));
   	   for(ii =0; ii<gTxDataBlkIndex; ii++)
   	   DBG(("--> tx data buffer to flush, index [%8d],(T,L,V)= 0x%02X,%2d,0x%08X\n-> ",
              ii, TxDataBlkType[ii], TxDataBlkBits[ii], TxDataBuffer[ii] ));
    } 
    // printf("in flush index: 0x%08x, 0x%08x, 0x%08x\n",	gTxFlushingIndex,	gRxDataBufferIndex,	gTxDataBlkIndex);
    while (	gTxFlushingIndex < gTxDataBlkIndex)
    {
   	if(DEBUGTXBUF) DBG((" In Flushing at location %d, with total bit %d\n",gTxFlushingIndex, gFlushingBitlen));
   		
     if (gFlushingBitlen >= (LL5-1)* 8 )
     	{ 
         if( (gFlushingBitlen>0) && ((((gFlushingBitlen+7) >>3) % 64) == 0) )
         	{
         		 for (ii = 0; ii < 7; ii++) // add 1 byte to prevent read multiple 64bytes.
             {
              JL_scan_oneclk( 1, 0 );   // JTAG ST move to TLR
             }  
              JL_scan_oneclk( 0, 0 );
             CurJtagST = JTAG_RTI;
          }   
    //   printf("flush index: 0x%08x, 0x%08x, 0x%08x\n",	gTxFlushingIndex,	gRxDataBufferIndex,	gTxDataBlkIndex);  
       st |= JL_scan_flush();
       if(DEBUGTXBUF) DBG((" end Flushing ,at location %d\n",gTxFlushingIndex));
      } 
     else
       JL_scan_blk ( gTxFlushingIndex );   //add one data block to output buffer
     }

     if (gFlushingBitlen >0)
     	{ 
         if ( (((gFlushingBitlen+7) >>3) % 64) == 0)
         	{
         		 for (ii = 0; ii < 8; ii++)     // add 1 byte to prevent read multiple 64bytes.
             {
              JL_scan_oneclk( 0x7F & (1<<ii), 0 );  // JTAG ST move to RTI
             }  
             CurJtagST = JTAG_RTI;
          }   
       
       st |= JL_scan_flush();
      if(DEBUGTXBUF) DBG((" final Flushing ,at location %d\n",gTxFlushingIndex));
      } 
     //  printf(" final Flushing\n");
   if(DEBUGRXBUF)
   	{
   	   for(ii =0; ii<gRxDataBufferIndex; ii++)
   	    DBG(("====> RxDataBuffer[%d] = %08X \n",ii, RxDataBuffer[ii] ));
   	    DBG(("\n\n\n"));
    } 
     gTxDataBlkIndex = 0;
     return st;  // st = 1 if error occur
}

// TAP delay in RTI
// limit less than 8*6*32 = 1536 tcks, about max delay 128us under 12MHz
// find problem when long wait in TLR with J-Link, try using RTI
static void JL_Wait(int us)
{
	   int i, tCount;
	  
	   if( us >512) us =512;      //512us max
	   tCount = (us * LL1)/1000;
	   if(tCount<0) tCount = 1;

	   while(tCount > 48)	   //4us per delay block
     {
       JL_AddBlkToTxDataBuffer( 0, TAP_TMS_DELAY, 48, 0);
       tCount -= 48;
     }
     if(tCount >0)
  	   JL_AddBlkToTxDataBuffer( 0, TAP_TMS_DELAY, tCount, 0);
  
}

static void JL_AddJTAGStateMoveTo(JtagStates JtagST2)
{
	JL_AddBlkToTxDataBuffer( (DWORD)JtagST2, TAP_TMS_MOVE, 0, 0);
}

// reset tap to TLR,then move to RTI
static void JL_ResetToRTI (void)
{
    int st;
    WORD *p;
        
    cmdbuf[0] = (JTAGRW_VER == 2)?EMU_CMD_HW_JTAG2:EMU_CMD_HW_JTAG3;
    cmdbuf[1] = 0;
    cmdbuf[2] = 16;
    cmdbuf[3] = 0;
    cmdbuf[4] = 0x1F;
    cmdbuf[5] = 0;
    cmdbuf[6] = 0;
    cmdbuf[7] = 0;

    st = libusb_bulk_write(cmdbuf, 8); 
    if (st != 8)
     printf("Jlink TLR reset error (%d)\n",st);

    mssleep(1);
        	
    st = libusb_bulk_read(cmdbuf, 200);
    if (st != 2)
       printf("Jlink TLR reset error (%d)\n",st);
    if (JTAGRW_VER == 3)
     	{
       cmdbuf[0] = 0;
       st = libusb_bulk_read(cmdbuf, 200);
       if( (st!= 1) || cmdbuf[0] )
        	printf("Jlink TLR reset error (%d)\n",st);
      }   
   CurJtagST = JTAG_RTI; 
    
}

/////////////////////////////////////////////////////////////////////////////////////
//
// Queue/Bulk R/W
// Add blk to Tx Data buffer -> scan xfer -> scan blk or scan one clk -> scan flush -> post process Rx Data buffer
//
// push a DWORD data block to Tx data buffer
static void JL_AddBlkToTxDataBuffer(DWORD data, int type, int bitlen, int Is_clear)
{
  if (Is_clear) {gTxDataBlkIndex = 0;}

  TxDataBuffer [gTxDataBlkIndex] = data;
  TxDataBlkType[gTxDataBlkIndex] = type;
  TxDataBlkBits[gTxDataBlkIndex] = bitlen;

    if (DEBUGTXBUF) 
  	{   		
      DBG(("-- add to Tx buffer, index [%08d],(T,L,V)= 0x%02X,%2d,0x%08X\n-> ",
              gTxDataBlkIndex, type, bitlen, TxDataBuffer[gTxDataBlkIndex] ));
    }
  
  gTxDataBlkIndex ++;
}

static void JL_simplecmd ( BYTE cmd )
{
    int st;
    
    cmdbuf[0] = cmd;
    st = libusb_bulk_write (cmdbuf, 1);

    if (st != 1)
     printf("Jlink simple cmd %02X error (%d)\n",cmd,st);
    
}

static void JL_setspeed ( WORD khz )
{
    int st;
    WORD *p;
        
    cmdbuf[0] = EMU_CMD_SET_SPEED;
    p = (WORD*)(cmdbuf +1);
    *p = khz;
    st = libusb_bulk_write(cmdbuf, 3); 
    
    if (st != 3)
     printf("Jlink set speed error (%d)\n",st);
    
}

static void JL_seleJtag (void)
{
    int st;
    
    cmdbuf[0] = EMU_CMD_HW_SELECT_IF;
    cmdbuf[1] = 0xFF;
    st = libusb_bulk_write (cmdbuf, 2);
    if (st != 2)
     printf("Jlink select IF error (%d)\n",st);
    libusb_bulk_read (cmdbuf, 200);

    cmdbuf[0] = EMU_CMD_HW_SELECT_IF;
    cmdbuf[1] = 0;
    st = libusb_bulk_write (cmdbuf, 2);
    if (st != 2)
     printf("Jlink select IF error (%d)\n",st);
    libusb_bulk_read (cmdbuf, 200);    

}

// return Vref
static int JL_getstate(void)
{
    int st;
    WORD *p;
        
    JL_simplecmd (EMU_CMD_GET_STATE);
    st = libusb_bulk_read(cmdbuf, 200); 
    if (st != 8)
     printf("Jlink get state error (%d)\n",st);
    return (cmdbuf[0] + (cmdbuf[1]<<8));
}

// return HW version
static DWORD JL_getversion(void)
{
    int st;
    DWORD *p;

    JL_simplecmd (EMU_CMD_GET_HW_VERSION);
    st = libusb_bulk_read(cmdbuf, 200); 
    if (st != 4)
     printf("Jlink get version error (%d)\n",st);
    p = (DWORD*)cmdbuf;
    return (*p);
}

// return HW cap
static DWORD JL_getcaps(void)
{
    int st;
    DWORD *p;
        
    JL_simplecmd (EMU_CMD_GET_CAPS);
    st = libusb_bulk_read(cmdbuf, 200); 
    if (st != 4)
     printf("Jlink get cap error (%d)\n",st);
    p = (DWORD*)cmdbuf;
    return (*p);
}

static DWORD bitmask32(int len)
{
 
  return ((len==32)? -1 : (1 << len) - 1 ) ;
}


static void JL_getconfig(void)
{
// performance fine tune parameters
//L1: clk, khz
//L2: FT2232 USB read latency>>> 2 ~ 255ms, default 2ms
//L3: DMA xfer Polling timeout>>> 0 ~ 48 us, defualt 1 us
//L4: FLASH Write Polling timeout>>> 1 ~ 127 us, defualt 16 us
//L5: USB Buffer size
//L6: Allowed DMA Bulk xfer count>>>
//L7: Block Size on FLASH DWORD Read in x32Bits Mode>>>
//L8: Block Size on FLASH DWORD Write in x16Bits Mode>>>
//L9: Block Size on FLASH DWORD Write in x8Bits Mode>>>
//	printf(" LL1[%d],LL2[%d],LL3[%d],LL4[%d],LL5[%d],LL6[%d],LL7[%d],LL8[%d],LL9[%d]\n",LL1,LL2,LL3,LL4,LL5,LL6,LL7,LL8,LL9);

  if(LL1 == 0) LL1 = JL_SPEED_1M * 3;
  if(LL3==0 || LL3==0xFFFF) LL3 = DMA_POLLING; 
  if(LL4==0xFFFF) LL4 = FLASH_POLLING;
  if(!LL5) LL5 = JL_MAX_BUFFER_SIZE;
	if(LL5 <1000) LL5 = 1000;
	if(LL5 >JL_MAX_BUFFER_SIZE) LL5 = JL_MAX_BUFFER_SIZE;
  if(!LL6) LL6 = DMA_BLOCK_SIZE;  //256  
  if(!LL7) LL7 = 128;   //RD: 32K data buffer
  if(!LL8) LL8 = 16;    //WR16:32K data buffer
  if(LL9 == 1)     //safe mode
  	{ LL1 = 100;
  		LL4 = 128;
  		LL8 = 2;
  		LL3 = 48;
  		LL7 = 16;
  		LL5 = 1000;
  	}
  else if(LL9 == 2)     //risk read mode
  	{ LL1 = 12000;
  		LL3 = 1;
  		LL7 = 128;
  	}  	
  	
  if(!USBID) USBID = 0x13660101;	
//	printf(" LL1[%d],LL2[%d],LL3[%d],LL4[%d],LL5[%d],LL6[%d],LL7[%d],LL8[%d],LL9[%d]\n",LL1,LL2,LL3,LL4,LL5,LL6,LL7,LL8,LL9);
	
}

static void fill_cable_prop(void *p)
{
   cable_prop_type *pcbl = (cable_prop_type*)p;
   
     pcbl->feature = 0;
     pcbl->close = jlclose;
     pcbl->test_reset = jltest_reset;       
     pcbl->det_instr = jldet_instr;
     pcbl->set_instr = jlset_instr;
     pcbl->ReadWriteData = jlReadWriteData;
     pcbl->ReadData = jlReadData;
     pcbl->WriteData = jlWriteData;
     

     pcbl->ejtag_dma_read_x = 0;
     pcbl->ejtag_dma_write_x = 0;

     pcbl->ejtag_pracc_read_x = 0;
     pcbl->ejtag_pracc_write_x = 0;

     pcbl->sflash_blkread = jl_sflash_blkread;
     pcbl->sflash_blkwrite = jl_sflash_blkwrite;
}

////////////////////// iNIT  J-Link device /////////////////////////////////////////////////////
int jlinit(void*p)
{
  int ii=1;
  int vref;
  DWORD cap, tmp;
 
 fill_cable_prop(p);
 JL_getconfig();
 JL_purgebuffer();
 
 OutputBuffer  = (BYTE *) malloc(MAX_NUM_BYTES_USB_BUFFER*2 + 4);
 pTDIbuffer    = (BYTE *) malloc(MAX_NUM_BYTES_USB_BUFFER);
 InputBuffer   = (BYTE *) malloc(MAX_NUM_BYTES_USB_BUFFER);
 RxDataBuffer  = (DWORD*) malloc(MAX_DATA_BUFFER_SIZE*4);
 TxDataBuffer  = (DWORD*) malloc(MAX_DATA_BUFFER_SIZE*4);
 TxDataBlkType  = (int *) malloc(MAX_DATA_BLOCK_SIZE*4);
 TxDataBlkBits  = (int *) malloc(MAX_DATA_BLOCK_SIZE*4);
 InputBlkReadLoc= (int *) malloc(MAX_DMA_BLOCK_SIZE*4);
 
 
 if (  (OutputBuffer == NULL) || (InputBuffer == NULL)
 	   ||(pTDIbuffer == NULL)  || (InputBlkReadLoc == NULL)
 	   ||(RxDataBuffer == NULL)  || (TxDataBuffer == NULL)
 	   ||(TxDataBlkType == NULL)  || (TxDataBlkBits == NULL)  )
    { printf("Allocate Buffer error!!\n"); exit(1); }
    
  pTMSbuffer = OutputBuffer + 4; 

  libusb_open(USBID, JL_EP1 , JL_EP2, 0, NULL,NULL,NULL);  
                                           // ver > = 5. two eps for bulk in & out
                                           // v3 and v4 use same ep1 for in & out
  
  cap = JL_getcaps();

  if (cap & EMU_CAP_GET_HW_VERSION)
  	{
  		tmp = JL_getversion()/1000;
  		printf("Initializing J-Link HW Ver. %d.%d\n",tmp/10,tmp%10);
  		if(tmp/10 < 5) 
  			{
  				JTAGRW_VER = 2;
  		    printf("Warning! J-Link HW version too old, may not work with BRJTAG!!\n");
  		  }
  	}
  if (cap & EMU_CAP_SELECT_IF)
  	{
  		JL_seleJtag();
  	}
  
  JL_setspeed ( (WORD)LL1 );
  printf(" Set I/O speed to %d KHz\n",LL1);
  if((LL1 < 100)||ejtag_speed) lowspeed = 1;

  JL_simplecmd ( EMU_CMD_HW_TRST0  );
  JL_simplecmd ( EMU_CMD_HW_RESET0 );
  JL_simplecmd ( EMU_CMD_HW_TMS0   );
  JL_simplecmd ( EMU_CMD_HW_DATA0  );
  
   vref = JL_getstate();
  printf(" Detected target Vref = %d.%dV\n",vref/1000, vref%1000);
  return 1;
}


void jlclose(void)
{
  libusb_close();
  
  free(InputBlkReadLoc);
	free(TxDataBlkBits);
	free(TxDataBlkType);
	free(TxDataBuffer);
	free(RxDataBuffer);
	free(InputBuffer);
	free(pTDIbuffer);
	free(OutputBuffer);
	
}

////////////////////////////////////////////////////////////////////////////////////////////////
// Follow once scan command is for reuse brjtag traditonl access method.
// Also for probe, init setting and safe mode access. Very Slow !!!
////////////////////////////////////////////////////////////////////////////////////////////////
void jltest_reset(void)
{
	JL_ResetToRTI ();
}

DWORD jlset_instr( DWORD instr)
{
  DWORD* dd;

  JL_AddBlkToTxDataBuffer( instr, TAP_IR, instruction_length,1);
  if (JL_scan_xfer())
   printf("setinstr:flushing tx buffer error!\n");

   if(gRxDataBufferIndex > 0) 
    {dd = RxDataBuffer; 
    	printf(">>>JL set insta = \t\t\t%08X\n\t\t\t%08X\n", *dd, *(dd+1));
  	if (DEBUGMSG) DBG((">>>JL set insta = \t\t\t%08X\n", *dd));
  	return *dd;}
  return 0;
  
}

DWORD jldet_instr(void)
{
  DWORD* dd;

  JL_AddBlkToTxDataBuffer(0xFFFFFFF9, TAP_IR|TAP_RD, 32,1);  //BYPASS
//  JL_AddBlkToTxDataBuffer(0xFFFFFFFF, TAP_DATA|TAP_RD, 32,0);  //BYPASS
  if (JL_scan_xfer())
  	 printf("detinstr:flushing tx buffer error!\n");
    
  if(gRxDataBufferIndex > 0) 
    {dd = RxDataBuffer; 
    	printf(">>>JL det insta = \t\t\t%08X\n\t\t\t%08X\n", *dd, *(dd+1));
  	if (DEBUGMSG) DBG((">>>JL det insta = \t\t\t%08X\n", *dd));
  	return *dd;}
  return 0;
  
}

// RW 32bits
DWORD jlReadWriteData(DWORD data)
{
  DWORD* dd;
  if (DEBUGMSG) DBG((">>>JL Rd/WR  send data \t\t\t%08X\n", data));
  JL_AddBlkToTxDataBuffer(data, TAP_DATA|TAP_RD, 32,1);
  if (JL_scan_xfer())
  	 printf("JL R/W data:flushing tx buffer error!\n");
   
  if(gRxDataBufferIndex > 0) 
    {dd = RxDataBuffer; 
  	if (DEBUGMSG) DBG((">>>JL Rd/Wr get data \t\t\t%08X\n\n", *dd));
  	return *dd;}
  return -1;
}

// RO 32bits
DWORD jlReadData(void)
{
 return jlReadWriteData(0);
}

// WO 32bits
void jlWriteData(DWORD data)
{

  if (DEBUGMSG) DBG((">>>JL Wr send data \t\t\t%08X\n", data));

  JL_AddBlkToTxDataBuffer(data, TAP_DATA, 32,1);
  if (JL_scan_xfer())
  	 printf("JL Wr data:flushing tx buffer error!\n"); 	 
  	 
}


// fetch DWORD from a BYTE buffer addr
static DWORD getDWORD(BYTE * pAddr)
{
// DWORD* x;
// x = (DWORD *) pAddr;
//  return   *x;
  return (*((DWORD *) pAddr));
}

static void JL_purgebuffer()
{

	gTxFlushingIndex  = 0;     // output buffer
	gFlushingBitlen   = 0;     // output buffer bit
	gInputBufferIndex = 0;     // input buffer
	gRxDataBufferIndex  = 0;   // Rx data buffer
	gTxDataBlkIndex     = 0;   // Tx data buffer
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

#define CCT_CT   1
DWORD JL_dma_blkfetch(DWORD addr, DWORD txdata, int mode, int dmadir, int* opdir)
{
  DWORD rxdata;
  static int RWdir = XFER_TX;   //Internal queue op dir
                                //XFER_Tx: add app DMA data to output queue,
                                //XFER_Rx: read answer from RX data queue
  static int BlockIndex = 0;    //how many DMA xfer block add to queue
  static int BlockDirReg[MAX_DMA_BLOCK_SIZE];  //dma xfer direction, max keep 400 DMA xfer block for 64KB buffer
  static int fetchIndex =0;     //data read out index;
  static int gNextLocOnDataRead = 0;
  int haveread=0;               //have read flag
  int Status;
  int i;
  DWORD cct;                    //control register value for DMA xfer
  
  set_LL3 = 0;      // here set_LL3 is alway equal 0, I don't want to polling control register value

  if (DEBUGFETCH)  
  	DBG(("\n\n>>>>> DMA blk fetch start...\n\n"));
//  	printf("\n\n>>>>> DMA blk fetch start...index: %d,%d\n", BlockIndex,fetchIndex);
  	
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
  	
  	
 if (*opdir == XFER_ADD)  // push to buffer op
	{
     if( RWdir == XFER_TX )
     	{	 
      if (DEBUGFETCH) DBG(("add queue dir TX\n\n"));

       //R/W
       //1  set_instr(INSTR_ADDRESS);
       JL_AddBlkToTxDataBuffer(INSTR_ADDRESS, TAP_IR, instruction_length, (BlockIndex == 0)?1:0);
       //2   WriteData(addr);
       JL_AddBlkToTxDataBuffer(addr, TAP_DATA, 32, 0);
    
       if(dmadir == DMA_WR)
       	{
         //X/3.  set_instr(INSTR_DATA);
          JL_AddBlkToTxDataBuffer(INSTR_DATA, TAP_IR, instruction_length, 0);
         //X/4.  WritedData(data);
          JL_AddBlkToTxDataBuffer(txdata, TAP_DATA, 32, 0);
        }

       // Initiate DMA Read & set DSTRT
       //3/5    set_instr(INSTR_CONTROL);
       JL_AddBlkToTxDataBuffer(INSTR_CONTROL, TAP_IR, instruction_length, 0);
       //4/6.  WriteData(DMAACC | DRWN | DMA_WORD | DSTRT | PROBEN | PRACC);
       JL_AddBlkToTxDataBuffer( (DMAACC|((dmadir == DMA_RD)?DRWN:0)|DMASZ(mode)|DSTRT|PROBEN|PRACC), TAP_DATA, 32, 0);
       // Wait for DSTRT to Clear
       //5/7.  flag = ReadWriteData(DMAACC | PROBEN | PRACC)& DSTRT;
       if (set_LL3)
        {
         for(i = 0; i<CCT_CT;i++)
          {
           JL_Wait(LL3);
           JL_AddBlkToTxDataBuffer((DMAACC | PROBEN | PRACC), TAP_DATA|TAP_RD, 32, 0);
          }
        }
        else
      	  {	JL_Wait((dmadir == DMA_WR)?LL3+11:LL3);
      	  	if(lowspeed) JL_Wait(24);   // add 24us latency
      	  	}
    
       if(dmadir == DMA_RD)
       	{
         //6/X.  set_instr(INSTR_DATA);
         JL_AddBlkToTxDataBuffer(INSTR_DATA, TAP_IR, instruction_length, 0);
         //7/X.  data = ReadData();
         JL_AddBlkToTxDataBuffer(0, TAP_DATA|TAP_RD, 32, 0);
         //8/X   set_instr(INSTR_CONTROL);
         JL_AddBlkToTxDataBuffer(INSTR_CONTROL, TAP_IR, instruction_length, 0);
        }
    
        //9/8. (ReadWriteData(PROBEN | PRACC) & DERR)
       if (set_LL3)
         {
          for(i = 0; i<CCT_CT;i++)
          {
           JL_AddBlkToTxDataBuffer((PROBEN | PRACC), TAP_DATA|TAP_RD, 32, 0);
          }
         }
      
        if (DEBUGFETCH)
      	  DBG(("add current block DMA opdir [%d] = %d\n\n",BlockIndex, dmadir));
        
        BlockDirReg[BlockIndex] = dmadir;  //save current dma xfer dir, DMA write or DMA read
        BlockIndex ++;                     //next block index
 	 	  
       //check block reg queue full?  if full, flush buffer
       if(BlockIndex >= LL6)     //block xfer reg queue full, invert to RX
         {RWdir = XFER_RX;                  //next xfer is need fetch data; 
          fetchIndex = 0;                   //xfer block read index clean
          gNextLocOnDataRead = 0;           //read positon clean
          *opdir = XFER_QRY;                //indicate op invert to fetch
          Status = JL_scan_xfer();         // flush out and read data in Rx Data buffer
          if (DEBUGBLK) DBG(("DMA xfer block count = %d\n\n",BlockIndex));
          if (Status)
           { printf("JL DMA block Read flush buffer error %d\n", Status); }
          
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
      }                // finish push xfer block to tx data buffer
     else              // if ( RWdir == XFER_RX )
     {
       *opdir = XFER_QRY;
       return -1;        //return add not success, all input buffer not empty
      }
    
  }  
 else    // fetch data from queue
  {      // (*opdir == XFER_QRY)
     if( RWdir == XFER_TX  )            //force flush the ouput queue, and flip to Rx opdir, fetch first
  	 	{ 
  	 	 if (BlockIndex <=0)
  	 	 	{
  	 	 		*opdir = XFER_ADD;
  	 	 		 return -1;   // no data flush, return
  	 	 	}
  	 	
  	 	 RWdir = XFER_RX; 
       
       Status = JL_scan_xfer();
       if (DEBUGBLK) DBG(("DMA xfer block count = %d\n\n",BlockIndex));
       if (Status)
         { printf("JL DMA block Read flush buffer error %d\n", Status); }
       
       fetchIndex = 0;               //xfer block read index clean
       gNextLocOnDataRead = 0;       //read data positon clean

       if (DEBUGFETCH) 
 	      { 
 	        DBG(("force flush queue, invert to RX\n\n"));
 	      	for (i=0; i< BlockIndex;i++) 
 	        DBG((" dma opdir [%d]= %d\n", i, BlockDirReg[i]));   }
  	 	}   // if( RWdir == XFER_TX  )   force read out; and continue get first readable data
 
    //    Read: ADDR      CT DL (CT2) DATA (CT3)
  	//   Write: ADDR DATA CT DL (CT2)      (CT3)
       
    //  (XFER_RX) start fetch out --------------------------------------------------------
    
         if (DEBUGFETCH)
        	{   DBG(("pop queue RWdir RX\n\n"));
              DBG(("Curent data queue read location = %d\n\n",gNextLocOnDataRead));
              DBG(("pop queue dmadir [%d]= %d\n\n",fetchIndex, dmadir));
          }

       rxdata = 0;    
       haveread = 0; //data not fetch out
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
             if ( (cct & DSTRT) && (!silent_mode) ) {printf("\n>>>DMA NOT start... %08X\n",cct ); }
             	if ( (cct & DSTRT) ) {printf("\n>>>DMA NOT start... %08X\n",cct ); }
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
             if ( (cct & DERR) && (!silent_mode) ) {printf("\n>>>DMA error occur... %08X\n",cct );}
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
    		        if (DEBUGFETCH) DBG(("fetch out.............\n"));
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

int jl_sflash_blkread(DWORD Inaddr, DWORD* pbuff, int len)
{
	DWORD data_lo, data_hi, data_poll;
  DWORD blocksize;
  int xfer_op = XFER_NOOP;  
  int dmadir = DMA_RD;
  int i, ii;
  int errflag=1;
  DWORD data, addr;
  int freebuffer, ilen;
  
//  printf("\n\n\n\n\n\n\n\n\n------------------------> enter ft_flash_blkread, len = %d \n",len);
  
    freebuffer = JL_dma_blkfetch(0, 0, 0, 0, &xfer_op);
    ilen = freebuffer;  // can xfer such many dma block, guarentee buffrr not overflow.
    len = (len > ilen) ? ilen : len;

   //ADD read command to queue
   for(ii = 0; ii < len; ii ++)
   {
   	addr = Inaddr + 4*ii;
   	xfer_op =XFER_ADD;
   	JL_dma_blkfetch(addr, 0, MIPS_WORD, DMA_RD, &xfer_op);
   }
   
   //Fetch data back
   for(ii = 0; ii < len; ii ++)
   {
   	xfer_op =XFER_QRY;
   	data = JL_dma_blkfetch(0, 0, MIPS_WORD, DMA_RD, &xfer_op);
   	if (BigEndian) data = rev_endian(data);
   	pbuff[ii] = data;
   }   

   return len;

}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//FLASH a DWORD WRITE
//
///////////////////////////////////////////////////////////////////////////////////////////////////
#define POLL_CT  1
void jl_sflash_write_word(DWORD addr, DWORD data, int flpg_x8)
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
         JL_sflash_write_x8((addr& (~3)), data, &xfer_op);
         JL_Wait(LL4);;

         JL_sflash_write_x8((addr & (~3))+ 1, data, &xfer_op);
         JL_Wait(LL4);;

         JL_sflash_write_x8((addr & (~3))+ 2, data, &xfer_op);
         JL_Wait(LL4);;

         JL_sflash_write_x8((addr & (~3))+ 3, data, &xfer_op);
         JL_Wait(LL4);;

      } else {
      	
      	JL_sflash_write_x16((addr& (~3)), data, &xfer_op);
         JL_Wait(LL4);;

         JL_sflash_write_x16((addr& (~3))+2, data, &xfer_op);
         JL_Wait(LL4);;
      }  
          
          //polling 
        if ( (cmd_type != CMD_TYPE_AMD) || !bypass )
        {
          for(i =0; i< POLL_CT ;i++)
          {JL_dma_blkfetch(addr, 0, MIPS_WORD, DMA_RD, &xfer_op);
//           JL_Wait(1);       
          }

          //check polling data
          errflag = 1;
          for(i =0; i< POLL_CT ;i++)
          { 
          	xfer_op = XFER_QRY;
          	data_poll = JL_dma_blkfetch(0, 0, MIPS_WORD, DMA_RD, &xfer_op);
          	if (BigEndian) data_poll = rev_endian(data_poll);
          	if(data_poll == data) errflag = 0;
 //         	printf(" fetch data_poll[%d] = %08X\n", i,data_poll);
          }       
          if (errflag) printf(" \ndma write not correctly !!\n");
        }
        else
        {
           xfer_op = XFER_QRY;
           JL_dma_blkfetch(0, 0, MIPS_WORD, DMA_RD, &xfer_op);
         }         
        //  exit(1);

}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//FLASH Block WRITE, write N DWORDs
//
///////////////////////////////////////////////////////////////////////////////////////////////////

int jl_sflash_blkwrite(DWORD Inaddr, DWORD* pbuff, int len, int flpg_x8)
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

    freebuffer = JL_dma_blkfetch(0, 0, 0, 0, &xfer_op);

    ilen = freebuffer / 9;  // can xfer so many dma block, guarentee buffer not overflow.

    if(len == 0) return ilen;        // answer back the caller query result for max supported block size
    len = (len > ilen) ? ilen : len;
 
//   printf("flash bulk write len = %d\n", len);   	
   for(ii = 0; ii < len; ii ++)
   {
     data = pbuff[ii];
     addr = Inaddr + 4*ii;
     xfer_op = XFER_ADD;

// printf("write data at addr[%08X] = %08X\n", addr,data);
  
    if (flpg_x8) 
    	{  
         JL_sflash_write_x8((addr& (~3)), data, &xfer_op);
         JL_Wait(LL4);;

         JL_sflash_write_x8((addr & (~3))+ 1, data, &xfer_op);
         JL_Wait(LL4);;

         JL_sflash_write_x8((addr & (~3))+ 2, data, &xfer_op);
         JL_Wait(LL4);;

         JL_sflash_write_x8((addr & (~3))+ 3, data, &xfer_op);
         JL_Wait(LL4);;

      } else {
      	
      	JL_sflash_write_x16((addr& (~3)), data, &xfer_op);
         JL_Wait(LL4);;

         JL_sflash_write_x16((addr& (~3))+2, data, &xfer_op);
         JL_Wait(LL4);;
      }  

          //polling 
          if ( (cmd_type != CMD_TYPE_AMD) || !bypass )
          {	
            for(i = 0; i < POLL_CT ;i++)
             {JL_dma_blkfetch(addr, 0, MIPS_WORD, DMA_RD, &xfer_op);
//              JL_Wait(1);       
             }
           }
	  }
	   
	    // check polling data
	    if ( (cmd_type != CMD_TYPE_AMD) || !bypass )
          {
	    
            for(ii = 0; ii < len; ii ++)
             {
   	          data = *(pbuff + ii);
               addr = Inaddr + 4*ii;

               errflag = 1;
               for(i =0; i< POLL_CT ;i++)
                { 
          	      xfer_op = XFER_QRY;
          	      data_poll = JL_dma_blkfetch(0, 0, MIPS_WORD, DMA_RD, &xfer_op);
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
          	  JL_dma_blkfetch(0, 0, MIPS_WORD, DMA_RD, &xfer_op);
           	}
//           	exit(1);
     return len ;
}

#define LEMASK16(k)   (0xffff<<(16*(k)))
#define BEMASK16(k)   (0xffff<<(16*(1-(k))))

static void JL_sflash_write_x16(DWORD addr, DWORD data, int *xfer_op)
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
              JL_dma_blkfetch(FLASH_MEMORY_START+(0x555 << 1), 0x00AA00AA, MIPS_HALFWORD, DMA_WR, xfer_op);
              JL_dma_blkfetch(FLASH_MEMORY_START+(0x2AA << 1), 0x00550055, MIPS_HALFWORD, DMA_WR, xfer_op);
            }
          JL_dma_blkfetch(FLASH_MEMORY_START+(0x555 << 1), 0x00A000A0, MIPS_HALFWORD, DMA_WR, xfer_op);
          break;
        case CMD_TYPE_SST:
          JL_dma_blkfetch(FLASH_MEMORY_START+(0x5555 << 1), 0x00AA00AA, MIPS_HALFWORD, DMA_WR, xfer_op);
          JL_dma_blkfetch(FLASH_MEMORY_START+(0x2AAA << 1), 0x00550055, MIPS_HALFWORD, DMA_WR, xfer_op);
          JL_dma_blkfetch(FLASH_MEMORY_START+(0x5555 << 1), 0x00A000A0, MIPS_HALFWORD, DMA_WR, xfer_op);
          break;
        case CMD_TYPE_BCS:
        case CMD_TYPE_SCS:
        default:
          JL_dma_blkfetch(addr, 0x00500050, MIPS_HALFWORD, DMA_WR, xfer_op);
          JL_dma_blkfetch(addr, 0x00400040, MIPS_HALFWORD, DMA_WR, xfer_op);
       }
 
          JL_dma_blkfetch(addr, odata, MIPS_HALFWORD, DMA_WR, xfer_op);
}

#define LEMASK8(k)    (0xff<<(8*(k)))
#define BEMASK8(k)    (0xff<<(8*(3-(k))))

static void JL_sflash_write_x8(DWORD addr, DWORD data, int *xfer_op)
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
 
 
//     printf(" I am in x8, addr, data = %08x, %08x, %08x\n", addr, data,ldata);  	
     if( ldata == 0xff) return; // no need to program  
     	
     
     switch( cmd_type )
       {
        case CMD_TYPE_AMD:
        	if (!bypass)
        		{
              JL_dma_blkfetch(FLASH_MEMORY_START+(0xAAA), 0xAAAAAAAA, MIPS_BYTE, DMA_WR, xfer_op);
              JL_dma_blkfetch(FLASH_MEMORY_START+(0x555), 0x55555555, MIPS_BYTE, DMA_WR, xfer_op);
            }
           JL_dma_blkfetch(FLASH_MEMORY_START+(0xAAA), 0xA0A0A0A0,MIPS_BYTE, DMA_WR, xfer_op);
           break;
        case CMD_TYPE_SST:
        	return;  //SST 39 don't support x8
          break;
        case CMD_TYPE_BCS:
        case CMD_TYPE_SCS:
        default:
        	JL_dma_blkfetch(addr, 0x50505050,MIPS_BYTE, DMA_WR, xfer_op);
          JL_dma_blkfetch(addr, 0x40404040,MIPS_BYTE, DMA_WR, xfer_op);
       }
 
          JL_dma_blkfetch(addr, odata, MIPS_BYTE, DMA_WR, xfer_op);

}





