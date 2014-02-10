/*
 * created by hugebird @ chinadsl.net 17/9/2010 Brjtag rev1.9m
 *
 *
 * Brjtag application for HID-BRJTAG v1.xx MCU firmware.
 * fw v1.0 only implemented USB EP0
 *
 * Copyright (C) 2010 Hugebird
 *
 * This code is covered by the GPL v2.
 *
 *
 */
 
 

#if ( defined(_MSC_VER)  )
#define WINDOWS_VERSION
#endif        

#define BUSBASP


#include "zjtag.h" 
#include "busbasp.h"    
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


static BYTE cmdbuf[250];
static int gIndex = 0;

static DWORD tmpbuf[100];

static int lowspeed = 0;
static int skip = 0;

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
extern DWORD LL4;
extern DWORD LL5;
extern DWORD LL6;
extern DWORD LL7;
extern DWORD LL8;
extern DWORD LL9;

extern DWORD USBID;


static void u_getconfig(void)
{
// performance fine tune parameters
//L1: clk
//L2: FT2232 USB read latency>>> 2 ~ 255ms, default 2ms
//L3: DMA xfer Polling timeout>>> 0 ~ 48 us, defualt 1 us
//L4: FLASH Write Polling timeout>>> 1 ~ 127 us, defualt 16 us
//L5: USB Buffer size
//L6: Allowed DMA Bulk xfer count>>>
//L7: Block Size on FLASH DWORD Read in x32Bits Mode>>>
//L8: Block Size on FLASH DWORD Write in x16Bits Mode>>>
//L9: default profile>>>
 
  if(LL4==0xFFFF) LL4 = FLASH_POLLING;
  if(LL1>5) LL1 =5;  //limit to 5 cycle delay max
  if(!LL7 || LL7 >16) LL7 = 16;
  if(!LL8 || LL8 >8) LL8 = 8;
  if(LL9 == 1)       //safe mode
  	{ LL1 = 1;
  		LL7 = 16;
  		LL8 = 4;
  		LL4 = 128;
  	}
  else if(LL9 == 2)  //risk read mode
  	{ 
  		LL1 = 0;
  		LL7 = 16;
  		LL8 = 8;
  	}  	
  	
    if(!USBID) USBID = 0x16C005DF;
//  if(!USBID) USBID = 0x04835750;		
//	printf(" LL1[%d],LL2[%d],LL3[%d],LL4[%d],LL5[%d],LL6[%d],LL7[%d],LL8[%d],LL9[%d]\n",LL1,LL2,LL3,LL4,LL5,LL6,LL7,LL8,LL9);
	
}

static void u_set_speed(int cy)
{

 int cnt = 0;  

  cnt = libusb_msg_read(REQ_MCU_SETSPD, cy, 0, cmdbuf, 8);
  
  if(cnt!=1 || cmdbuf[0] )
  	     printf("MCU set speed error!\n");
}

static void u_get_mcuversion(void)
{

 int cnt = 0;  

  cnt = libusb_msg_read(REQ_MCU_HWVER, 0, 0, cmdbuf, 8);
  
//  printf(" get HW ver, reply msg len = %d\n", cnt);
  
  if(cnt == 4 && cmdbuf[0] == 'B' && cmdbuf[1] == 'r')
  	     printf("HID-Brjtag MCU ROM version: %d.%02d on USBASP hardware!\n",cmdbuf[3],cmdbuf[2]);
  	else 
  		   printf("HID-Brjtag MCU ROM version not fetched!!\n");
}

static int u_get_mcustate(void)
{

 int cnt = 0;  

  cnt = libusb_msg_read(REQ_MCU_GetSTT, 0, 0, cmdbuf, 8);
  
  if(cnt==1 ) 
  	{  printf("MCU st = %d\n",cmdbuf[0]); 
  		 return (cmdbuf[0] | 0x80);
  		}
  	else 
  		{
  			printf("MCU st fetch error\n");
  			return 0;
  		}
}

static void fill_cable_prop(void *p)
{
   cable_prop_type *pcbl = (cable_prop_type*)p;
   
     pcbl->feature = 0;
     pcbl->close = uclose;
     pcbl->test_reset = utest_reset;       
     pcbl->det_instr = udet_instr;
     pcbl->set_instr = uset_instr;
     pcbl->ReadWriteData = uReadWriteData;
     pcbl->ReadData = uReadData;
     pcbl->WriteData = uWriteData;
     
     pcbl->feature |= CBL_DMA_RD|CBL_DMA_WR;
     pcbl->ejtag_dma_read_x = uejtag_dma_read_x;
     pcbl->ejtag_dma_write_x = uejtag_dma_write_x;

     pcbl->ejtag_pracc_read_x = 0;
     pcbl->ejtag_pracc_write_x = 0;

     pcbl->sflash_blkread = u_sflash_blkread;
     pcbl->sflash_blkwrite = u_sflash_blkwrite;
}

////////////////////// iNIT  USB device /////////////////////////////////////////////////////
int uinit(void *p)
{
  int ii=1;
  int vref;
  DWORD cap, tmp;
  char vname[]="Brjtag";
  char pname[]="HID-Brjtag";
  int oo;
  
unsigned char buf[255];
int cmd;
int cnt;  
DWORD * ptr;
 
 fill_cable_prop(p);
 
 u_getconfig();
 
  libusb_open(USBID, 0 ,0, 0, vname,NULL,NULL);  

  u_get_mcuversion();
  		   
  u_set_speed(LL1);		   

 return 1;

}


void uclose(void)
{
  libusb_close();
}


void utest_reset(void)
{

 int cnt = 0;  
  cnt = libusb_msg_read(REQ_MCU_RESET, 0, 0, cmdbuf, 8);
  
  if(cnt!=1 || cmdbuf[0] )
  	     printf("MCU reset error!\n");
}




DWORD udet_instr(void)
{

 int cnt = 0;
 DWORD * dd;


  cmdbuf[0] = 0x03; //det ir
  cmdbuf[1] = 32;   //32  
  dd = (DWORD*)(cmdbuf +2);
 *dd = 0xFFFFFFFF;

  cnt = libusb_msg_write(REQ_MCU_CMDSEQ, 0, 0, cmdbuf, 6);
  
  if(cnt !=6)
    {
    printf(" cmd [detir] write to usb  error! len = %d\n", cnt);
    return 0;
    }


  cnt = libusb_msg_read(REQ_MCU_GetDAT, 0, 0, cmdbuf, 4);
  
  if(cnt !=4)
     {printf(" cmd [detir] read from usb  error! len = %d\n", cnt);
      return 0;
     }

    dd = (DWORD*)cmdbuf;
//    printf(" det ir 0x%08X\n",*dd);
  	return *dd;
    
}


DWORD uset_instr( DWORD instr)
{
	
 int cnt = 0;
 DWORD* dd;
 int dlen;

//getmcust();

  cmdbuf[0] = CMD_TAP_SETIR;      //det ir
  cmdbuf[1] = instruction_length; //32  
  dlen = (cmdbuf[1]+7)>>3;
  dd = (DWORD*)(cmdbuf +2);
 *dd = instr;


  cnt = libusb_msg_write(REQ_MCU_CMDSEQ, 0, 0, cmdbuf, 2+dlen);
  
  if(cnt != (2+dlen))
    {
    printf(" cmd [setir] write to usb  error! len = %d\n", cnt);
    }
//   ussleep(200);

  cnt = libusb_msg_read(REQ_MCU_GetDAT, 0, 0, cmdbuf, 4);
  
  if(cnt !=0)
     {printf(" cmd [setir] read from usb  error! len = %d\n", cnt);
     }

    dd = (DWORD*)cmdbuf;
//    printf(" set ir 0x%08X\n",*dd);
  	return *dd;
    
}


//////////////////////////////////////////////////////////////////////////////////////
// RW 32bits
DWORD uReadWriteData(DWORD data)
{
 int cnt = 0;
 DWORD* dd;


  cmdbuf[0] = CMD_TAP_DR32; //det ir
  dd = (DWORD*)(cmdbuf +1);
 *dd = data;


  cnt = libusb_msg_write(REQ_MCU_CMDSEQ, 0, 0, cmdbuf, 5);
  
  if(cnt !=5)
    {
    printf(" cmd [rwdata] write to usb  error! len = %d\n", cnt);
    return 0;
    }
 
//   ussleep(500);

  cnt = libusb_msg_read(REQ_MCU_GetDAT, 0, 0, cmdbuf, 4);
  
  if(cnt !=4)
     {printf(" cmd [rwdata] read from usb  error! len = %d\n", cnt);
      return 0;
     }

    dd = (DWORD*)cmdbuf;
//    printf(" RW data 0x%08X\n",*dd);
  	return *dd;
}

// RO 32bits
DWORD uReadData(void)
{
 return uReadWriteData(0);
}

// WO 32bits
void uWriteData(DWORD data)
{

   uReadWriteData(data);
  	 
}



///////////////////////////////////////////////////////////////////////////

DWORD uejtag_dma_read_x(DWORD addr, int mode)
{
  DWORD data;
  int k;

 int cnt = 0;
 DWORD* dd;

  cmdbuf[0] = CMD_TAP_DMAREAD;
  cmdbuf[1] = (mode&0x03);        //| (instruction_length <<4);
  dd = (DWORD*)(cmdbuf +2);
 *dd = addr;

// for(k =0; k<6;k++) printf("rdbuf [%d] = 0x%02X\n",k, cmdbuf[k]);

  cnt = libusb_msg_write(REQ_MCU_CMDSEQ, 0, 0, cmdbuf, 6);
  
  if(cnt !=6)
    {
    printf(" cmd [dmard] write to usb  error! len = %d\n", cnt);
    return 0xFFFFFFFF;
    }


//   ussleep(2000);
  cnt = libusb_msg_read(REQ_MCU_GetDAT, 0, 0, cmdbuf, 4);
  
  if(cnt !=4)
     {printf(" cmd [dmard] read from usb  error! len = %d\n", cnt);
      return 0xFFFFFFFF;
     }
   data = *(DWORD*)(cmdbuf);

// dd =(DWORD*) (cmdbuf);
// data = *dd;

// printf("RD return data %08x\n", data);


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
  return(data);
}


void uejtag_dma_write_x(DWORD addr, DWORD data, int mode)
{

 int cnt = 0;
 DWORD* dd;

cmdbuf[0] = CMD_TAP_DMAWRITE;
cmdbuf[1] = (mode&0x03);         // | (instruction_length <<4);
dd = (DWORD*)(cmdbuf +2);
*dd = addr;
dd = (DWORD*)(cmdbuf +6);
*dd = data;

  cnt = libusb_msg_write(REQ_MCU_CMDSEQ, 0, 0, cmdbuf, 10);
  
  if(cnt !=10)
    {
    printf(" cmd [dmawr] write to usb  error! len = %d\n", cnt);
    return;
    }

//  mssleep(50);
  cnt = libusb_msg_read(REQ_MCU_GetDAT, 0, 0, cmdbuf, 4);
  
  if(cnt !=0)
     {printf(" cmd [dmawr] read from usb  error! len = %d\n", cnt);
      return;
     }


}


////////////////////////////////////////////////////////////////////

int u_sflash_blkread(DWORD Inaddr, DWORD* pbuff, int len)
{
  int  ii;
  DWORD data;
  int k;
  int cnt = 0;
  DWORD* dd;

  len = (len > 16) ? 16 : len;


cmdbuf[0] = CMD_TAP_DMABLKRD32;
cmdbuf[1] = MIPS_WORD;              // | (instruction_length <<4);
dd = (DWORD*)(cmdbuf +2);
*dd = Inaddr;
cmdbuf[6] = len;

// for(k =0; k<7;k++) printf("rdbuf [%d] = 0x%02X\n",k, cmdbuf[k]);

  cnt = libusb_msg_write(REQ_MCU_CMDSEQ, 0, 0, cmdbuf, 7);
  
  if(cnt !=7)
    {
    printf(" cmd [dmablkrd] write to usb  error! len = %d\n", cnt);
    return 0;
    }


// mssleep(20);

  cnt = libusb_msg_read(REQ_MCU_GetDAT, 0, 0, cmdbuf, 4*len);
  
  if(cnt <4*len)
     {printf(" cmd [dmablkrd] read from usb  error! len = %d\n", cnt);
      return 0;
     }
// for(k =0; k<4;k++) printf("rdbuf [%d] = 0x%02X\n",k, cmdbuf[k]);

   for( ii =0; ii<len; ii++)
   {
 
     data = *(DWORD*)(cmdbuf + 4*ii);
     if (BigEndian) data = rev_endian(data);
   	 pbuff[ii] = data;
//   	 printf(" blkrddata[%d] = %08X\n", ii, data);
   }   

  return len;
}

int u_sflash_blkwrite(DWORD Inaddr, DWORD* pbuff, int len, int flpg_x8)
{
	DWORD data_poll;
  int xfer_op;  
  int ii,k;
  DWORD data, addr;
  DWORD *dd;
  int cnt;
  int  ilen;
  

    ilen = 8;  // can xfer so many dma block, guarentee buffer not overflow.

    if(len == 0) return ilen;        // answer back the caller query result for max supported block size
    if(len >ilen) len = ilen;
 
    gIndex = 0;   //buffer clean
    xfer_op = flpg_x8?MIPS_BYTE:MIPS_HALFWORD;
    ilen = flpg_x8?4:2;

// file cmd head    
    cmdbuf[0] = CMD_TAP_FLSHBLKWR;
    cmdbuf[1] = xfer_op ;                       //| (instruction_length <<4);
    dd = (DWORD*)(cmdbuf +2);
   *dd = FLASH_MEMORY_START;
    cmdbuf[6] = len*ilen;
    dd = (DWORD*)(cmdbuf +7);
    *dd = Inaddr;
    gIndex +=11;

   for(ii = 0; ii < len; ii ++)
   {
     data = pbuff[ii];
     addr = Inaddr + 4*ii;

// printf("write data at addr[%08X] = %08X\n", addr,data);
  
    if (flpg_x8) 
    	{  
         u_buf_write_x8((addr& (~3)), data);
         u_buf_delayus(LL4);

         u_buf_write_x8((addr & (~3))+ 1, data);
         u_buf_delayus(LL4);

         u_buf_write_x8((addr & (~3))+ 2, data);
         u_buf_delayus(LL4);

         u_buf_write_x8((addr & (~3))+ 3, data);
         u_buf_delayus(LL4);

      } else {
      	
      	u_buf_write_x16((addr& (~3)), data);
        u_buf_delayus(LL4);

        u_buf_write_x16((addr& (~3))+2, data);
        u_buf_delayus(LL4);
      }  
 
	  }

    if ( (cmd_type != CMD_TYPE_AMD) || !bypass )
       {
            // fetch polling data
            cmdbuf[gIndex] = CMD_TAP_DMABLKRD32;
            cmdbuf[gIndex+1] = MIPS_WORD;                   // | (instruction_length <<4);
            dd = (DWORD*)(cmdbuf +gIndex +2);
           *dd = Inaddr;
            cmdbuf[gIndex+6] = len;
            gIndex +=7;
          
       }

//      for(k =0; k<gIndex;k++) printf("rdbuf [%d] = 0x%02X\n",k, cmdbuf[k]);
     cnt = libusb_msg_write(REQ_MCU_CMDSEQ, 0, 0, cmdbuf, gIndex);
  
     if(cnt != gIndex)
         {
           printf(" cmd [flshwrite] write to usb  error! len = %d\n", cnt);
           return 0;
             exit(1); 
         }


             // mssleep(200);

       if ( (cmd_type != CMD_TYPE_AMD) || !bypass )
          {   
             cnt = libusb_msg_read(REQ_MCU_GetDAT, 0, 0, (BYTE*)tmpbuf, 4*len);
  
             if(cnt <4*len)
              {printf(" cmd [flshwrite] read from usb  error! len = %d\n", cnt);
               return 0; 
                 exit(1); 
               }
 
	          // check polling data
	              
             for(ii = 0; ii < len; ii ++)
              {
    	          data = *(pbuff + ii);
                data_poll = *(tmpbuf + ii);
                if (BigEndian) data_poll = rev_endian(data_poll);
                if (data_poll != data) printf("\n dma write not correctly !!\n %08X - %08X\n",data,data_poll);
              }

         }
  
//  exit(1);       
  return len ;
}


#define LEMASK16(k)   (0xffff<<(16*(k)))
#define BEMASK16(k)   (0xffff<<(16*(1-(k))))

static void u_buf_write_x16(DWORD addr, DWORD data)
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

     if( ldata == 0xffff) {skip =1;return;} // no need to program
     switch( cmd_type )
       {
        case CMD_TYPE_AMD:
        	if (!bypass)
        		{
              u_buf_write_I(AMD16_I1);
              u_buf_write_I(AMD16_I2);
            }
          u_buf_write_I(AMD16_I3);
          break;
        case CMD_TYPE_SST:
          u_buf_write_I(SST16_I1);
          u_buf_write_I(SST16_I2);
          u_buf_write_I(SST16_I3);
          break;
        case CMD_TYPE_BCS:
        case CMD_TYPE_SCS:
        default:
          u_buf_write_I(INTL16_I1);
          u_buf_write_I(INTL16_I2);
       }
 
          u_buf_write(odata);
}

#define LEMASK8(k)    (0xff<<(8*(k)))
#define BEMASK8(k)    (0xff<<(8*(3-(k))))

static void u_buf_write_x8(DWORD addr, DWORD data)
{
  int k;
  DWORD odata,ldata;
  
  k = addr & 0x3;
  
    ldata = (data >> (8*k))&0xFF;
    odata = ldata | (ldata <<8);
    odata |= (odata<<16);
 
//     printf(" I am in x8, addr, data = %08x, %08x, %08x\n", addr, data,ldata);  	
     if( ldata == 0xff) {skip =1;return;} // no need to program  
  
     switch( cmd_type )
       {
        case CMD_TYPE_AMD:
        	if (!bypass)
        		{
              u_buf_write_I(AMD8_I1);
              u_buf_write_I(AMD8_I2);
            }
           u_buf_write_I(AMD8_I3);
           break;
        case CMD_TYPE_SST:
        	return;  //SST 39 don't support x8
          break;
        case CMD_TYPE_BCS:
        case CMD_TYPE_SCS:
        default:
        	u_buf_write_I(INTL8_I1);
          u_buf_write_I(INTL8_I2);
       }
 
          u_buf_write(odata);

}

static void u_buf_write(DWORD data)
{
	DWORD *dd;
	
  cmdbuf[gIndex +0] = 0;
  dd = (DWORD*)(cmdbuf + gIndex +1);
 *dd = data;	

	gIndex +=5;
	
}	

static void u_buf_write_I(DWORD id)
{
	
  cmdbuf[gIndex+0] = (BYTE)(0x80|id);
	gIndex ++;
	
}	


static void u_buf_delayus(DWORD us)
{
	
  cmdbuf[gIndex +0] = 0xFF;
  if(skip)
  	 cmdbuf[gIndex +1] = 0;
  	else
     cmdbuf[gIndex +1] = (BYTE)(us);
	gIndex +=2;
	skip = 0;
	
}

static void u_bufcmd_delayus(DWORD us)
{
	
  cmdbuf[gIndex +0] = CMD_TAP_DELAY;
  cmdbuf[gIndex +1] = (BYTE)(us);
	gIndex +=2;
	
}	
