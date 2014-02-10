/* Libusb supporting, Brjtag 1.9m
 * Copyright (c) 2010 hugebird @chinadsl.net
 *
 *
 */



#if ( defined(_MSC_VER)  )
#define WINDOWS_VERSION             //windows make with: cl brjtag.c xxx.c
#endif                    

#define BRLIBUSB

#ifdef WINDOWS_VERSION

#include <windows.h>      // Only for Windows Compile
#include "usb_w.h"
#define strcasecmp  stricmp
#define strncasecmp strnicmp
#define mssleep(s)   Sleep(s)

#else   //For linux

#include "usb.h"
#define mssleep(s)   usleep((s) *1000)

#ifndef MY_TYPE
#define MY_TYPE
typedef unsigned long     DWORD;
typedef unsigned short    WORD;
typedef unsigned char     BYTE;
typedef int               bool;
#endif   //MY_TYPE

#endif


#include "libusb.h"

#define MATCH_SUCCESS			1
#define MATCH_FAILED			0
#define MATCH_ABORT				-1

static int usbGetStringAscii(usb_dev_handle *dev, int index, char *buf, int buflen);
static int shellStyleMatch(char *text, char *pattern);
static int  _shellStyleMatch(char *text, char *p);
static usb_dev_handle *open_dev(void);

#define USB_TIMEOUT 1000
static usb_dev_handle *handle = NULL; /* the device handle */
static int ep_in = 0x81, ep_out = 0x02;
static WORD my_vid, my_pid;
static int usb_timeout = USB_TIMEOUT;
static char    v_name[256], p_name[256], s_name[256];


#ifdef WINDOWS_VERSION

typedef void (__cdecl *tinit)   (void);
typedef int  (__cdecl *tfindbus)(void);
typedef int  (__cdecl *tfinddev)(void);
typedef struct usb_device* (__cdecl *tdevice)(usb_dev_handle*);
typedef struct usb_bus* (__cdecl *tgetbus)(void);
typedef int (__cdecl *treset)     (usb_dev_handle*);
typedef int (__cdecl *tsetconfig) (usb_dev_handle*, int);
typedef int (__cdecl *tclaimintf) (usb_dev_handle*, int);
typedef int (__cdecl *tsetaltintf)(usb_dev_handle *, int);
typedef usb_dev_handle* (__cdecl *topen)(struct usb_device*);
typedef int (__cdecl *treleaseintf)(usb_dev_handle*, int);
typedef int (__cdecl *tclose)      (usb_dev_handle*);
typedef int (__cdecl *tbulkwrite)  (usb_dev_handle*, int, char*,int, int);
typedef int (__cdecl *tbulkread)   (usb_dev_handle*, int, char*,int, int);
typedef int (__cdecl *tgetstrs)   (usb_dev_handle*, int, char*,size_t);
typedef int (__cdecl *tctrlmsg)   (usb_dev_handle*, int, int, int, int, char*,int, int);


static tinit        _usb_init;
static tfindbus     _usb_find_busses;
static tfinddev     _usb_find_devices;
static tdevice      _usb_device;
static tgetbus      _usb_get_busses;
static treset       _usb_reset;
static tsetconfig   _usb_set_configuration;
static tclaimintf   _usb_claim_interface;
static tsetaltintf  _usb_set_altinterface;
static topen        _usb_open = NULL;
static treleaseintf _usb_release_interface;
static tclose       _usb_close;
static tbulkwrite   _usb_bulk_write;
static tbulkread    _usb_bulk_read;
static tgetstrs     _usb_get_string_simple;
static tctrlmsg     _usb_control_msg;


HINSTANCE libusb_dll;
static void load_libusbdll(void);

#else

#define _usb_init usb_init
#define _usb_find_busses usb_find_busses
#define _usb_find_devices usb_find_devices
#define _usb_device usb_device
#define _usb_get_busses usb_get_busses
#define _usb_reset usb_reset
#define _usb_set_configuration usb_set_configuration
#define _usb_claim_interface usb_claim_interface
#define _usb_set_altinterface usb_set_altinterface
#define _usb_open usb_open
#define _usb_release_interface usb_release_interface
#define _usb_close usb_close
#define _usb_bulk_write usb_bulk_write
#define _usb_bulk_read usb_bulk_read
#define _usb_get_string_simple usb_get_string_simple
#define _usb_control_msg usb_control_msg

#endif



#ifdef WINDOWS_VERSION
static void load_libusbdll(void)
{

     libusb_dll = LoadLibrary("libusb0.dll");
     if (!libusb_dll)
       {
        printf("Couldn't load libusb liberary\n");
        exit (1);
      }

   _usb_init = (tinit) GetProcAddress(libusb_dll, "usb_init");
   _usb_find_busses = (tfindbus)  GetProcAddress(libusb_dll, "usb_find_busses");
   _usb_find_devices = (tfinddev) GetProcAddress(libusb_dll, "usb_find_devices");
   _usb_device = (tdevice) GetProcAddress(libusb_dll, "usb_device");   
   _usb_get_busses = (tgetbus) GetProcAddress(libusb_dll, "usb_get_busses");  
   _usb_reset = (treset) GetProcAddress(libusb_dll, "usb_reset");
   _usb_set_configuration = (tsetconfig) GetProcAddress(libusb_dll, "usb_set_configuration");       
   _usb_claim_interface = (tclaimintf) GetProcAddress(libusb_dll, "usb_claim_interface");   
   _usb_set_altinterface = (tsetaltintf) GetProcAddress(libusb_dll, "usb_set_altinterface"); 
     
   _usb_open = (topen)GetProcAddress(libusb_dll, "usb_open"); 
   _usb_release_interface = (treleaseintf) GetProcAddress(libusb_dll, "usb_release_interface");
   _usb_close = (tclose) GetProcAddress(libusb_dll, "usb_close");
   _usb_bulk_write = (tbulkwrite) GetProcAddress(libusb_dll, "usb_bulk_write");
   _usb_bulk_read = (tbulkread)   GetProcAddress(libusb_dll, "usb_bulk_read");

   _usb_get_string_simple = (tgetstrs) GetProcAddress(libusb_dll, "usb_get_string_simple");
   _usb_control_msg = (tctrlmsg)   GetProcAddress(libusb_dll, "usb_control_msg");


 if ( !_usb_init || !_usb_find_busses || !_usb_find_devices || !_usb_device ||
 	    !_usb_get_busses || !_usb_reset || !_usb_set_configuration || !_usb_claim_interface ||
 	    !_usb_set_altinterface || !_usb_open || !_usb_release_interface || !_usb_close ||
 	    !_usb_bulk_write || !_usb_bulk_read || !_usb_get_string_simple   || !_usb_control_msg )

       {
        printf("Couldn't link to libusb liberary\n");
        exit (1);
      }
}

#endif


static usb_dev_handle *open_dev(void)
{
    struct usb_bus *bus;
    struct usb_device *dev;
    for (bus = _usb_get_busses(); bus; bus = bus->next)
    { 
        for (dev = bus->devices; dev; dev = dev->next)
        {
          if ( (my_vid == 0 || dev->descriptor.idVendor == my_vid)
                    && (my_pid==0 || dev->descriptor.idProduct == my_pid) )
            {
                return _usb_open(dev);;
            }
        }
    }
    return NULL;
}


void libusb_open(DWORD id, int epin, int epout, int timeout, char *vendorName, char *productName, char *serialNO)
{
  
  struct usb_bus    *bus;
  struct usb_device *dev;
  int len;
  int err;

  if(epin) ep_in = epin;
  if(epout) ep_out = epout;
  my_vid = (id>>16)&0xFFFF;
  my_pid = id & 0xFFFF;
  if(timeout) usb_timeout = timeout;

#ifdef WINDOWS_VERSION
    load_libusbdll();
#endif

    _usb_init(); /* initialize the library */
    _usb_find_busses(); /* find all busses */
    _usb_find_devices(); /* find all connected devices */


    for (bus = _usb_get_busses(); bus; bus = bus->next)
    { 
        for (dev = bus->devices; dev; dev = dev->next)
        {
          err = 0;
          if (  dev->descriptor.idVendor  == my_vid
            &&  dev->descriptor.idProduct == my_pid )
            
           {
            handle = _usb_open(dev); // open dev to query strings
            if(!handle){ err=1; continue; }
                

           //match Vendor Name String
            len = 0;
            v_name[0] = 0;
            
            if(vendorName != NULL && dev->descriptor.iManufacturer > 0)
             {
               len = usbGetStringAscii(handle, dev->descriptor.iManufacturer, v_name, sizeof(v_name));
               if(  (len < 0) || (shellStyleMatch(v_name, vendorName) != MATCH_SUCCESS) ) 
               	  {err = 2; goto target1; }
               	  
             }      
                        
           //match Production Name String
            len = 0;
            p_name[0] = 0;
            
            if(productName != NULL && dev->descriptor.iProduct > 0)
             {
               len = usbGetStringAscii(handle, dev->descriptor.iProduct, p_name, sizeof(p_name));
               if(  (len < 0) || (shellStyleMatch(p_name, productName) != MATCH_SUCCESS) ) 
               	  {err = 3; goto target1; }
              }
                   
           //match Serial Number String
            len = 0;
            s_name[0] = 0;
            
            if(serialNO != NULL && dev->descriptor.iSerialNumber > 0)
             {
               len = usbGetStringAscii(handle, dev->descriptor.iSerialNumber, s_name, sizeof(s_name));
               if(  (len < 0) || (shellStyleMatch(s_name, serialNO) != MATCH_SUCCESS) ) 
               	  {err = 4;  goto target1; }
              }
     
target1:    
             if (err)  
           	  {  
           		  _usb_close(handle);
                handle = NULL;
              }
             break;
            }

        }
        if(handle) break;        
    }
  if(!handle)
  	{
  		if(err ==0) printf("libusb error: open find device %04X:%04X !\n",my_vid,my_pid);
  		if(err ==1) printf("libusb error: open device %04X:%04X error!\n",my_vid,my_pid);
  		if(err ==2) printf("libusb error: NOT find matching device with vendor name [%s]\n",vendorName);
  		if(err ==3) printf("libusb error: NOT find matching device with production name [%s]\n",productName);
  		if(err ==4) printf("libusb error: NOT find matching device with serial no [%s]\n",serialNO);
      exit(1);
    }

   if(handle)
  	{

      usbGetStringAscii(handle, dev->descriptor.iManufacturer, v_name, sizeof(v_name));
      usbGetStringAscii(handle, dev->descriptor.iProduct, p_name, sizeof(p_name));
      usbGetStringAscii(handle, dev->descriptor.iSerialNumber, s_name, sizeof(s_name));
      printf("  Open USB device: 0x%04X:%04X\n",my_vid,my_pid);
      printf("  Vendor Name: [%s]\n",v_name);
      printf("  Produc Name: [%s]\n",p_name);
      printf("  Serial No  : [%s]\n\n",s_name);
    }


/*
 // reset found device

    _usb_reset(handle);
    handle = NULL;

    timeout = 10;
    while (handle == NULL && timeout)
    {
    mssleep(100);
    _usb_find_devices(); // find all connected devices
    
    handle = open_dev();
    timeout -- ;
    }

    if (!handle)
    {
      printf("libusb error: USB TAP device reset error!\n");
      exit(1);
    }

*/

    
	  dev = _usb_device(handle);    
    if (_usb_set_configuration(handle, dev->config[0].bConfigurationValue) < 0)
    {
        printf("libusb error: setting config failed\n");
        _usb_close(handle);
        exit(1);
    }

    if (_usb_claim_interface(handle, 0) < 0)
    {
        printf("libusb error: claiming interface failed\n");
        _usb_close(handle);
       exit(1);
    }
#if 0    
    else
		{
			_usb_set_altinterface(handle, 0);
		}
#endif
}


void libusb_close(void)
{
	  _usb_release_interface(handle, 0);
    _usb_close(handle);

#ifdef WINDOWS_VERSION
	if (libusb_dll != NULL)
         FreeLibrary (libusb_dll);
#endif
}


int libusb_bulk_read(BYTE* buf, int bytelen)
{
	return _usb_bulk_read(handle, ep_in, (char*)buf, bytelen, usb_timeout);
}

int libusb_bulk_write(BYTE* buf, int bytelen)
{
	 return _usb_bulk_write(handle, ep_out, (char*)buf, bytelen, usb_timeout);
}


/*
    int usb_control_msg(usb_dev_handle *dev, int requesttype, int request,
                        int value, int index, char *bytes, int size,
                        int timeout);


typedef union usbWord{
    unsigned    word;
    uint8_t       bytes[2];
}usbWord_t;

typedef struct usbRequest{
    uint8_t       bmRequestType;       // data[0]
    uint8_t       bRequest;            // data[1]
    usbWord_t   wValue;              // data[2],[3]    [3]<<8 | [2]
    usbWord_t   wIndex;              // data[4],[5]
    usbWord_t   wLength;             // data[6],[7]
}usbRequest_t;

*/

// read usb data to buffer, len<255
int libusb_msg_read(int cmd, int value, int index, BYTE* buf, int bytelen)
{
	
	return _usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, cmd, value, index, buf, bytelen, usb_timeout);
	
}

// write buffer to usb, len<255
int libusb_msg_write(int cmd, int value, int index, BYTE* buf, int bytelen)
{

	return _usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT,  cmd, value, index, buf, bytelen, usb_timeout);
	
}



/* private interface: match text and p, return MATCH_SUCCESS, MATCH_FAILED, or MATCH_ABORT. */
static int  _shellStyleMatch(char *text, char *p)
{
int last, matched, reverse;

    for(; *p; text++, p++){
        if(*text == 0 && *p != '*')
            return MATCH_ABORT;
        switch(*p){
        case '\\':
            /* Literal match with following character. */
            p++;
            /* FALLTHROUGH */
        default:
            if(*text != *p)
                return MATCH_FAILED;
            continue;
        case '?':
            /* Match anything. */
            continue;
        case '*':
            while(*++p == '*')
                /* Consecutive stars act just like one. */
                continue;
            if(*p == 0)
                /* Trailing star matches everything. */
                return MATCH_SUCCESS;
            while(*text)
                if((matched = _shellStyleMatch(text++, p)) != MATCH_FAILED)
                    return matched;
            return MATCH_ABORT;
        case '[':
            reverse = p[1] == '^';
            if(reverse) /* Inverted character class. */
                p++;
            matched = MATCH_FAILED;
            if(p[1] == ']' || p[1] == '-')
                if(*++p == *text)
                    matched = MATCH_SUCCESS;
            for(last = *p; *++p && *p != ']'; last = *p)
                if (*p == '-' && p[1] != ']' ? *text <= *++p && *text >= last : *text == *p)
                    matched = MATCH_SUCCESS;
            if(matched == reverse)
                return MATCH_FAILED;
            continue;
        }
    }
    return *text == 0;
}

/* public interface for shell style matching: returns 0 if fails, 1 if matches */
static int shellStyleMatch(char *text, char *pattern)
{
    if(pattern == NULL) /* NULL pattern is synonymous to "*" */
        return MATCH_SUCCESS;
    return _shellStyleMatch(text, pattern);
}

/* ------------------------------------------------------------------------- */

static int usbGetStringAscii(usb_dev_handle *dev, int index, char *buf, int buflen)
{
char    buffer[256];
int     rval, i;

    if((rval = _usb_get_string_simple(dev, index, buf, buflen)) >= 0) /* use libusb version if it works */
        return rval;
    if((rval = _usb_control_msg(dev, USB_ENDPOINT_IN, USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING << 8) + index, 0x0409, buffer, sizeof(buffer), 5000)) < 0)
        return rval;
    if(buffer[1] != USB_DT_STRING){
        *buf = 0;
        return 0;
    }
    if((unsigned char)buffer[0] < rval)
        rval = (unsigned char)buffer[0];
    rval /= 2;
    /* lossy conversion to ISO Latin1: */
    for(i=1;i<rval;i++){
        if(i > buflen)              /* destination buffer overflow */
            break;
        buf[i-1] = buffer[2 * i];
        if(buffer[2 * i + 1] != 0)  /* outside of ISO Latin1 range */
            buf[i-1] = '?';
    }
    buf[i-1] = 0;
    return i-1;
}


/* ------------------------------------------------------------------------- */





