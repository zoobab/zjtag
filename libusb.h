/* Libusb supporting, Brjtag 1.8f
 * Copyright (c) 2010 hugebird @chinadsl.net
 *
 *
 */



int libusb_bulk_read(BYTE* buf, int bytelen);
int libusb_bulk_write(BYTE* buf, int bytelen);
int libusb_msg_read(int cmd, int value, int index, BYTE* buf, int bytelen);
int libusb_msg_write(int cmd, int value, int index, BYTE* buf, int bytelen);
void libusb_close(void);
void libusb_open(DWORD id, int epin, int epout, int timeout, char* vn, char* pn, char* sn);


