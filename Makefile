
#CFLAGS = -Wall -O2 -I. -I./lnx
CFLAGS = -Wall -g -I. -I./lnx

#gcc link to *.so & *.a libs
#app running needs *.so.0 libs
#for dynamic make, *.so libs placed in /usr/local/lib
#for static make, *.a libs placed  app-dir/libs
#so need pre-install libusb and ftd2xx libs
#add lib curses for getch()    -lcurses

DL_PATH = -Wl,-rpath /usr/local/lib
L_LIBS = -L/usr/local/lib -lftd2xx -lusb -ldl -lrt
S_LIBS = lnx/libftd2xx.a lnx/libusb.a -ldl -lpthread -lrt

OBJS = zjtag.o busbasp.o ftdixx.o j-link.o libusb.o stmhid.o


all: zjtag zjtag.a

zjtag: $(OBJS)
	gcc $(CFLAGS) -o zjtag $(OBJS) $(L_LIBS) $(DL_PATH) 

zjtag.a: $(OBJS)
	gcc $(CFLAGS) -o zjtag.a $(OBJS) $(S_LIBS) $(DL_PATH) 
clean:
	rm -rf *.o *~ zjtag zjtag.a
