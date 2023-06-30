# VersaLogic VersaAPI Sample Code Makefile - 20191212
#
# This code is given with no guarantees and is intended to be used as an 
# example of how to use the VersaAPI software package and to check the 
# operation post install. 
#
# This code has been compiled and run using gcc on Ubuntu 14.04.1, 
# kernel 4.4.0-148-generic, with VersaAPI version 1.5.3can. gcc version 5.4.0
# was used.
#
# Usage: 	make -f rx_tx.mk clean	=> Removes all dependant .o files.
# 		make -f rx_tx.mk all	=> Builds versaAPI_sample application.
#
# Build fruit is an example application 'rx_tx'

BUILD_FLAGS=	-Dusb

LIBS=	/usr/local/lib/libVL_CAN.so.1 /usr/local/lib/libVL_OSALib.so -lcgos -lusb-1.0 
INCLUDEPATHS=	-I. -I/usr/include

O_FILES=	rx_tx.o
SRC_FILES=	rx_tx.c

all:		rx_tx

rx_tx:	$(O_FILES)
	@echo 'Building target: $@'
	gcc -o rx_tx $(BUILD_FLAGS) $(O_FILES) $(INCLUDEPATHS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

clean:	
	rm *.o rx_tx

%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -g -Wall -c $(BUILD_FLAGS) $(INCLUDEPATHS) -fmessage-length=0 -o "$@" "$<"
	@echo 'Finished building: $<'


