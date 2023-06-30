# VersaLogic VersaAPI Sample Code Makefile - 20160225
#
# This code is given with no guarantees and is intended to be used as an 
# example of how to use the VersaAPI software package and to check the 
# operation post install. 
#
# This code has been compiled and run using gcc on Ubuntu 14.04.1, 
# kernel 3.19.0-51-generic, with VersaAPI version 1.4.2. gcc version 4.8.4
# was used.
#
# Usage: 	make clean	=> Removes all dependant .o files.
#		make 		=> Builds versaAPI_sample application.
#
# Build fruit is an example application 'versaAPI_sample'

BUILD_FLAGS=	-Dusb

LIBS=	/usr/local/lib/libVL_CAN.so.1 /usr/local/lib/libVL_OSALib.so -lcgos -lusb-1.0 
INCLUDEPATHS=	-I. -I/usr/include

O_CAN_RCV_FILES=canAPI_rcv-only.o
SRC_RCV_FILES=	canAPI_rcv-only.c

all:	can_rcv

can_rcv:	$(O_CAN_RCV_FILES)
	@echo 'Building target: $@'
	gcc -g -o can_rcv $(O_CAN_RCV_FILES) $(BUILD_FLAGS) $(INCLUDEPATHS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

clean:	
	rm *.o can_rcv

%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -g -Wall -c $(BUILD_FLAGS) $(INCLUDEPATHS) -fmessage-length=0 -o "$@" "$<"
	@echo 'Finished building: $<'


