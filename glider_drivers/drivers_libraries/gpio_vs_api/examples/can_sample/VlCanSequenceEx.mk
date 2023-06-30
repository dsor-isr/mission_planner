# VersaLogic non-VersaAPI MPEu-C1E Code Makefile - 20200116
#
# This code is given with no guarantees and is intended to be used as an 
# example of how to use native OS software packages to send/receive CAN
# packets using the VersaLogic MPEu-C1E mPCIe module.
#
# This code has been compiled and run using gcc on Ubuntu 18.04.1, 
# kernel 4.4.0-148-generic, with libusb version 1.0.22 and gcc version 5.4.0
# was used.
#
# Usage: 	make clean	=> Removes all dependant .o files.
#		make 		=> Builds versaAPI_sample application.
#
# Build fruit is an example application 'versaAPI_sample'

LIBS=	-lusb-1.0 
INCLUDEPATHS=	-I. -I/usr/include

O_CAN_SEQ_FILES=VlCanSequenceEx.o CanSupport.o
SRC_CAN_SEQ_FILES=VlCanSequenceEx.c CanSupport.c

O_CAN_RCV_FILES=VlCanRcvEx.o CanSupport.o
SRC_CAN_RCV_FILES=VlCanRcvEx.c CanSupport.c

all:	vlCanSequenceEx

vlCanSequenceEx:	$(O_CAN_SEQ_FILES)
	@echo 'Building target: $@'
	gcc -g -o vlCanSequenceTest $(O_CAN_SEQ_FILES) $(INCLUDEPATHS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

clean:	
	rm *.o vlCanSequenceTest

%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -g -Wall -c $(INCLUDEPATHS) -fmessage-length=0 -o "$@" "$<"
	@echo 'Finished building: $<'


