###############################################################
# makefile for SN-GCJA5. November 2020 / paulvha
#
# To create a build with only the SN-GCJA5 monitor type: 
#		make
#
# To create a build with the SN-GCJA5 and SDS011 monitor type:
# 		make BUILD=SDS011
#
###############################################################
BUILD := gcja5

# Objects to build
OBJ := gcja5_lib.o gcja5.o
OBJ_SDS := sds011/serial.o sds011/sds011_lib.o sds011/sdsmon.o

# GCC flags
CXXFLAGS := -Wall -Werror -c
CC_DYLOS := -DDYLOS 
CC_SDS := -DSDS011

# set the right flags and objects to include
ifeq ($(BUILD),gcja5)
fresh:

#include SDS011
else ifeq ($(BUILD),SDS011)
CXXFLAGS += $(CC_SDS) 
OBJ += $(OBJ_SDS)
fresh:

#others to add here
endif

# set variables
CC := gcc
DEPS := gcja5_lib.h bcm2835.h 
LIBS := -lbcm2835 -lm

# how to create .o from .c or .cpp files
.c.o: %c $(DEPS)
	$(CC) $(CXXFLAGS) -o $@ $<

.cpp.o: %c $(DEPS)
	$(CC) $(CXXFLAGS) -o $@ $<

.PHONY : clean gcja5 fresh newgcja5 
	
gcja5 : $(OBJ)
	$(CC) -o $@ $^ $(LIBS)

clean :
	rm -f gcja5 sds011/sds011_lib.o sds011/serial.o sds011/sdsmon.o $(OBJ)

# gcja5.o is removed as this is only impacted by including
newgcja5 :
	rm -f gcja5.o
	
# first execute newsps then build sps30
fresh : newgcja5 gcja5


