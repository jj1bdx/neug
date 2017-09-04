# Makefile for NeuG

# Define project name here
PROJECT = neug

CHOPSTX = ../chopstx
FRAUCHEKY = ../fraucheky

# Define linker script file here
LDSCRIPT= @LDSCRIPT@

CSRC = main.c sha256.c neug.c
CHIP=@CHIP@
USE_SYS = @USE_SYS@
USE_USB = yes
USE_ADC = yes
EMULATION=@EMULATION@

@ENABLE_FRAUCHEKY@include $(FRAUCHEKY)/src.mk

@HEXOUTPUT_MAKE_OPTION@

###################################
CROSS = @CROSS@
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = @MCU@
CWARN = -Wall -Wextra -Wstrict-prototypes
# DEFS: Add  
DEFS  = @USE_SYS3@ @DEF_EMULATION@
OPT   = -O3 -Os -g
LIBS  = @LIBS@

#######################
include $(CHOPSTX)/rules.mk

@ENABLE_FRAUCHEKY@include $(FRAUCHEKY)/build.mk
###################################

board.h:
	@echo Please run configure to have a symbolic link \'board.h\'
	@exit 1

build/sha256.o: OPT = -O3 -g
build/neug.o: OPT = -O3 -g

distclean:: clean
	-rm -f neug.ld config.h board.h Makefile \
	       usb-strings.c.inc usb-vid-pid-ver.c.inc
