# Makefile for NeuG

# Define project name here
PROJECT = neug

CHOPSTX = ../chopstx
FRAUCHEKY = ../fraucheky

CSRC = main.c sha256.c neug.c

include config.mk

USE_USB = yes
USE_ADC = yes

ifneq ($(ENABLE_FRAUCHEKY),)
include $(FRAUCHEKY)/src.mk
endif

###################################
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

CWARN = -Wall -Wextra -Wstrict-prototypes
OPT   = -O3 -Os -g

#######################
include $(CHOPSTX)/rules.mk

ifneq ($(ENABLE_FRAUCHEKY),)
include $(FRAUCHEKY)/build.mk
endif
###################################

board.h:
	@echo Please run configure to have a symbolic link \'board.h\'
	@exit 1

build/sha256.o: OPT = -O3 -g
build/neug.o: OPT = -O3 -g

distclean:: clean
	-rm -f neug.ld config.h board.h config.mk \
	       usb-strings.c.inc usb-vid-pid-ver.c.inc
