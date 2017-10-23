# Makefile for NeuG

# Define project name here
PROJECT = neug

CHOPSTX = ../chopstx
FRAUCHEKY = ../fraucheky

CSRC = main.c sha256.c neug.c

include config.mk

USE_SYS = yes
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
ifeq ($(EMULATION),)
BFDNAME_OBJ = elf32-little
else
# For now, it assumes x86_64
BFDNAME_OBJ = elf64-x86-64
BFDARCH = i386
endif
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

ifneq ($(EMULATION),)
# By specifying DESTDIR on invocation of "make", you can install
# program to different ROOT.

# The variables prefix, exec_prefix, libexecdir are defined in
# config.mk.

install: build/neug
	test -d "$(DESTDIR)$(libexecdir)" || mkdir -p "$(DESTDIR)$(libexecdir)"
	install -c build/neug "$(DESTDIR)$(libexecdir)"

endif
