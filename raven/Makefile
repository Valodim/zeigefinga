
## CONTIKI ##

CONTIKI_PROJECT = zeigefinga

CONTIKI = ../lib/contiki-raven

# multipurpose:
# in lufa, this is application name
# in contiki, it's the platform
TARGET       = avr-ravenusb

# enable radio stack (this is why we want contiki!)
RF230BB=1

# disable all of contiki's usb functionality
# requires contiki patch: 24a7ab5b
CONTIKI_NO_USB = 1

# disable cfs (don't need this anyways)
# requires contiki patch: 0a3be5f5
CONTIKI_NO_CFS = 1

CFLAGS       += -DAUTOSTART_ENABLE

# for contiki: marks dependency on lufa thing
PROJECT_LIBRARIES = lib$(TARGET).a

include $(CONTIKI)/Makefile.include

# override some targets that don't play nice with PROJECT_LIBFILES
%.hex: %.$(TARGET)
	$(OBJCOPY) $^ -j .text -j .data -O ihex $@
%.out: %.$(TARGET)


## LUFA ##

LUFA_PATH    = ../lib/lufa/LUFA/
ARCH         = AVR8
F_CPU        = 8000000
F_USB        = $(F_CPU)UL
SRC          = Descriptors.c $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS)
CFLAGS       += -DUSE_LUFA_CONFIG_HEADER -IConfig/
OPTIMIZATION = s

# multipurpose, see above!
TARGET       = zeigefinga

# include lufa sources, build as normal otherwise
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk

# for contiki: include path for LUFA/ stuff
PROJECTDIRS = $(LUFA_PATH)/..
# for contiki: lufa needs stuff from c99
CFLAGS     += -std=c99 -DF_USB=$(F_USB) 
# for contiki: includes lufa-built lib in binary
TARGET_LIBFILES = lib$(TARGET).a


## AVRDUDE ##

AVRDUDE_PROGRAMMER = jtag2
AVRDUDE_PORT       = usb
# so convenient :3
include $(LUFA_PATH)/Build/lufa_avrdude.mk
