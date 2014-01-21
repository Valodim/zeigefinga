CONTIKI_PROJECT = zeigefinga

## CONTIKI ##

CONTIKI = ../../contiki/

# multipurpose:
# in lufa, this is application name
# in contiki, it's the platform
TARGET       = avr-ravenusb

# enable radio stack (this is why we want contiki!)
RF230BB=1

# enable ipv6
UIP_CONF_IPV6=1

# disable all of contiki's usb functionality
# requires contiki patch: 24a7ab5b
CONTIKI_NO_USB=1

# disable cfs, raven doesn't support it
# requires contiki patch: 1e1adbb6
CONTIKI_NO_CFS=1

include $(CONTIKI)/Makefile.include


## LUFA ##

LUFA_PATH    = ../../lufa/LUFA/
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

# for contiki: marks dependency (doesn't work?)
PROJECT_LIBRARIES = lib$(TARGET).a
# for contiki: includes lufa-built lib in binary
TARGET_LIBFILES = lib$(TARGET).a
# for contiki: include path for LUFA/ stuff
PROJECTDIRS = $(LUFA_PATH)/..
# for contiki: lufa needs stuff from c99
CFLAGS     += -std=c99 -DF_USB=$(F_USB) 

all: $(CONTIKI_PROJECT)


## AVRDUDE ##

AVRDUDE_PROGRAMMER = jtag2
AVRDUDE_PORT       = usb:00B000000D79
# so convenient :3
include $(LUFA_PATH)/Build/lufa_avrdude.mk

