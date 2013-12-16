CONTIKI_PROJECT = acc-example

all: $(CONTIKI_PROJECT)
	
TARGET=inga

CONTIKI = ../contiki/
include $(CONTIKI)/Makefile.include
