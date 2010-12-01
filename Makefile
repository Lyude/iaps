UNAME := $(shell uname -r)

obj-m := iaps.o

KDIR  := /lib/modules/$(shell uname -r)/build
PWD	  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) $(INCLUDE) SUBDIRS=$(PWD) modules

install:
	$(MAKE) -C $(KDIR) $(INCLUDE) SUBDIRS=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) $(INCLUDE) SUBDIRS=$(PWD) clean

.PHONY: all install clean

