ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
obj-m  := cgos-core.o cgos-wdt.o gpio-cgos.o i2c-cgos.o
cgos-wdt-y := cgos_wdt.o
else
# normal makefile
KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

endif
