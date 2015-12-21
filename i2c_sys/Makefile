CROSS_COMPILE=arm-linux-gnueabihf-gcc
ARCH=arm
# make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
# INSTALL_MOD_PATH=output modules_install	

INCDIR = -I /home/vini/projs/texas/am3352/linux/include

ifneq ($(KERNELRELEASE),)
obj-m := at24.o
else
KDIR := /home/vini/projs/texas/am3352/linux
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) $(INCDIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
