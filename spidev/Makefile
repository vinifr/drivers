#CC=arm-linux-gnueabihf-gcc
#ARCH=arm
# make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
# INSTALL_MOD_PATH=output modules_install	

INCDIR = -I /home/vini/projs/sunxi/kernel/linux-sunxi/include

ifneq ($(KERNELRELEASE),)
obj-m := at24_i2c.o
else
KDIR := /home/vinix/projs/sunxi/kernel/linux-sunxi
PWD := $(shell pwd)

all:
	$(MAKE) -g -C $(KDIR) $(INCDIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
