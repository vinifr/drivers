
# Compilacao: make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

# Caminho para o codigo do kernel
KNERL_PATH = /home/vini/projs/texas/kernel/linux

INCDIR = -I ${KNERL_PATH}/include

ifneq ($(KERNELRELEASE),)
obj-m := pmic-consumer.o
else
KDIR := ${KNERL_PATH}
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) $(INCDIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
