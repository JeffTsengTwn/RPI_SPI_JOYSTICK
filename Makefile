
obj-m	+= mygamepad-spi.o

KERNEL=/lib/modules/`uname -r`/build

all drivers:
	make -C $(KERNEL) M=$(PWD) modules

clean:
	make -C  $(KERNEL) M=$(PWD) clean