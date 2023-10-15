
obj-m	+= mygamepad-spi.o

KERNEL=/lib/modules/`uname -r`/build

all drivers:
	make -C $(KERNEL) M=$(PWD) modules
	dtc -@ -I dts -O dtb -o mygamepad-spi.dtbo mygamepad-spi.dts

clean:
	make -C  $(KERNEL) M=$(PWD) clean
	rm -rf *.dtbo