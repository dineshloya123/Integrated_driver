obj-m +=integrated_Driver.o

all: module dt

module:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
dt:
	dtc -@ -I dts -O dtb -o integrated_driver.dtbo integrated_driver.dts
	
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -rf integrated_driver.dtbo
