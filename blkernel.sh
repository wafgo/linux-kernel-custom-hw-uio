#! /bin/sh

make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j8 uImage LOADADDR=0x00008000 && \
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zynq-zed-adv7511.dtb && \
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zynq-zed-disp-coproc.dtb && 
#echo "copy uImage to tftp" && sudo cp arch/arm/boot/uImage /srv/tftp && \
#echo "copy device tree to tftp" && sudo cp arch/arm/boot/dts/zynq-zed-adv7511.dtb /srv/tftp &&
#echo "copy device tree to tftp" && sudo cp arch/arm/boot/dts/zynq-zed-disp-coproc.dtb /srv/tftp

