#PREFIX=arm-unknown-eabi
PREFIX=arm-none-eabi
FLAGS=-mcpu=cortex-m0plus -mthumb --specs=nosys.specs -DCMSIS -DCPU_MKL25Z128VLK4 -DTUSB_CFG_MCU=MCU_KL25X -DTUSB_CFG_OS=TUSB_OS_NONE -DBOARD=BOARD_FRDM_KL25Z  -fno-exceptions -g 
INC=-ICMSIS/MKL25Z4/Include -Icore/include -I../../tinyusb -I../../demos/device/src -I../../boards
CFLAGS=$(FLAGS) $(INC)
CXXFLAGS=$(FLAGS) $(INC) -fno-rtti

CC=$(PREFIX)-gcc
CXX=$(PREFIX)-g++
LD=$(PREFIX)-gcc

OBJS=startup.o main.o 

KINETIS_OBJS=system_MKL25Z4.o 

USBSOURCES= \
../../tinyusb/host/hub.c \
../../tinyusb/host/ehci/ehci.c\
../../tinyusb/host/hcd.c\
../../tinyusb/host/ohci/ohci.c\
../../tinyusb/host/usbh.c\
../../tinyusb/device/usbd.c\
../../tinyusb/device/dcd.c\
../../tinyusb/osal/osal_none.c\
../../tinyusb/hal/hal_kl25x.c\
../../tinyusb/device/dcd_kl25x.c\
../../tinyusb/tusb.c\
../../tinyusb/common/fifo.c\
../../tinyusb/common/tusb_errors.c\
../../tinyusb/class/cdc_rndis_host.c\
../../tinyusb/class/cdc_device.c\
../../tinyusb/class/cdc_host.c\
../../tinyusb/class/hid_device.c\
../../tinyusb/class/msc_device.c\
../../tinyusb/class/msc_host.c\
../../tinyusb/class/hid_host.c\
../../tinyusb/class/custom_class_host.c\
../../demos/device/src/tusb_descriptors.c\
../../demos/device/src/cdc_device_app.c\
../../demos/device/src/msc_device_app.c\
../../demos/device/src/keyboard_device_app.c\
../../demos/device/src/msc_device_ramdisk.c\
../../boards/frdm-kl25z/board_frdm_kl25z.c


USBOBJS = $(USBSOURCES:.c=.o)

LINKER_SCRIPTS=linker.ld

test.elf: $(OBJS) $(KINETIS_OBJS) $(USBOBJS)
	$(LD) $(CFLAGS) -o $@ $^ -T $(LINKER_SCRIPTS) -Xlinker -Map=test.map

dump: test.elf
	#$(PREFIX)-objdump -h $^
	$(PREFIX)-readelf -S $^

debug: test.elf
	$(PREFIX)-gdb -ex "target remote | openocd -p -f board/frdm-kl25z.cfg" -ex "load" $^



clean:
	$(RM) *.o *.elf
	$(RM) kl2xx/*.o
	$(RM) *.map
	$(RM) $(OBJS) $(KINETIS_OBJS) $(USBOBJS)
