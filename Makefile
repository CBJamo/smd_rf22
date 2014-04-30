DEBUG=1
MAKE=make
KBUILD=/lib/modules/`uname -r`/build

HOPERF_CFLAGS=-I`pwd`/hoperf/include
SMD_RF22_CFLAGS=-I`pwd`/hoperf/include

ifneq ($(DEBUG), )
ifneq ($(DEBUG), 0)
ifneq ($(DEBUG), n)
HOPERF_CFLAGS+= -DDEBUG
SMD_RF22_CFLAGS+= -DDEBUG
endif
endif
endif

all: hoperf/hoperf.ko smd_rf22/smd_rf22.ko

hoperf/hoperf.ko: hoperf/hoperf.c hoperf/include/linux/hoperfdevice.h hoperf/include/linux/if_hoperf.h hoperf/include/uapi/linux/if_hoperf.h
	$(MAKE) -C $(KBUILD) M=`pwd`/hoperf/ CONFIG_HOPERF=m CFLAGS_hoperf.o="$(HOPERF_CFLAGS)"

smd_rf22/smd_rf22.ko: smd_rf22/smd_rf22.c | hoperf/hoperf.ko
	$(MAKE) -C $(KBUILD) M=`pwd`/smd_rf22/ CONFIG_USB_NET_SMD_RF22=m CFLAGS_smd_rf22.o="$(SMD_RF22_CFLAGS)" KBUILD_EXTRA_SYMBOLS=`pwd`/hoperf/Module.symvers

clean: rf22_clean hoperf_clean 32u4_clean

hoperf_clean:
	$(MAKE) -C $(KBUILD) M=`pwd`/hoperf/ clean

rf22_clean:
	$(MAKE) -C $(KBUILD) M=`pwd`/smd_rf22/ clean

32u4_clean:
	$(MAKE) -C `pwd`/32u4 clean

program:
	$(MAKE) -C `pwd`/32u4 program

reload:	hoperf/hoperf.ko smd_rf22/smd_rf22.ko
	@echo "Reloading hoperf and smd_rf22"
	@lsmod | grep -q smd_rf22 && sudo rmmod smd_rf22 || true
	@lsmod | grep -q hoperf && sudo rmmod hoperf || true
	@sudo insmod `pwd`/hoperf/hoperf.ko && \
	sudo insmod `pwd`/smd_rf22/smd_rf22.ko
