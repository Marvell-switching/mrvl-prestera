# -*-makefile-*-
# call from kernel build system

TARGETS = mvDmaDrv mvIntDrv mvMbusDrv
ifeq ("$(TARGET)", "armv8")
TARGETS += mvEthDrv
endif

ifneq ($(KERNELRELEASE),)

#Extract the path of the CPSS project into HOME_DIR variable
ifeq (customer, $(environment_mode))
HOME_DIR := $(CPSS_PATH)/
else
FULL_PATH := $(subst /cpssEnabler/,/ cpssEnabler/,$(M))
HOME_DIR := $(strip $(firstword $(FULL_PATH)))
endif

ccflags-y := -I$(HOME_DIR)common/h
obj-m := $(addsuffix .o,$(TARGETS))
mvDmaDrv-y := dmaDriver.o
mvIntDrv-y := intDriver.o
mvMbusDrv-y := mbusDriver.o mbusResources.o
ifeq ("$(TARGET)", "armv8")
mvEthDrv-y := ethDriver.o
endif

else
#
# CPSS make
HOME_ROMFS	= $(DIST_HOME)/rootfs
MODULES_DIR	= $(HOME_ROMFS)/lib/modules
INIT_SH     = $(HOME_ROMFS)/etc/init.sh
ifneq (ARMARCH7.ARMADA_390, $(CPU).$(ARM_CPU))
XTRA_RULE=-a ! -f /proc/mvPP
endif

all: default

default:
	$(MAKE) -C $(DIST_HOME) M=$(shell pwd) modules
	@echo -e "\tstripping debug information from $(addsuffix .ko,$(TARGETS))"
	$(PREFIX)strip$(POSTFIX) -g $(addsuffix .ko,$(TARGETS))
	@echo -e "\tinstalling modules to $(MODULES_DIR)"
	@unset MAKEFLAGS;KSUBD=$$($(MAKE)  -s -C $(DIST_HOME) kernelversion); \
	grep -q 'modules.*\(kernel_ver\|uname -r\)' $(INIT_SH) || KSUBD=.; \
	mkdir -p $(MODULES_DIR)/$${KSUBD:-.}; \
	cp $(addsuffix .ko,$(TARGETS)) $(MODULES_DIR)/$${KSUBD:-.} ; \
	if [ "$${KSUBD}" = "." -a -f "$(INIT_SH)" ]; then \
		for target in $(TARGETS); do \
			echo "target=$$target"; \
			grep -q $$target $(INIT_SH) || sed -i \
				-e '/date/ i\' \
				-e ' test -f /lib/modules/'$$target'.ko $(XTRA_RULE) &&\' \
				-e '    insmod /lib/modules/'$$target'.ko\' \
				-e '' \
				$(INIT_SH) ; \
		done ; \
	fi

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions modules.order Module.symvers

depend .depend dep:
	$(CC) $(EXTRA_CFLAGS) -M *.c > .depend

ifeq (.depend,$(wildcard .depend))
include .depend
endif

endif
