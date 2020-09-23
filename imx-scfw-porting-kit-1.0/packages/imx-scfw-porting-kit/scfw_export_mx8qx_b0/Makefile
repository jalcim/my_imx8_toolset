# This Makefile will build the System Controller and Test

SHELL = /bin/sh

# Configure Si Rev
DEF_SREV := B0
ifdef r
    R := $(r)
endif
LOWER_R := $(shell echo $(R) | tr A-Z a-z)
UPPER_R := $(shell echo $(R) | tr a-z A-Z)
export R LOWER_R UPPER_R

# Include config makefiles
HELP := ""
HELP_OPT := ""
MAKECONFIG := $(shell find makefiles -name Makefile.config)
MAKECONFIG += $(shell find makefiles -name Makefile.full)
include $(foreach makeconfig,$(MAKECONFIG),$(makeconfig))

# Default config
CONFIG ?= mx8qm
DEVICE ?= ALL
OUT ?= build_api
HW ?= SIMU
PROJ ?= ""
SREC_OFFSET ?= 0x00000000

DOX_OUT = $(OUT)/docs
FLAGS = -DCPU_$(DEVICE) $(OPT) -DECC_CLEAN

# Configure verbose logging

ifdef v
    V := $(v)
endif
ifeq ($(V),1)
    AT :=
else
    AT := @
endif
export AT

# Configure forced error
ifdef f
    F := $(f)
endif
ifndef F
    F = 0
endif
ifeq ($(F),1)
	FLAGS += -DFORCE_COMPILE_ERROR
endif
ifeq ($(F),2)
	FLAGS += -DFORCE_TEST_ERROR
endif

# Configure board
ifdef b
    B := $(b)
endif
ifeq ($(HW), SIMU)
    B := none
endif
ifndef B
    B := val
endif
LOWER_B := $(shell echo $(B) | tr A-Z a-z)
export B
export LOWER_B
ifeq ($(LOWER_B),none)
	BOARD = none
else
	BOARD = $(CONFIG)_$(LOWER_B)
endif
FLAGS += -DBOARD_$(shell echo $(B) | tr a-z A-Z)

# Configure SS
ifdef c
    C := $(c)
endif
ifndef C
    C := ALL
endif
LOWER_C := $(shell echo $(C) | tr A-Z a-z)
UPPER_C := $(shell echo $(C) | tr a-z A-Z)
export C LOWER_C UPPER_C

# Configure debug
ifdef d
    D := $(d)
endif
ifndef D
    D = 1
endif
ifeq ($(D),1)
	DEBUG = -g -DDEBUG -Os
else
	DEBUG = -Os
endif
ifdef dl
    DL := $(dl)
endif
ifdef DL
	FLAGS += -DDL=$(DL)
endif
ifdef tl
    TL := $(tl)
endif
ifdef TL
	FLAGS += -DTL=$(TL)
endif

# Configure emul
ifdef z
    Z := $(z)
endif
ifndef Z
    Z = 0
endif
ifeq ($(Z),1)
	FLAGS += -DEMUL
endif

# Configure tests
ifdef t
    T := $(t)
endif
ifndef T
    T := NONE
else
	FLAGS += -DHAS_TEST
endif
LOWER_T := $(shell echo $(T) | tr A-Z a-z)
UPPER_T := $(shell echo $(T) | tr a-z A-Z)
export T LOWER_T UPPER_T
ifeq ($(LOWER_T),all)
	FLAGS += -DTEST_ALL
endif
FLAGS += -DTEST=$(LOWER_T)

# Configure XRDC
ifdef x
    X := $(x)
endif
ifndef X
    X = 1
endif
ifeq ($(X),1)
	FLAGS += -DXRDC_SUPPORT
endif
ifeq ($(X),2)
	FLAGS += -DXRDC_SUPPORT -DXRDC_NOCHECK
endif

# Configure MSI posted writes
ifdef w
    W := $(w)
endif
ifndef W
    W = 1
endif
FLAGS += -DPOST_WR_LIMIT=$(W)

# Configure AutoCal
ifdef ac
    AC := $(ac)
endif
ifndef AC
    AC = 0
endif
ifeq ($(AC),1)
	FLAGS += -DAUTOCAL
endif

FLAGS += -DSREV_$(UPPER_R)

OUT := $(OUT)_$(LOWER_R)

# Configure monitor
ifdef m
    M := $(m)
endif
ifndef M
    M = 0
endif
ifndef EM
    EM = 0
endif
ifeq ($(M),1)
	FLAGS += -DMONITOR
endif
ifeq ($(EM),1)
	FLAGS += -DEXPORT_MONITOR
endif

# Check config
ifeq ($(D),0)
ifeq ($(M),1)
$(error M=1 and D=0 are mutually exclusive)
endif
endif

WARNS = -Wall -Wextra -Wno-missing-braces -Wno-missing-field-initializers \
    -Wfloat-equal -Wswitch-default -Wcast-align \
    -Wpointer-arith -Wredundant-decls -Wbad-function-cast -Wstrict-prototypes \
    -Wundef -Wcast-qual -Wshadow \
    -Wold-style-definition -Wno-unused-parameter -Werror

# Set common flags
rootdir := $(CURDIR)
SRC = platform
DOC = doc
FLAGS += $(DEBUG) ${WARNS}
# Configure tools
ifeq ($(wildcard $(TOOLS)/srec/srec_cat),)
    SREC_CAT = srec_cat 
else
    SREC_CAT = $(TOOLS)/srec/srec_cat    
endif
ifeq ($(wildcard $(TOOLS)/cppcheck/cppcheck),)
    CPPCHECK = cppcheck   
else
    CPPCHECK = $(TOOLS)/cppcheck/cppcheck   
endif

# Configure compiler
ifeq ($(HW), SIMU)
    #GNU GCC
    AS		= as
    LD		= ld
    CC		= gcc
    CPP		= cpp
    AR		= ar
    NM		= nm
    LDR		= ldr
    STRIP	= strip
    OBJCOPY	= objcopy
    OBJDUMP	= objdump

    INCLUDE = -I$(rootdir)/platform/ \
              -I$(rootdir)/platform/CMSIS/Include/ \
              -I$(rootdir)/platform/drivers/common/ \
              -I$(rootdir)/platform/devices/ \
			  -I$(rootdir)/platform/devices/$(DEVICE)/ \
			  -I$(rootdir)/platform/devices/MX8/ \
              -I$(rootdir)/platform/config/$(CONFIG)/ \
              -I$(rootdir)/platform/config/$(CONFIG)/$(UPPER_C) \
              -I$(rootdir)/platform/board/$(BOARD)
    CFLAGS = $(FLAGS) -std=gnu99 -m32 -fshort-enums
    LDFLAGS = -m32 -Wl,-lrt
    FLAGS += -DSIMU -DNO_DEVICE_ACCESS
    WARNS += -Wformat=2 -Wnested-externs
else
	CROSS_COMPILE = $(TOOLS)/gcc-arm-none-eabi-*/bin/arm-none-eabi-
    #GNU GCC
    AS		 = $(CROSS_COMPILE)as
    LD		 = $(CROSS_COMPILE)ld
    CC		 = $(CROSS_COMPILE)gcc
    CPP		 = $(CROSS_COMPILE)cpp
    AR		 = $(CROSS_COMPILE)ar
    NM		 = $(CROSS_COMPILE)nm
    LDR		 = $(CROSS_COMPILE)ldr
    STRIP	 = $(CROSS_COMPILE)strip
    OBJCOPY	 = $(CROSS_COMPILE)objcopy
    OBJDUMP	 = $(CROSS_COMPILE)objdump
    CPPCHECK = $(TOOLS)/cppcheck/cppcheck   

    SYSROOT  := $(shell $(CC) -print-sysroot)
    INCLUDE = -I$(SYSROOT)/usr/include \
              -I$(rootdir)/platform/ \
              -I$(rootdir)/platform/CMSIS/Include/ \
              -I$(rootdir)/platform/drivers/common/ \
              -I$(rootdir)/platform/devices/ \
			  -I$(rootdir)/platform/devices/$(DEVICE)/ \
			  -I$(rootdir)/platform/devices/MX8/ \
              -I$(rootdir)/platform/config/$(CONFIG)/ \
              -I$(rootdir)/platform/config/$(CONFIG)/$(UPPER_C) \
              -I$(rootdir)/platform/board/$(BOARD)
    ARCHFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 
    CFLAGS = $(INCLUDE) $(ARCHFLAGS) $(FLAGS) -ffunction-sections -std=c99 -mno-unaligned-access -pipe -fno-common -ffunction-sections -fdata-sections -ffreestanding -mapcs -std=c99 -fno-builtin -fshort-enums
    AFLAGS = $(INCLUDE) $(ARCHFLAGS) $(FLAGS) -fno-common -ffunction-sections -fdata-sections -mno-unaligned-access -ffreestanding -fno-builtin -std=c99 -pipe
    LDFLAGS = -Wl,--defsym=__ram_vector_table__=1 -Wl,-Bstatic -Wl,--gc-sections -nostdlib
    LIB_C    := $(shell $(CC) $(ARCHFLAGS) --print-file-name=libc_nano.a)
    LIB_GCC  := $(shell $(CC) $(ARCHFLAGS) --print-file-name=libgcc.a)
    WARNS += -Wformat=0 -Wunreachable-code
endif

export $(CFLAGS)

# Define Components to build
SUBCOMPS := devices test

ifdef ROM
	SUBCOMPS += rom
else
	SUBCOMPS += main
endif

ifeq ($(HW), REAL)
	SUBCOMPS += utilities
endif

# Define drivers, subsystems, and services to build for the SoC

include $(SRC)/config/$(CONFIG)/soc.bom

# Define drivers, subsystems, and services to build for the board

include $(SRC)/board/$(BOARD)/board.bom

SUBCOMPS += soc/$(SOC)

include $(foreach subcomp,$(SUBCOMPS),$(rootdir)/$(SRC)/$(subcomp)/Makefile)

ifeq ($(HW), REAL)
	DRV += $(DRV2)
endif

ifeq ($(DEVICE), ALL)
    DRV := $(shell find $(SRC)/drivers -name Makefile)
    SS := $(shell find $(SRC)/ss -name Makefile)
    BRD := $(shell find $(SRC)/board -name Makefile)
    SVC := $(shell find $(SRC)/svc -mindepth 1 -type d)
    DOX_API := $(foreach svc,$(SVC),$(svc)/api.h)
    include $(foreach drv,$(DRV),$(drv))
    include $(foreach ss,$(SS),$(ss))
    include $(foreach brd,$(BRD),$(brd))
    include $(foreach svc,$(SVC),$(svc)/Makefile)
else
    include $(foreach drv,$(DRV),$(SRC)/drivers/$(drv)/Makefile)
    include $(SRC)/ss/inf/Makefile
    include $(foreach ss,$(SS),$(rootdir)/$(SRC)/ss/$(ss)/Makefile)
    include $(SRC)/board/$(BOARD)/Makefile
    include $(foreach svc,$(SVC),$(rootdir)/$(SRC)/svc/$(svc)/Makefile)
    DOX_API := $(foreach svc,$(SVC),platform/svc/$(svc)/api.h)
    DOX_ADD := platform/config/$(CONFIG)
    DOX_ADD += $(foreach drv,$(DRV),platform/drivers/$(drv))
    DOX_ADD += $(foreach drv,$(DRV2),platform/drivers/$(drv))
    DOX_ADD += platform/ss/inf
    DOX_ADD += $(foreach ss,$(SS),platform/ss/$(ss))
    DOX_ADD += platform/board/$(BOARD)
    DOX_ADD += $(foreach svc,$(SVC),platform/svc/$(svc))
endif

# Dox input files

DOX_SOC := doc/resources.md doc/clocks.md doc/controls.md doc/pads.md

ifdef EXP
	DOX_SOC += doc/memmap.md
	MD += doc/memmap.md
endif

# Build rules

all : help

ifndef NO_IPC
    OBJS += ${RPCS} ${RPCL}
endif

CONFIGH+=$(SRC)/config/$(CONFIG)/pads.h
CONFIGH+=$(SRC)/config/$(CONFIG)/pad_data.h
CONFIGH+=$(SRC)/config/$(CONFIG)/pad_priority.h
PAD_MD += $(SRC)/config/pad.txt
MEMMAP_MD += $(SRC)/devices/$(DEVICE)/memmap.txt

BIH+=$(SRC)/main/build_info.h

INCH+= $(SRC)/config/$(CONFIG)/all_api.h $(SRC)/config/$(CONFIG)/all_svc.h \
	$(SRC)/config/$(CONFIG)/all_ss.h $(SRC)/config/$(CONFIG)/all_config.h

MD+=doc/resources.md doc/clocks.md doc/controls.md doc/pads.md

.SECONDARY : ${INCH} ${INFH} ${CONFIGH} ${RPCH} ${RPCC} ${RSRC_MD} ${CLK_MD} $(CTRL_MD) $(MEMMAP_MD)

.INTERMEDIATE : ${MD}

.PRECIOUS : ${DCDH}

# Include config makefiles
MAKERULE := $(shell find makefiles -name Makefile.rules)
include $(foreach makerule,$(MAKERULE),$(makerule))

$(OUT)/sc : ${OBJS}
	${CC} -o "$@" ${OBJS} ${LDFLAGS}

-include $(OBJS:.o=.d)

.SUFFIXES :

$(OUT)/board/%.o : $(SRC)/board/%.c ${INCH} ${INFH} ${CONFIGH} ${BIH} ${DCDH}
	@echo "Compiling $<"
	$(AT)${CC} ${CFLAGS} ${INCLUDE} -c $< -o $@

$(SRC)/board/$(CONFIG)_$(B)/dcd/%.h : $(SRC)/board/$(CONFIG)_$(B)/dcd/%.cfg
	@echo "Generating $@"
	$(AT)rm -f $(*F).temp
	$(AT)${CPP} -P -I$(SRC)/devices $< $(*F).temp
	$(AT)sed -i -f bin/dcd.sed $(*F).temp
	$(AT)echo "#ifndef $(subst .,_,$(*F))_H" > $@
	$(AT)echo "#define $(subst .,_,$(*F))_H" >> $@
	$(AT)cat $(*F).temp >> $@
	$(AT)echo '#endif' >> $@
	$(AT)rm -f $(*F).temp

$(SRC)/board/$(CONFIG)_$(B)/dcd/dcd.h : $(SRC)/board/$(CONFIG)_$(B)/dcd/$(DDR_CON).h
	@echo "Generating $@ from $<"
	$(AT)rm -f $@
	$(AT)echo "#ifndef DCD_H" > $@
	$(AT)echo "#define DCD_H" >> $@
	$(AT)echo "#include \"$(DDR_CON).h\"" >> $@
	$(AT)echo '#endif' >> $@

$(SRC)/board/$(CONFIG)_$(B)/dcd/%_retention.h : $(SRC)/board/$(CONFIG)_$(B)/dcd/%.cfg
	@echo "Generating $@"
	$(AT)rm -f $(*F)_ret.temp
	$(AT)${CPP} -P -I$(SRC)/devices $< $(*F)_ret.temp
	$(AT)sed -i -f bin/retention.sed $(*F)_ret.temp
	$(AT)echo "#ifndef $(subst .,_,$(*F))_RET_H" > $@
	$(AT)echo "#define $(subst .,_,$(*F))_RET_H" >> $@
	$(AT)cat $(*F)_ret.temp >> $@
	$(AT)echo '#endif' >> $@
	$(AT)rm -f $(*F)_ret.temp

$(SRC)/board/$(CONFIG)_$(B)/dcd/dcd_retention.h : $(SRC)/board/$(CONFIG)_$(B)/dcd/$(DDR_CON)_retention.h
	@echo "Generating $@ from $<"
	$(AT)rm -f $@
	$(AT)echo "#ifndef DDR_RETENTION_H" > $@
	$(AT)echo "#define DDR_RETENTION_H" >> $@
	$(AT)echo "#include \"$(DDR_CON)_retention.h\"" >> $@
	$(AT)echo '#endif' >> $@

$(OUT)/drivers/pmic/%.o : $(SRC)/drivers/pmic/%.c ${INCH} ${INFH} ${CONFIGH} ${BIH} ${DCDH}
	@echo "Compiling $<"
	$(AT)${CC} ${CFLAGS} ${INCLUDE} -c $< -o $@


$(OUT)/%.o : $(SRC)/%.S
	@echo "Assembling $<"
	$(AT)${CC} ${AFLAGS} ${INCLUDE} -c $< -o $@

$(OUT)/scfw.srec : $(OUT)/scfw.elf $(OUT)/scfw_tcm.elf
	@echo "Generating srec ...."
	$(AT)-rm -f $(OUT)/scfw_fb.bin
	$(AT)$(OBJCOPY) -O srec --gap-fill 0x0 --pad-to 0x10000 $(OUT)/scfw.elf $(OUT)/scfw.srec --strip-debug
	$(AT)$(OBJCOPY) -O binary --gap-fill 0x0 $(OUT)/scfw.elf $(OUT)/scfw.bin --strip-debug

$(OUT)/scfw.hex : $(OUT)/scfw.srec
	@echo "Generating hex ...."
	@-rm -f $(OUT)/scfw.hex
	$(AT)$(SREC_CAT) $(OUT)/scfw.srec -byte_swap 4 -crop 0x00000000 0xffffffff -offset $(SREC_OFFSET) -Line_Length=82 -output $(OUT)/scfw.hex -vmem 32
	@echo "done."

$(OUT)/scfw_m4.hex : $(OUT)/scfw.srec
	@echo "Generating hex ...."
	@-rm -f $(OUT)/scfw.hex
	$(AT)$(SREC_CAT) $(OUT)/scfw.srec -byte_swap 4 -crop 0x00000000 0xffffffff -offset -0x00000000 -Line_Length=82 -output $(OUT)/scfw.hex -vmem 32
	@echo "Converting M4 image located at img/m4_image.elf"
	$(AT)objcopy -O srec     --gap-fill 0x0 --pad-to 0x40000 img/m4_image.elf img/m4_image.srec --strip-debug
	$(AT)$(SREC_CAT) img/m4_image.srec  -byte_swap 4 -crop 0x00000000 0xffffffff  -offset 0x00040000 -Line_Length=82 -output img/m4_image_temp.hex -vmem 32
	@tail -n +2 img/m4_image_temp.hex > img/m4_image.hex
	@echo "done converting appending to SCU image..."
	$(AT)cat img/m4_image.hex >> $(OUT)/scfw.hex
	@echo "done."

$(OUT)/scfw_ddr.hex : $(OUT)/scfw.srec img/u-boot.hex
	@echo "Generating hex ...."
	@-rm -f $(OUT)/scfw_ddr.hex
	$(AT)$(SREC_CAT) $(OUT)/scfw.srec -byte_swap 4 -crop 0x00000000 0xffffffff -offset -0x00000000 -Line_Length=82 -output $(OUT)/scfw_ddr.hex -vmem 32
	$(AT)cat img/u-boot.hex >> $(OUT)/scfw_ddr.hex
	@echo "done."

$(OUT)/scfw.elf : $(OBJS) $(SRC)/devices/$(DEVICE)/linker/gcc/$(DEVICE)_flash.ld
	@echo "Linking $@ ...."
	$(AT)$(CC) $(ARCHFLAGS) $(LDFLAGS) -o $@ -T $(SRC)/devices/$(DEVICE)/linker/gcc/$(DEVICE)_flash.ld $(OBJS) -lc_nano -lgcc -Wl,-Map=$(OUT)/scfw.map

$(OUT)/scfw_tcm.elf : $(OBJS) $(SRC)/devices/$(DEVICE)/linker/gcc/$(DEVICE)_tcm.ld
	@echo "Linking $@ ...."
	$(AT)$(CC) $(ARCHFLAGS) $(LDFLAGS) -o $@ -T $(SRC)/devices/$(DEVICE)/linker/gcc/$(DEVICE)_tcm.ld $(OBJS) -lc_nano -lgcc -Wl,-Map=$(OUT)/scfw_tcm.map
	$(AT)$(OBJCOPY) -O binary --gap-fill 0x0 $(OUT)/scfw_tcm.elf $(OUT)/scfw_tcm.bin --strip-debug
	$(AT)$(OBJCOPY) -O srec --gap-fill 0x0 --pad-to 0x30000 $(OUT)/scfw_tcm.elf $(OUT)/scfw_tcm.srec --strip-debug
	$(AT)$(SREC_CAT) $(OUT)/scfw_tcm.srec -byte_swap 4 -crop 0x00000000 0xffffffff -offset 0x11000000 -Line_Length=82 -output $(OUT)/scfw_tcm.hex -vmem 32

$(OUT)/rom.srec : $(OUT)/rom.elf
	@echo "Generating srec ...."
	$(AT)-rm -f $(OUT)/rom_fb.bin
	$(AT)$(OBJCOPY) -O srec --gap-fill 0x0 --pad-to 0x40000 $(OUT)/rom.elf $(OUT)/rom.srec --strip-debug
	$(AT)$(OBJCOPY) -O binary --gap-fill 0x0 $(OUT)/rom.elf $(OUT)/rom.bin --strip-debug
	$(AT)dd if=$(OUT)/rom.bin of=$(OUT)/rom_fb.bin bs=64k count=2
	$(AT)dd if=$(OUT)/rom.bin of=$(OUT)/rom_fb.bin skip=1 seek=3 bs=64k count=1

$(OUT)/rom.hex : $(OUT)/rom.srec
	@echo "Generating hex ...."
	@-rm -f $(OUT)/rom.hex
	$(AT)$(SREC_CAT) $(OUT)/rom.srec -byte_swap 4 -crop 0x00000000 0xffffffff -offset -0x00000000 -Line_Length=82 -output $(OUT)/rom.hex -vmem 32
	@echo "done."

$(OUT)/rom.elf : $(OBJS) $(SRC)/devices/$(DEVICE)/linker/gcc/$(DEVICE)_rom.ld
	@echo "Linking ...."
	$(AT)$(LD) $(LDFLAGS) -o $@ -T $(SRC)/devices/$(DEVICE)/linker/gcc/$(DEVICE)_rom.ld --start-group $(OBJS) -l:$(LIB_C) -l:$(LIB_GCC) --end-group -Map $(OUT)/rom.map

$(OUT)/ : FORCE
	$(AT)-mkdir -p $(DIRS)

FORCE:

help :
	@/bin/echo -e "Usage: make TARGET OPTIONS\n"
	@/bin/echo -e "Targets:"
	@/bin/echo -e "\thelp           : display help text"
	@/bin/echo -e "\tclean          : remove all build files\n"
	@/bin/echo -en $(HELP)
	@/bin/echo -e "\nOptions:"
	@/bin/echo -e "\tV=0            : quite output (default)"
	@/bin/echo -e "\tV=1            : verbose output"
	@/bin/echo -e "\tD=0            : configure for no debug"
	@/bin/echo -e "\tD=1            : configure for debug (default)"
	@/bin/echo -e "\tDL=<level>     : configure debug level (0-5)"
	@/bin/echo -e "\tB=<board>      : configure board (default=val)"
	@/bin/echo -e "\tDDR_CON=<file> : specify DDR config file"
	@/bin/echo -e "\tR=<srev>       : silicon revision (default=A0)"
	@/bin/echo -e "\tM=0            : no debug monitor (default)"
	@/bin/echo -e "\tM=1            : include debug monitor"
	@/bin/echo -e "\tT=<test>       : run tests rather than boot next core"
	@/bin/echo -en $(HELP_OPT)

