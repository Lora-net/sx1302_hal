### get external defined data

LIBLORAGW_VERSION := `cat ../VERSION`
include library.cfg
include ../target.cfg

### constant symbols

ARCH ?=
CROSS_COMPILE ?=
CC := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar

CFLAGS := -O2 -Wall -Wextra -std=c99 -Iinc -I.

OBJDIR = obj
INCLUDES = $(wildcard inc/*.h)

### linking options

LIBS := -lloragw -lrt -lm

### general build targets

all: libloragw.a test_loragw_spi test_loragw_reg test_loragw_hal_tx test_loragw_hal_rx test_loragw_cal test_loragw_capture_ram

clean:
	rm -f libloragw.a
	rm -f test_loragw_*
	rm -f $(OBJDIR)/*.o
	rm -f inc/config.h

install:
ifneq ($(strip $(TARGET_IP)),)
 ifneq ($(strip $(TARGET_DIR)),)
  ifneq ($(strip $(TARGET_USR)),)
	@echo "---- Copying libloragw files to $(TARGET_IP):$(TARGET_DIR)"
	@scp test_loragw_* $(TARGET_USR)@$(TARGET_IP):$(TARGET_DIR)
	@scp ../tools/reset_lgw.sh $(TARGET_USR)@$(TARGET_IP):$(TARGET_DIR)
  else
	@echo "ERROR: TARGET_USR is not configured in target.cfg"
  endif
 else
	@echo "ERROR: TARGET_DIR is not configured in target.cfg"
 endif
else
	@echo "ERROR: TARGET_IP is not configured in target.cfg"
endif

### transpose library.cfg into a C header file : config.h

inc/config.h: ../VERSION library.cfg
	@echo "*** Checking libloragw library configuration ***"
	@rm -f $@
	#File initialization
	@echo "#ifndef _LORAGW_CONFIGURATION_H" >> $@
	@echo "#define _LORAGW_CONFIGURATION_H" >> $@
	# Release version
	@echo "Release version   : $(LIBLORAGW_VERSION)"
	@echo "	#define LIBLORAGW_VERSION	"\"$(LIBLORAGW_VERSION)\""" >> $@
	# Debug options
	@echo "	#define DEBUG_AUX	$(DEBUG_AUX)" >> $@
	@echo "	#define DEBUG_SPI	$(DEBUG_SPI)" >> $@
	@echo "	#define DEBUG_REG	$(DEBUG_REG)" >> $@
	@echo "	#define DEBUG_HAL	$(DEBUG_HAL)" >> $@
	@echo "	#define DEBUG_GPS	$(DEBUG_GPS)" >> $@
	@echo "	#define DEBUG_GPIO	$(DEBUG_GPIO)" >> $@
	@echo "	#define DEBUG_LBT	$(DEBUG_LBT)" >> $@
	@echo "	#define DEBUG_RAD	$(DEBUG_RAD)" >> $@
	@echo "	#define DEBUG_CAL	$(DEBUG_CAL)" >> $@
	# Configuration options
	@echo "	#define BYPASS_FW_INIT	$(BYPASS_FW_INIT)" >> $@
	@echo "	#define FPGA_BOARD_16_CH	$(FPGA_BOARD_16_CH)" >> $@
	# end of file
	@echo "#endif" >> $@
	@echo "*** Configuration seems ok ***"

### library module target

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: src/%.c $(INCLUDES) inc/config.h | $(OBJDIR)
	$(CC) -c $(CFLAGS) $< -o $@

### static library

libloragw.a: $(OBJDIR)/loragw_spi.o $(OBJDIR)/loragw_aux.o $(OBJDIR)/loragw_reg.o $(OBJDIR)/loragw_sx1250.o $(OBJDIR)/loragw_sx125x.o $(OBJDIR)/loragw_sx1302.o $(OBJDIR)/loragw_cal.o $(OBJDIR)/loragw_hal.o $(OBJDIR)/tinymt32.o
	$(AR) rcs $@ $^

### test programs

test_loragw_spi: tst/test_loragw_spi.c
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

test_loragw_reg: tst/test_loragw_reg.c
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

test_loragw_hal_tx: tst/test_loragw_hal_tx.c
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

test_loragw_hal_rx: tst/test_loragw_hal_rx.c
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

test_loragw_capture_ram: tst/test_loragw_capture_ram.c
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

test_loragw_cal: tst/test_loragw_cal.c
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

### EOF
