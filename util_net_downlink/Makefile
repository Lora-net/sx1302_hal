### get external defined data

include ../target.cfg

### User defined build options

ARCH ?=
CROSS_COMPILE ?=
BUILD_MODE := release
OBJDIR = obj

### ----- AVOID MODIFICATIONS BELLOW ------ AVOID MODIFICATIONS BELLOW ----- ###

ifeq '$(BUILD_MODE)' 'alpha'
  $(warning /\/\/\/ Building in 'alpha' mode \/\/\/\)
  WARN_CFLAGS   :=
  OPT_CFLAGS    := -O0
  DEBUG_CFLAGS  := -g
  LDFLAGS       :=
else ifeq '$(BUILD_MODE)' 'debug'
  $(warning /\/\/\/  Building in 'debug' mode \/\/\/\)
  WARN_CFLAGS   := -Wall -Wextra
  OPT_CFLAGS    := -O2
  DEBUG_CFLAGS  := -g
  LDFLAGS       :=
else ifeq  '$(BUILD_MODE)' 'release'
  $(warning /\/\/\/  Building in 'release' mode \/\/\/\)
  WARN_CFLAGS   := -Wall -Wextra
  OPT_CFLAGS    := -O2 -ffunction-sections -fdata-sections
  DEBUG_CFLAGS  :=
  LDFLAGS       := -Wl,--gc-sections
else
  $(error BUILD_MODE must be set to either 'alpha', 'debug' or 'release')
endif

### Application-specific variables
APP_NAME := net_downlink
APP_LIBS := -lparson -lbase64 -lpthread

### Environment constants
LIB_PATH := ../libtools

### Expand build options
CFLAGS := -std=c99 $(WARN_CFLAGS) $(OPT_CFLAGS) $(DEBUG_CFLAGS)
CC := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar

### General build targets
all: $(APP_NAME)

clean:
	rm -f obj/*.o
	rm -f $(APP_NAME)

install:
ifneq ($(strip $(TARGET_IP)),)
 ifneq ($(strip $(TARGET_DIR)),)
  ifneq ($(strip $(TARGET_USR)),)
	@echo "---- Copying net_downlink files to $(TARGET_IP):$(TARGET_DIR)"
	@ssh $(TARGET_USR)@$(TARGET_IP) "mkdir -p $(TARGET_DIR)"
	@scp net_downlink $(TARGET_USR)@$(TARGET_IP):$(TARGET_DIR)
  else
	@echo "ERROR: TARGET_USR is not configured in target.cfg"
  endif
 else
	@echo "ERROR: TARGET_DIR is not configured in target.cfg"
 endif
else
	@echo "ERROR: TARGET_IP is not configured in target.cfg"
endif

$(OBJDIR):
	mkdir -p $(OBJDIR)

### Compile main program
$(OBJDIR)/$(APP_NAME).o: src/$(APP_NAME).c | $(OBJDIR)
	$(CC) -c $< -o $@ $(CFLAGS) -Iinc -I../libtools/inc

### Link everything together
$(APP_NAME): $(OBJDIR)/$(APP_NAME).o
	$(CC) -L$(LIB_PATH) $^ -o $@ $(LDFLAGS) $(APP_LIBS)

### EOF
