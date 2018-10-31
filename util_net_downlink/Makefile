### User defined build options

ARCH ?=
CROSS_COMPILE ?=
BUILD_MODE := release

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
MODULES  := parson base64
APP_LIBS := -lpthread

### Expand application sub-module names
MOD_OBJ := $(addprefix obj/,$(addsuffix .o,$(MODULES)))
MOD_INC := $(addprefix inc/,$(addsuffix .h,$(MODULES)))

### Expand build options
CFLAGS := -std=c99 $(WARN_CFLAGS) $(OPT_CFLAGS) $(DEBUG_CFLAGS)
CC := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar

### General build targets
all: $(APP_NAME)

clean:
	rm -f $(MOD_OBJ)
	rm -f obj/$(APP_NAME).o
	rm -f $(APP_NAME)

### Compile submodules of the main program
$(MOD_OBJ): obj/%.o: src/%.c inc/%.h
	$(CC) -c $< -o $@ $(CFLAGS) -Iinc

### Compile main program
obj/$(APP_NAME).o: src/$(APP_NAME).c $(MOD_INC)
	$(CC) -c $< -o $@ $(CFLAGS) -Iinc

### Link everything together
$(APP_NAME): obj/$(APP_NAME).o $(MOD_OBJ)
	$(CC) $^ -o $@ $(LDFLAGS) $(APP_LIBS)

### EOF
