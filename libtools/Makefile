### get external defined data

### constant symbols

ARCH ?=
CROSS_COMPILE ?=
CC := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar

CFLAGS := -O2 -Wall -Wextra -std=c99 -Iinc -I.

OBJDIR = obj
INCLUDES = $(wildcard inc/*.h)

### linking options

### general build targets

all: libtinymt32.a

clean:
	rm -f libtinymt32.a
	rm -f $(OBJDIR)/*.o

### library module target

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: src/%.c $(INCLUDES) | $(OBJDIR)
	$(CC) -c $(CFLAGS) $< -o $@

### static library

libtinymt32.a:  $(OBJDIR)/tinymt32.o
	$(AR) rcs $@ $^

### test programs

### EOF
