### get external defined data

### constant symbols

ARCH ?=
CROSS_COMPILE ?=
CC := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar

CFLAGS := -O2 -Wall -Wextra -std=c99 -Iinc -I. -Wno-strict-aliasing

OBJDIR = obj
INCLUDES = $(wildcard inc/*.h)

### linking options

### general build targets

all: libtinymt32.a libparson.a libbase64.a libcursor.a

clean:
	rm -f libtinymt32.a
	rm -f libparson.a
	rm -f libbase64.a
	rm -f libcursor.a
	rm -f $(OBJDIR)/*.o

### library module target

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: src/%.c $(INCLUDES) | $(OBJDIR)
	$(CC) -c $(CFLAGS) $< -o $@

### static library

libtinymt32.a:  $(OBJDIR)/tinymt32.o
	$(AR) rcs $@ $^

libparson.a:  $(OBJDIR)/parson.o
	$(AR) rcs $@ $^

libbase64.a:  $(OBJDIR)/base64.o
	$(AR) rcs $@ $^

libcursor.a:  $(OBJDIR)/cursor.o $(OBJDIR)/packing.o
	$(AR) rcs $@ $^

### test programs

### EOF
