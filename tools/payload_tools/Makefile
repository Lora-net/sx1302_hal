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

LIBS :=

### general build targets

all: payload_crc payload_diff payload_gen

clean:
	rm -f payload_crc payload_diff payload_gen
	rm -f $(OBJDIR)/*.o

### library module target

### static library

### test programs

payload_crc: payload_crc.c
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

payload_diff: payload_diff.c
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

payload_gen: payload_diff.c tinymt32.c
	$(CC) $(CFLAGS) -L. $< -o $@ $(LIBS)

### EOF
