### get external defined data

### constant symbols

CFLAGS := -O2 -Wall -Wextra -std=c99 -I.

### linking options

LIBS :=

### general build targets

all: payload_crc payload_diff payload_gen

clean:
	rm -f payload_crc payload_diff payload_gen
	rm -f *.o

### rules

%.o : %.c
	gcc -c $(CFLAGS) $< -o $@

### test programs

payload_crc: payload_crc.o
	gcc $(CFLAGS) -o $@ $^ $(LIBS)

payload_diff: payload_diff.o
	gcc $(CFLAGS) -o $@ $^ $(LIBS)

payload_gen: tinymt32.o payload_gen.o
	gcc $(CFLAGS) -o $@ $^ $(LIBS)

### EOF
