### Environment constants

ARCH ?=
CROSS_COMPILE ?=
export

### general build targets

all:
	$(MAKE) all -e -C libloragw
	$(MAKE) all -e -C packet_forwarder

clean:
	$(MAKE) clean -e -C libloragw
	$(MAKE) clean -e -C packet_forwarder

### EOF
