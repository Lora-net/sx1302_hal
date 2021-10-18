FROM balenalib/raspberry-pi-debian:buster-build as sx1302-hal-builder

ENV ROOT_DIR=/opt

WORKDIR "$ROOT_DIR"

# Copy upstream source into expected location
COPY . "$ROOT_DIR"

RUN . "$ROOT_DIR/compile.sh"
