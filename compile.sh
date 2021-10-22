#!/usr/bin/env sh

build_all() {
    make clean
    make -j 4
}

copy_to_output() {
    mkdir -p "$OUTPUT_DIR"
    cp "$ROOT_DIR/packet_forwarder/lora_pkt_fwd" "$OUTPUT_DIR/lora_pkt_fwd"
    cp "$ROOT_DIR/util_chip_id/chip_id" "$OUTPUT_DIR/chip_id"
}

build_all
copy_to_output