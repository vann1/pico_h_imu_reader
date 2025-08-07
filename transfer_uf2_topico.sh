#!/bin/bash

sudo mount -t drvfs D: /mnt/d
echo "Mountattiin D: levy"

bootshell_addr="$PICO_BOOTSEL_PATH"
uf2_file=$(find . -name "*.uf2")
echo "UF 2 file location: $uf2_file"
cp "$uf2_file" "$bootshell_addr/"
echo "UF2 file copied into '$bootshell_addr'"
