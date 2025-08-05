#!/bin/bash

bootshell_addr="$HOME"
uf2_file=$(find . -name "*.uf2")
echo "UF 2 file location: $uf2_file"
cp "$uf2_file" "$bootshell_addr"
echo "UF2 file copied into '$bootshell_addr'"
