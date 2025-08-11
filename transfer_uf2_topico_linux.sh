#!/bin/bash

uf2_file_path=$(find . -name "*.uf2")
pico_dir=/media/vichy/RPI-RP2
cp $uf2_file_path $pico_dir