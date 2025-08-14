#!/bin/bash

uf2_file_path=$(find . -name "*.uf2")
user=$(whoami)
pico_dir=/media/$user/RPI-RP2
cp $uf2_file_path $pico_dir