#!/bin/sh

pLoc=.venv/bin/python
sReader=usb_serial_reader.py

source build_and_run.sh
sleep 5
sudo $pLoc $sReader asdasd
sleep 5
source transfer_uf2_topico_linux.sh
sleep 3
sudo $pLoc $sReader
