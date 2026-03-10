#!/usr/bin/env bash
set -e

cd ~/Documents/Progetto_Drone/Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent udp4 -p 8888
