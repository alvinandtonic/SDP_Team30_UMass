#!/bin/bash

sudo apt update
sudo apt install python3-pip -y
python3 -m venv myenv
source myenv/bin/activate

sudo apt update
pip3 install pyserial
sudo apt install libglib2.0-dev
pip3 install bluepy

sudo python3 sdp.py