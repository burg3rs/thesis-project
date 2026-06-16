#!/bin/bash

echo "Setting permissions for serial devices..."
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1

echo "Installing pyserial..."
pip install pyserial

echo "Done."
