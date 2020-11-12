#!/bin/sh
BOARD=atmega328old

#echo "Compiling..."
#arduino-cli compile -v --fqbn arduino:avr:nano:cpu=$BOARD ${1}.ino
echo "Uploading..."
arduino-cli upload -v -p /dev/cu.wchusbserial14110 --fqbn arduino:avr:nano:cpu=$BOARD $1
echo "Done"
