#!/bin/sh
BOARD=atmega328old

#PORT=/dev/cu.wchusbserial141110
#PORT=/dev/cu.wchusbserial141120

PORT=/dev/cu.wchusbserial14110
#PORT=/dev/cu.wchusbserial14310

#SCKETCH=OttoPLUS_ScratchAI

#echo "Compiling..."
#arduino-cli compile -v --fqbn arduino:avr:nano:cpu=$BOARD ${SCKETCH}.ino
#arduino-cli compile -v --fqbn arduino:avr:nano:cpu=$BOARD ${SCKETCH}.ino
echo "Uploading..."
arduino-cli upload -v -p $PORT --fqbn arduino:avr:nano:cpu=$BOARD $SCKETCH
echo "Done"
