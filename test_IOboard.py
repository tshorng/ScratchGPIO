#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys
import IOboard

if len(sys.argv) != 4:
    print("usage: {0} Address Channel[0|1|2|3] DigitalOut[0~255].".format(sys.argv[0]))
    print("example: {0} I2C Address Read from Channel 0 and write 127 to D/A converter. ".format(sys.argv[0]))
    sys.exit(2)

address = int("0x"+sys.argv[1], 16)
channel = int(sys.argv[2])
out = int(sys.argv[3])

if channel not in [0, 1, 2, 3]:
	print("Channel[0|1|2|3]")
if out not in range(0,256):
	print("DigitalOut[0~255]")

IOboard.init(address)

read = IOboard.readPCF8591(channel)
IOboard.writePCF8591(out)

inV = 3.3*read/255
outV = 3.3*out/255
print("A/D converter reads voltage = {0}".format(inV))
print("D/A converter writes voltage = {0}".format(outV))

data = 255
IOboard.writeAT45Buff(data)
readData = IOboard.readAT45Buff()
print("Read AT45: ", readData)
