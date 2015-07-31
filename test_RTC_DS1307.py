#!/usr/bin/env python
#
# Test RTC_DS1307

# imports

import sys
import time
import datetime

import RTC_DS1307

# Main Program

print "Program Started at:"+ time.strftime("%Y-%m-%d %H:%M:%S")

filename = time.strftime("%Y-%m-%d%H:%M:%SRTCTest") + ".txt"
starttime = datetime.datetime.utcnow()

ds1307 = RTC_DS1307.RTC_DS1307(3, 0x68)
ds1307.write_now()

# Main Loop - sleeps 10 minutes, then reads and prints values of all clocks


while True:

	currenttime = datetime.datetime.utcnow()

	deltatime = currenttime - starttime
 
	print ""
	print "Raspberry Pi=\t" + time.strftime("%Y-%m-%d %H:%M:%S")
	
	print "DS1307=\t\t%s" % ds1307.read_datetime()

	time.sleep(10.0)
