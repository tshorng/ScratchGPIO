#!/usr/bin/env python

import os
import time
import atexit
from step_motor import Motor

#motora = Motor(17,18,27,22)

motora = Motor(11,7,16,26)
motorb = Motor(4,25,24,23)
minute = 60

motora.init()
motorb.init()

def finish():
	motora.stop()
	motorb.stop()
	return

atexit.register(finish)

motora.lock()
motorb.lock()

print "revolution = " + str(Motor.REVOLUTION)
print str(motora.goto(200))
time.sleep(2)
print str(motora.goto(100))
time.sleep(2)
print str(motora.goto(900))
time.sleep(2)

speed = Motor.NORMAL

# Uncomment for setting step type
motora.setHalfStepDrive()
#motora.setFullStepDrive()
#motora.setWaveDrive()

while True:
	print "Motor A Clockwise"
	t1 = time.time()
	motora.turn(1*Motor.REVOLUTION, Motor.CLOCKWISE)
	t2 = time.time()
        print "time " + str(t2-t1)
	motora.lock()
	time.sleep(1)

	print "Motor A Anti-Clockwise"
	t1 = time.time()
	motora.turn(1*Motor.REVOLUTION, Motor.ANTICLOCKWISE)
	t2 = time.time()
        print "time " + str(t2-t1)
	motora.lock()
	time.sleep(1)

	print "Motor B Clockwise"
	t1 = time.time()
	motorb.turn(1*Motor.REVOLUTION, Motor.CLOCKWISE)
	t2 = time.time()
        print "time " + str(t2-t1)

	print "Motor B Anti-Clockwise"
	t1 = time.time()
	motorb.turn(1*Motor.REVOLUTION, Motor.ANTICLOCKWISE)
	t2 = time.time()
        print "time " + str(t2-t1)
	motorb.lock()
	time.sleep(5)

	if speed == Motor.NORMAL:
                speed = Motor.SLOW
        else:
                speed = Motor.NORMAL
        motora.setSpeed(speed)


# End of program
