#!/usr/bin/env python
#
# Hardware 28BYJ-48 Stepper 
# Gear Reduction Ratio: 1/64 
# Step Torque Angle: 5.625 degrees /64
# 360/5.625 = 64
#

import sys
import os
import time

import RPi.GPIO as GPIO

# The stepper motor can be driven in different ways
# See http://en.wikipedia.org/wiki/Stepper_motor

# Highest torque speed
out1=[]
out2=[]
out3=[]
out4=[]

# Full step drive (Maximum Torque)
Aout1=[1,1,0,0]
Aout2=[0,1,1,0]
Aout3=[0,0,1,1]
Aout4=[1,0,0,1]

# Wave drive (increase angular resolution)
Bout1=[1,0,0,0]
Bout2=[0,1,0,0]
Bout3=[0,0,1,0]
Bout4=[0,0,0,1]

# Half step drive ( Maximum angle minimum torque)
Cout1=[1,1,0,0,0,0,0,1]
Cout2=[0,1,1,1,0,0,0,0]
Cout3=[0,0,0,1,1,1,0,0]
Cout4=[0,0,0,0,0,1,1,1]

class Motor:

	CLOCKWISE = 0
	ANTICLOCKWISE = 1
	GEARING = 64
	STEPS = 8
	REVOLUTION = GEARING * STEPS
	NORMAL = 0.0025
	SLOW = NORMAL * 2
	running = False 
	position = 0
	halt = False

	def __init__(self, pin1, pin2, pin3, pin4):
		self.pin1 = pin1
		self.pin2 = pin2
		self.pin3 = pin3
		self.pin4 = pin4
		self.speed = self.NORMAL
		self.moving = False
		self.setFullStepDrive()
		return

	# Initialise GPIO pins for this motor
	def init(self):
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.pin1,GPIO.OUT)
		GPIO.setup(self.pin2,GPIO.OUT)
		GPIO.setup(self.pin3,GPIO.OUT)
		GPIO.setup(self.pin4,GPIO.OUT)
		self.zeroPosition()
		return	

	# Reset (stop) motor
	def reset(self):
		GPIO.output(self.pin1,GPIO.LOW)
		GPIO.output(self.pin2,GPIO.LOW)
		GPIO.output(self.pin3,GPIO.LOW)
		GPIO.output(self.pin4,GPIO.LOW)
		self.moving = False
		return	

	# Turn the motor
	def turn(self,steps,direction):
		global CLOCKWISE
		self.stop()
		self.moving = True
		self.steps = steps		
		mrange = len(out1)
		while self.steps > 0:
			if direction == self.CLOCKWISE:
				for pin in range(mrange):
					GPIO.output(self.pin1,out1[pin])
					GPIO.output(self.pin2,out2[pin])
					GPIO.output(self.pin3,out3[pin])
					GPIO.output(self.pin4,out4[pin])
					time.sleep(self.speed)
				self.incrementPosition()
			else:
				for pin in reversed(range(mrange)):
					GPIO.output(self.pin1,out1[pin])
					GPIO.output(self.pin2,out2[pin])
					GPIO.output(self.pin3,out3[pin])
					GPIO.output(self.pin4,out4[pin])
					time.sleep(self.speed)
				self.decrementPosition()
			self.steps -= 1
			if self.halt:
				break
		self.stop()
		return

	def interrupt(self):
		self.halt = True
		return

	# Increment current position 
	def incrementPosition(self):
		self.position += 1
		if self.position >= self.REVOLUTION:
			self.position -= self.REVOLUTION
		return self.position

	# Increment current position 
	def decrementPosition(self):
		self.position -= 1
		if self.position < 0:
			self.position += self.REVOLUTION
		return self.position

	# Increment current position 
	def zeroPosition(self):
		self.position = 0
		return self.position

	# Is the motor running (Future use)
	def isRunning(self):
		return self.running

	# Goto a specific position
	def goto(self, position):
		newpos = position

		while newpos > self.REVOLUTION:
			newpos -= self.REVOLUTION
		
		delta =  newpos - self.position

		# Figure which direction to turn
		if delta > self.REVOLUTION/2:
			delta = self.REVOLUTION/2 - delta

		elif delta < (0-self.REVOLUTION/2):
			delta = self.REVOLUTION + delta

		# Turn the most the efficient direction
		if delta > 0:
			self.turn(delta,self.CLOCKWISE)

		elif delta < 0:
			delta = 0 - delta
			self.turn(delta,self.ANTICLOCKWISE)

		self.position = newpos
		if self.position == self.REVOLUTION:
			self.position = 0
		return self.position			

	# Stop the motor (calls reset)
	def stop(self):
		self.reset()	
		return

	# Lock the motor (also keeps motor warm)
	def lock(self):
		GPIO.output(self.pin1,GPIO.HIGH)
		GPIO.output(self.pin2,GPIO.LOW)
		GPIO.output(self.pin3,GPIO.LOW)
		GPIO.output(self.pin4,GPIO.HIGH)
		self.moving = False
		return	


	# Set Full Step Drive
	def setFullStepDrive(self):
		global out1,out2,out3,out4
		global Aout1,Aout2,Aout3,Aout4
		out1 = Aout1
		out2 = Aout2
		out3 = Aout3
		out4 = Aout4
		self.speed = self.NORMAL
		return

	# Set Half Step Drive
	def setHalfStepDrive(self):
		global out1,out2,out3,out4
		global Bout1,Bout2,Bout3,Bout4
		out1 = Bout1
		out2 = Bout2
		out3 = Bout3
		out4 = Bout4
		self.speed = self.NORMAL
		return

	# Set Wave Drive
	def setWaveDrive(self):
		global out1,out2,out3,out4
		global Cout1,Cout2,Cout3,Cout4
		out1 = Cout1
		out2 = Cout2
		out3 = Cout3
		out4 = Cout4
		self.speed = self.NORMAL/3
		return

	# Set speed of motor
	def setSpeed(self,speed):
		self.speed = speed
		return
	
	# Get motor position
	def getPosition(self):
		return self.position
		

# End of Motor class
