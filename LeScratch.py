#!/usr/bin/env python

from __future__ import division
import os
import re
import sys
import time
import glob
import smbus
import string
import signal
import socket
import binascii
import datetime
from Tkinter import Tk
from array import array
import RPi.GPIO as GPIO
import step_motor as StepMotor
import RTC_DS1307
import DHTreader
import IOboard

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(True)

stepM1 = StepMotor.Motor(11,7,16,26)
stepM2 = StepMotor.Motor(4,25,24,23)

root = Tk()
root.withdraw()

Bus = smbus.SMBus(2)
PORT = 42001
HOST = "localhost"

Debug = False
Connected = False
Setup = False
I2Csetup = False

I2Inputs = []
SPInputs = []

# GPIO
GPIOset = [False] * 35
GPIOdir = ["OUT"] * 35

# LNdigital
LNout = [False] * 8
LNin = [False] * 8

# UltraSonic
GPIO_Trigger = 0
GPIO_Echo = 0

# DHT
temperature = 0
humidity = 0

# Sound Detect
soundPin = 0

# Touch Detect
touchPin = 0

# Tilt Detect
tiltPin = 0

# Light Sensor
lightPin = 0

# MCP23S17
SPI_SLAVE_ADDR = 0x40

SPI_IODIRA = 0x00
SPI_IODIRB = 0x01

SPI_GPIOA = 0x12
SPI_GPIOB = 0x13

SPI_IOCONA =0x0A
SPI_IOCONB =0x0B

SPI_OLATA =0x14
SPI_OLATB =0x15

SPI_INTENA = 0x04
SPI_INTENB = 0x05

SPI_DEFVALA = 0x06
SPI_DEFVALB = 0x07

SPI_INTCONA = 0x08
SPI_INTCONB = 0x09

SPI_GPPUA = 0x0C
SPI_GPPUB = 0x0D

SPI_SLAVE_WRITE = 0x00
SPI_SLAVE_READ = 0x01

# MCP23S17-Pins
SPI_SCLK = 11 # Serial-Clock
SPI_MOSI = 10 # Master-Out-Slave-In
SPI_MISO = 9  # Master-In-Slave-Out
SPI_CS0 = 8   # Chip-Select
                   
LedPattern = ("0b00000001", "0b00000010", "0b00000100", "0b00001000", \
              "0b00010000", "0b00100000", "0b01000000", "0b10000000")

Addons = {
    "LNdigital": False,
    "USB_HUB": False,
    "IOboard": False,
    "StepMotor": False,
    "RTC": False,
    "LCD1602": False,
    "LCD1602MSG": False,
    "UltraSonic": False,
    "DHTreader": False,
    "PCF8591": False,
    "SoundDetect": False,
    "TouchSensor": False,
    "TiltSensor": False,
    "LightSensor": False,
}


#################################################################################


def help():
    print("Help function not finished.")
    time.sleep(2)
    sys.exit(0)


def usage():
    os.system("clear")
    print(sys.argv[0][:-3] + " Usage:\n")
    print(" -d , --debug    Enables the Debugging output.")
    print(" -h , --help     Will print out the help page.")
    print(" -u , --usage    Will print out this page")


def handler(signum, frame):
    SPIsend(SPI_SLAVE_ADDR, SPI_IOCONA, 0x00)
    SPIsend(SPI_SLAVE_ADDR, SPI_IOCONB, 0x00)
    SPIsend(SPI_SLAVE_ADDR, SPI_GPIOA, 0x00)
    GPIO.cleanup()
    os.system("clear")
    print("Clean")
    if Failed == 0:
        PSent = "Sent: {0}".format(Sent)
        PRecv = " | Received: {0}".format(Finished)
        PFail = " | Failed: {0}".format(Failed)
        PPerc = " | Failure: 0%"
    else:
        PSent = "Sent: {0}".format(Sent)
        PRecv = " | Received: {0}".format(Finished)
        PFail = " | Failed: {0}".format(Failed)
        PPerc = " | Failure: {0}%".format(str(100 / (Sent / Failed)))
    print(PSent + PRecv + PFail)
    sys.exit()


#################################################################################


def SPIsendValue(value):
    GPIO.setup(SPI_MOSI, GPIO.OUT)
    GPIO.setup(SPI_SCLK, GPIO.OUT)
    for i in range(8):
        if (value & 0x80):
            GPIO.output(SPI_MOSI, GPIO.HIGH)
        else:
            GPIO.output(SPI_MOSI, GPIO.LOW)
        GPIO.output(SPI_SCLK, GPIO.HIGH)
        GPIO.output(SPI_SCLK, GPIO.LOW)
        value <<= 1


def SPIsend(opcode, addr, data):
    GPIO.setup(SPI_CS0, GPIO.OUT)
    GPIO.output(SPI_CS0, GPIO.LOW)
    SPIsendValue(opcode | SPI_SLAVE_WRITE)
    SPIsendValue(addr)
    SPIsendValue(data)
    GPIO.output(SPI_CS0, GPIO.HIGH)


def SPIread(opcode, addr):
    GPIO.setup(SPI_MISO, GPIO.IN)
    GPIO.output(SPI_CS0, GPIO.LOW)
    SPIsendValue(opcode | SPI_SLAVE_READ)
    SPIsendValue(addr)
    value = 0
    for i in range(8):
        value <<= 1
        if(GPIO.input(SPI_MISO)):
            value |= 0x01
        GPIO.output(SPI_SCLK, GPIO.HIGH)
        GPIO.output(SPI_SCLK, GPIO.LOW)
    GPIO.output(SPI_CS0, GPIO.HIGH)
    return value

        
#################################################################################

def I2CButton():
    try:
        Bus.write_byte_data(0x21, 0x13, 0x00)
        Bus.write_byte_data(0x21, 0x13, 0x55)
        time.sleep(0.1)
        Read1 = Bus.read_byte_data(0x21, 0x13)
        if Read1 != 0x55:
            Addons["I2CButton"] = True
            Bus.write_byte_data(0x21, 0x01, 0xFF)   # Set all BankB to Inputs
            Bus.write_byte_data(0x21, 0x03, 0xFF)   # Invert Polarity of pins
            Bus.write_byte_data(0x21, 0x00, 0x00)   # Set all BankA to Outputs
        else:
            Addons["I2CButton"] = False
    except:
        Addons["I2CButton"] = False


def LNdigital():
    try:
        GPIO.setup(SPI_SCLK, GPIO.OUT)
        GPIO.setup(SPI_MOSI, GPIO.OUT)
        GPIO.setup(SPI_MISO, GPIO.IN)
        GPIO.setup(SPI_CS0, GPIO.OUT)

        GPIO.output(SPI_CS0, GPIO.HIGH)       
        GPIO.output(SPI_SCLK, GPIO.LOW)

        SPIsend(SPI_SLAVE_ADDR, SPI_IODIRA, 0x00)#outputs
        SPIsend(SPI_SLAVE_ADDR, SPI_IODIRB, 0xFF)#inputs
        SPIsend(SPI_SLAVE_ADDR, SPI_GPIOA, 0x00)
        SPIsend(SPI_SLAVE_ADDR, SPI_GPIOB, 0xFF)
        Test = SPIread(SPI_SLAVE_ADDR, SPI_GPIOA)
        if Test == 0x00:
            Addons["LNdigital"] = True
        else:
            Addons["LNdigital"] = False
    except:
        Addons["LNdigital"] = False
           

def Check():
    if not Connected:
        I2CButton()
        LNdigital()
        Finished = 0
        Sent = 0
        Failed = 0


def Connect():
    Check()
    print("Connecting...")
    global Socket
    try:
        Setup = False
        Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        Socket.connect((HOST, PORT))
        print("Connected!")
        Connected = True
        sendCmd("broadcast startup")
        return
    except:
        print("Scratch not up. Sleeping for 5 and trying again.")
        time.sleep(5)
        Connected = False
        Connect()


def List():
    for Key, Value in Addons.iteritems():
        print(Key)
  
        
##################################################################################


def I2CsetFunc():
    Address = [0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27]
    for Addr in Address:
        try:
            Bus.write_byte_data(Addr, 0x00, 0x00)
            Bus.write_byte_data(Addr, 0x01, 0x00)
            Bus.write_byte_data(Addr, 0x0A, 0x00)
        except:
            print (str(Addr) + " not availible.")


def sendCmd(cmd):
    n = len(cmd)
    a = array('c')
    a.append(chr((n >> 24) & 0xFF))
    a.append(chr((n >> 16) & 0xFF))
    a.append(chr((n >> 8) & 0xFF))
    a.append(chr(n & 0xFF))
    Socket.send(a.tostring() + cmd)


def LNout_update(Pin, status):
    if status == "ON":
        New = int(LedPattern[Pin], 2)
        Old = int('{:08}'.format(SPIread(SPI_SLAVE_ADDR, SPI_GPIOA)))
        Bin = bin(Old | New)
    elif status == "OFF":
        New = ~int(LedPattern[Pin], 2)
        Old = int('{:08}'.format(SPIread(SPI_SLAVE_ADDR, SPI_GPIOA)))
        Bin = bin(Old & New)

    SPIsend(SPI_SLAVE_ADDR, SPI_GPIOA, int(Bin, 2))


def calDistance(trigger, echo):
    stop = start = 0
    GPIO.setup(trigger, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)
    GPIO.output(trigger, 0)
    time.sleep(0.5)
    
    GPIO.output(trigger, 1)
    time.sleep(0.00001)
    GPIO.output(trigger, 0)
    start = time.time()
    while GPIO.input(echo) == 0:
        start = time.time()

    while GPIO.input(echo) == 1:
        stop = time.time()

    delta = stop - start
    print("UltraSonic time %.8f" % (delta))
    distance = delta * 34300
    distance = distance / 2.0
    GPIO.cleanup()
    return distance


def soundDetect(soundPin):
    LED = 9
    GPIO.setup(LED, GPIO.OUT)
    GPIO.output(LED, 0)  
    print("Sound detect on GPIO %d is %d" %(soundPin, GPIO.input(soundPin)))
    sendCmd("sensor-update soundDetect %d" % (GPIO.input(soundPin)))
    time.sleep(30)
    GPIO.output(LED, 1)


#################################################################################


if __name__ == '__main__':

    os.system("clear")
    Path = os.path.dirname(os.path.abspath(__file__))
    if os.getcwd() == Path:
        print ("Paths match, Continue.")
    else:
        os.chdir(Path)
        print ("Paths do not match, Changing Directory.")

    signal.signal(signal.SIGTSTP, handler)

    if len(sys.argv) != 1:
        if sys.argv[1] == "--debug" or sys.argv[1] == "-d":
            Debug = True
        elif sys.argv[1] == "--help" or sys.argv[1] == "-h":
            help()
            sys.exit()
        elif sys.argv[1] == "--usage" or sys.argv[1] == "-u":
            usage()
            sys.exit()
        else:
            usage()
            sys.exit()
    
    if not Connected:
        Connect()
    List()

    SPIsend(SPI_SLAVE_ADDR, SPI_IOCONA, 0x0A)
    SPIsend(SPI_SLAVE_ADDR, SPI_IOCONB, 0x0A)

    SPIsend(SPI_SLAVE_ADDR, SPI_IODIRA, 0x00)
    SPIsend(SPI_SLAVE_ADDR, SPI_IODIRB, 0xFF)
 
    SPIsend(SPI_SLAVE_ADDR, SPI_GPPUA, 0x00)
    SPIsend(SPI_SLAVE_ADDR, SPI_GPPUB, 0xFF)
    
    SPIsend(SPI_SLAVE_ADDR, SPI_INTCONA, 0x00)
    SPIsend(SPI_SLAVE_ADDR, SPI_INTCONB, 0xFF)

    SPIsend(SPI_SLAVE_ADDR, SPI_DEFVALA, 0x00)
    SPIsend(SPI_SLAVE_ADDR, SPI_DEFVALB, 0xFF)

    SPIsend(SPI_SLAVE_ADDR, SPI_GPIOA, 0x00)
    SPIsend(SPI_SLAVE_ADDR, SPI_GPIOB, 0x00)

    SPIsend(SPI_SLAVE_ADDR, SPI_INTENA, 0x00)
    SPIsend(SPI_SLAVE_ADDR, SPI_INTENB, 0x00)
      
    SPIsend(SPI_SLAVE_ADDR, SPI_OLATA, 0x00)
    SPIsend(SPI_SLAVE_ADDR, SPI_OLATB, 0xFF)

    address = 0x48
    IOboard.init(address)
                        
    LastMsg = time.time()

    while True:
        try:
            if not Setup:
                try:
                    print("")
                    Bus.write_byte_data(0x21, 0x00, 0xFF)
                    Bus.write_byte_data(0x21, 0x01, 0xFF)
                    Bus.write_byte_data(0x21, 0x12, 0x55)
                    Bus.write_byte_data(0x21, 0x12, 0x00)

                    Bus.write_byte_data(0x22, 0x00, 0xFF)
                    Bus.write_byte_data(0x22, 0x01, 0xFF)
                    Bus.write_byte_data(0x22, 0x12, 0x55)
                    Bus.write_byte_data(0x22, 0x12, 0x00)

                    Bus.write_byte_data(0x23, 0x00, 0xFF)
                    Bus.write_byte_data(0x23, 0x01, 0xFF)
                    Bus.write_byte_data(0x23, 0x12, 0x55)
                    Bus.write_byte_data(0x23, 0x12, 0x00)

                    Bus.write_byte_data(0x20, 0x00, 0xFF)
                    Bus.write_byte_data(0x20, 0x01, 0xFF)
                    Bus.write_byte_data(0x20, 0x12, 0x55)
                    Bus.write_byte_data(0x20, 0x12, 0x00)
                except:
                    if Debug:
                        print("I2C error")

                Setup = True


            Data = Socket.recv(1024)
            if not Data:
                if (time.time() - LastMsg) > 30:
                    Socket.close()
                    time.sleep(1)
                    Connect()
                    print ("Reconnected.")
                time.sleep(0.5)
            
            if Data:
                LastMsg = time.time()
                Data = Data.replace('"', ' ')
                Data = Data.replace("'", ' ')
                Data = Data.replace('BROADCAST', '')
                Data = Data.replace('broadcast', '')
                Data = Data[4:]

                Data = filter(lambda x: x in string.printable, Data)
                if Debug:
                    print ("Data> " + Data)

                Data = Data.upper()
                Data = Data.split(' ')
                Length = len(Data)

                if not Data:
                    Data = ["False"*2]

                for A in range(len(Data)):
   
                    if Data[A] == "RTC" and not Addons["RTC"]:
                        Addons["RTC"] = True
                        print("Real Time Clock is added.")

                    if Data[A] == "ULTRASONIC" and not Addons["UltraSonic"]:
                        Addons["UltraSonic"] = True
                        print("UltraSonic device is added.")

                    if Data[A] == "PCF8591" and not Addons["PCF8591"]:
                        Addons["PCF8591"] = True
                        print("PCF8591 A/D D/A converter is added.")

                    if Data[A] == "LCD1602" and not Addons["LCD1602"]:
                        Addons["LCD1602"] = True
                        LCDhandler = IOboard.initLCD(2, 16, 8)
                        IOboard.clearLCD(LCDhandler)
                        print("LCD1602 to display is added.")
                        
                    if Data[A] == "LCD1602MSG" and not Addons["LCD1602MSG"]:
                        Addons["LCD1602MSG"] = True
                        LCDhandler = IOboard.initLCD(2, 16, 8)
                        IOboard.clearLCD(LCDhandler)
                        print("LCD1602 can now show messages.")

     
                    if Data[A] == "UPDATE":

                        for B in SPInputs:
                            Chip = int(B[0], 16)
                            Bank = B[1]
                            if Bank == 18:
                                Reg = 0x04
                                Bank1 = 0x12
                                Bank2 = "A"
                            elif Bank == 19:
                                Reg = 0x05
                                Bank1 = 0x13
                                Bank2 = "B"
                            SPIsend(Chip, Reg, 0xFF)
                            Read = SPIread(Chip, Bank1)
                            Read = Read ^ 0xFF
                            Read = '{:08b}'.format(Read)[::-1]         
                            Chip = (int(Chip) / 2) - 32
                            if Debug:
                                print (Chip, Bank, Read)
                            for A in range(len(Read)):
                                RdA = Read[A]
                                Out = "{0}{1}-{2} {3}".format(Chip, Bank2, A, RdA)
                                sendCmd("sensor-update SPI{0}".format(Out))
                            
                        for B in I2Inputs:
                            Addr = int(B[0], 16)
                            Reg = B[1]
                            if Reg == 9:
                                Bank = "A"
                            elif Reg == 19:
                                Bank = "B"                                   
                            Bus.write_byte_data(Addr, Reg, 0xFF)
                            Read = Bus.read_byte_data(Addr, Reg)
                            Read = '{:08b}'.format(Read)[::-1]
                            Chip = Addr - 32
                            if Debug:
                                print (Chip, Bank, Read)
                            for A in range(len(Read)):
                                TRead = Read[A]
                                Out = "{0}{1}-{2} {3}".format(Chip, Bank, A, TRead)
                                sendCmd("sensor-update I2C{0}".format(Out))
                        
                        try:
                            for B in range(len(LNin)):
                                if LNin[B]:
                                    SPIsend(SPI_SLAVE_ADDR, SPI_INTENB, 0xFF)
                                    New = SPIread(SPI_SLAVE_ADDR, SPI_GPIOB)
                                    if Debug:
                                        print("Data Read", New)
                                    Led = New ^ 0xFF
                                    if Debug:
                                        print("Data XOR", Led)
                                    Led = '{:08b}'.format(Led)[::-1]
                                    if Debug:
                                        print("Led send", Led)
                                    Input = int(Led[B])
                                    print ("LNDI-in-{0} -> {1}".format(B, Input))
                                    sendCmd("sensor-update LNDI-in-{0} {1}".format(B, Input))
                        except:
                            print("Can't update LNinput")

                        try:
                            for i in range(len(GPIOset)):
                                if GPIOset[i]:
                                    Dir = GPIOdir[i]
                                    if Dir == "IN":
                                        Input = GPIO.input(i)
                                        print ("Pin: {0} -> {1}".format(i, Input))
                                        sendCmd("sensor-update GPIO-{0} {1}".format(i, Input))
                        except:
                            print("Failed")

                        if Addons["RTC"]:
                            DS1307 = RTC_DS1307.RTC_DS1307(2, 0x68)
                            DS1307.write_now()
                            currenttime = datetime.datetime.utcnow()
                            print("DS1307 = \t\t%s" % DS1307.read_datetime())
                            print("System Time = \t" + time.strftime("%Y-%m-%d %H:%M:%S"))
    
                            sendCmd("sensor-update Day " + str(DS1307._read_day()))
                            sendCmd("sensor-update Date " + str(DS1307._read_date()))
                            sendCmd("sensor-update Month " + str(DS1307._read_month()))
                            sendCmd("sensor-update Year " + str(DS1307._read_year()))
                            sendCmd("sensor-update Hour " + str(DS1307._read_hours()))
                            sendCmd("sensor-update Minutes " + str(DS1307._read_minutes()))
                            sendCmd("sensor-update Seconds " + str(DS1307._read_seconds()))

                        if Addons["LCD1602"]:                        
                            IOboard.clearLCD(LCDhandler)
                            msgL1 = "20" + str(DS1307._read_year()) + "-" + str(DS1307._read_month()) + "-" + str(DS1307._read_date())
                            msgL2 = str(DS1307._read_hours()) + ":" + str(DS1307._read_minutes()) + ":" + str(DS1307._read_seconds())
                            IOboard.writeLCD(LCDhandler, 0, 0, msgL1)
                            IOboard.writeLCD(LCDhandler, 0, 1, msgL2)
                            msgL3 = str(int(temperature)) + " 'C"
                            msgL4 = str(int(humidity)) + " RH"                       
                            IOboard.writeLCD(LCDhandler, 11, 0, msgL3)
                            IOboard.writeLCD(LCDhandler, 11, 1, msgL4)
                            
                        if Addons["UltraSonic"]:
                            distance = calDistance(GPIO_Trigger, GPIO_Echo)
                            print("The distance measurement of UltraSonic: {0}".format(distance))
                            sendCmd("sensor-update UltraSonic %.2f" % (distance))

                        if Addons["DHTreader"]:
                            try:
                                temperature, humidity = DHTreader.read(typeDHT, pinDHT)
                                if temperature and humidity:
                                    print("Temperature = {0} *C, Humidity = {1} %".format(temperature, humidity))
                                    sendCmd("sensor-update Temperature %.2f" % (temperature))
                                    sendCmd("sensor-update Humidity %.2f" % (humidity))                                       
                                else:
                                    print("Failed to read from sensor, restart.")
                            except:
                                print("Failed to read from sensor, restart.")
                                
                        if Addons["TouchSensor"]:
                            GPIO.setup(touchPin, GPIO.IN, GPIO.PUD_DOWN)
                            print("Touch detect on GPIO %d is %d" %(touchPin, GPIO.input(touchPin)))
                            sendCmd("sensor-update touchSensor %d" % (GPIO.input(touchPin)))

                        if Addons["TiltSensor"]:
                            GPIO.setup(tiltPin, GPIO.IN, GPIO.PUD_DOWN)
                            print("Tilt detect on GPIO %d is %d" %(tiltPin, GPIO.input(tiltPin)))
                            sendCmd("sensor-update tiltSensor %d" % (GPIO.input(tiltPin)))                            

                        if Addons["LightSensor"]:
                            GPIO.setup(lightPin, GPIO.IN, GPIO.PUD_DOWN)
                            print("Light detect on GPIO %d is %d" %(lightPin, GPIO.input(lightPin)))
                            sendCmd("sensor-update lightSensor %d" % (GPIO.input(lightPin)))
                            
                if Data[0] == "SENSOR-UPDATE":
                    Data[0] = ""
                for A in range(len(Data)):
                    Spi = re.findall(r'SP(\d)(\D)(\d)', Data[A])
                    SpiIn = re.findall(r'SP(\d)(A|B)IN', Data[A])                    
                    Bits = re.findall(r'BITS(\d)(\D)(\d+)', Data[A])
                    Bits2 = re.findall(r'BITS(\d)(\D)(ON|OFF|CLR)', Data[A])      
                    I2C = re.findall(r'I2(\d+)(\D+)(\d+)', Data[A])
                    I2cIn = re.findall(r'I2(\d)+(A|B)IN', Data[A])
                    Bit = re.findall(r'BIT(\d+)(\D+)(\d+)', Data[A])
                    Bit2 = re.findall(r'BIT(\d+)(\D+)(ON|OFF|CLR)', Data[A])
                    Data2 = re.findall(r'G(\d+)(\D+)', Data[A])
                    LNDI = re.findall(r'LNDI(\d+)(\D+)', Data[A])
                    StepM1 = re.findall(r'^STEPMAINIT', Data[A])
                    StepM2 = re.findall(r'^STEPMBINIT', Data[A])
                    StepMode = re.findall(r'^STEPMODE(\d+)([A|B])', Data[A])                    
                    StepM = re.findall(r'STEPM(\d+)([A|B])(\d+)([P|N])', Data[A])
                    Trigger = re.findall(r'TRIGGER(\d+)', Data[A])
                    Echo = re.findall(r'ECHO(\d+)', Data[A])
                    DHT = re.findall(r'DHT(\d+)PIN(\d+)', Data[A])
                    AD = re.findall(r'AD(\d+)READ', Data[A])
                    DA = re.findall(r'DA(\d+)WRITE', Data[A])
                    Sound = re.findall(r'SOUND(\d+)', Data[A])
                    Touch = re.findall(r'TOUCH(\d+)', Data[A])
                    Tilt = re.findall(r'TILT(\d+)', Data[A])
                    Light = re.findall(r'LIGHT(\d+)AIN(\d+)', Data[A])  
                    Message = re.findall(r'MSG1602(\D+)', Data[A])
                    
                    if Message and Addons["LCD1602MSG"]:
                        Message = Message[0]
                        lenMSG = len(Message)                        
                        print(Message)
                        IOboard.clearLCD(LCDhandler)
                        if lenMSG < 16:
                            IOboard.writeLCD(LCDhandler, 0, 0, Message)
                        elif lenMSG < 32:
                            IOboard.writeLCD(LCDhandler, 0, 0, Message)
                            IOboard.writeLCD(LCDhandler, 0, 1, Message[16:lenMSG])
                        else:
                            print("LCD can't display more than 32 characters.")
        
                    if Spi:
                        Spi = Spi[0]
                        Chip = int(Spi[0])
                        Bank = Spi[1]
                        Led = Spi[2]
                        Addr = "0x" + str((Chip * 2) + 40)

                        if Chip == 5:
                            Addr = 0x4A
                        elif Chip == 6:
                            Addr = 0x4C
                        elif Chip == 7:
                            Addr = 0x4E
                            
                        Reg = Bank.upper()
                        if Reg == "A":
                            Reg = 0x12
                        elif Reg == "B":
                            Reg = 0x13
                        else:
                            Reg = 0x00
                            print("Error. Only A or B. Try Again.")

                        Led = int(Spi[2])
                        Bin = int(LedPattern[Led], 2)
                        SPIsend(int(Addr, 16), Reg, Bin)
                
                    if SpiIn:
                        In = SpiIn[0]
                        Chip = In[0]
                        Bank = In[1]
                        Hex = hex((int(Chip) * 2) + 64)
                        if Bank == "A":
                            Bank = 0x12
                        elif Bank == "B":
                            Bank = 0x13
                        else:
                            Bank = 0x00
                        if (Hex, Bank) not in SPInputs:
                            print("Added :", Hex, Bank)
                            SPInputs.append((Hex, Bank))

                    if Bits:
                        Bits = Bits[0]
                        Chip = int(Bits[0])
                        Bank = Bits[1]
                        Bin = Bits[2]
                        Addr = "0x" + str((Chip * 2) + 40)
                        if Chip == 5:
                            Addr = "0x4A"
                        elif Chip == 6:
                            Addr = "0x4C"
                        elif Chip == 7:
                            Addr = "0x4E"

                        if Bank.upper() == "A":
                            Bank = 0x12
                        elif Bank.upper() == "B":
                            Bank = 0x13
                        else:
                            Bank = 0x00
                        Bin = "0b" + Bin
                        if Debug:
                            print(Addr, Bank, Bin)
                        SPIsend(int(Addr, 16), Bank, int(Bin, 2))
                            
                    if Bits2:
                        Bits2 = Bits2[0]
                        Chip = int(Bits2[0])
                        Bank = Bits2[1]
                        Cmd = Bits2[2]
                        print (Chip, Bank, Cmd)
                        Addr = "0x" + str((Chip*2) + 40)
                        if Chip == 6:
                            Addr = "0x4A"
                        elif Chip == 7:
                            Addr = "0x4C"
                        elif Chip == 8:
                            Addr = "0x4E"

                        if Bank.upper() == "A":
                            Bank = 0x12
                        elif Bank.upper() == "B":
                            Bank = 0x13
                        else:
                            Bank = 0x00

                        if Cmd.upper() == "OFF" or Cmd.upper() == "CLR":
                            Bin = 0x00
                        elif Cmd.upper() == "ON":
                            Bin = 0xFF

                        SPIsend(int(Addr, 16), Bank, Bin)

                    if I2C:
                        if not I2Csetup:
                            I2CsetFunc()
                            I2Csetup = True
    
                        Addr = I2C[0][0]
                        Bank = I2C[0][1]
                        Led = int(I2C[0][2])
                        if Bank == "A":
                            Reg = 0x12
                        elif Bank == "B":
                            Reg = 0x13
                            
                        Bin = int(LedPattern[Led], 2)                        
                        Bus = smbus.SMBus(2)
                        Read = Bus.read_byte_data(int("0x"+Addr, 16), Reg)
                        if Debug:
                            print (Read)
                        Bus.write_byte_data(int("0x"+Addr, 16), Reg, Read ^ Bin)

                    if I2cIn:
                        In = I2cIn[0]
                        Chip = hex(int(In[0]) + 32)
                        Bank = In[1]
                        if Bank == "A":
                            Bank = 0x12
                        elif Bank == "B":
                            Bank = 0x13
                        print (In, Chip, Bank)
                        if (Chip, Bank) in I2Inputs:
                            print("Already in Inputs.")
                        else:
                            I2Inputs.append((Chip, Bank))

                    if Bit:
                        Addr = Bit[0][0]
                        Bank = Bit[0][1]
                        Binary = Bit[0][2]
                        Binary = int(Binary, 2)
                        Bus = smbus.SMBus(2)
                        if Debug:
                            print (Bit)
                            print (Bank, Binary)
                        if Bank.upper() == "A":
                            Bus.write_byte_data(int("0x"+Addr, 16), 0x12, Binary)
                            Bus.write_byte_data(int("0x"+Addr, 16), 0x09, Binary)
                        elif Bank.upper() == "B":
                            Bus.write_byte_data(int("0x"+Addr, 16), 0x13, Binary)
                            Bus.write_byte_data(int("0x"+Addr, 16), 0x19, Binary)
                        else:
                            if Debug:
                                print("Incorrect Bank enter in Bit command.")
                                
                    if Bit2:
                        Addr = Bit2[0][0]
                        Bank = Bit2[0][1]
                        Cmd = Bit2[0][2]
                        Bus = smbus.SMBus(2)
                        Bit = False
                        if Cmd == "ON":
                            if Bank.upper() == "A":
                                Bus.write_byte_data(int("0x"+Addr, 16), 0x12, 0xFF)
                                Bus.write_byte_data(int("0x"+Addr, 16), 0x09, 0xFF)
                            elif Bank.upper() == "B":
                                Bus.write_byte_data(int("0x"+Addr, 16), 0x13, 0xFF)
                                Bus.write_byte_data(int("0x"+Addr, 16), 0x19, 0xFF)
                        if Cmd == "OFF" or Cmd == "CLR":
                            if Bank.upper() == "A":
                                Bus.write_byte_data(int("0x"+Addr, 16), 0x12, 0x00)
                                Bus.write_byte_data(int("0x"+Addr, 16), 0x09, 0x00)
                            elif Bank.upper() == "B":
                                Bus.write_byte_data(int("0x"+Addr, 16), 0x13, 0x00)
                                Bus.write_byte_data(int("0x"+Addr, 16), 0x19, 0x00)         

                    if Data2:
                        Data2 = Data2[0]
                        Pin = int(Data2[0])
                        Cmd = Data2[1].upper()
                        if not Pin in [4, 5, 6, 12, 13, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27]:
                            break
                        if Cmd == "OUT":
                            GPIOset[Pin] = True
                            GPIOdir[Pin] = "OUT"
                            GPIO.setup(Pin, GPIO.OUT)
                            print(str(Pin) + " set to OUT")
                        if Cmd == "IN":
                            GPIOset[Pin] = True
                            GPIOdir[Pin] = "IN"
                            GPIO.setup(Pin, GPIO.IN)
                            print(str(Pin) + " set to IN")
                        if Cmd == "ON" and GPIOset[Pin]:
                            GPIO.setup(Pin, GPIO.OUT)
                            GPIO.output(Pin, 1)
                            print(str(Pin) + " set to ON")
                        if Cmd == "OFF" and GPIOset[Pin]:
                            GPIO.setup(Pin, GPIO.OUT)
                            GPIO.output(Pin, 0)
                            print(str(Pin) + " set to OFF")

                    if LNDI:                    
                        LNDI = LNDI[0]
                        Pin = int(LNDI[0])
                        Cmd = LNDI[1].upper()
                        if not Pin in range(0,8):
                            break
                        if Cmd == "OUT":
                            LNout[Pin] = True
                            print("Listen to OUTPUT " + str(Pin))
                        if Cmd == "IN":
                            LNin[Pin] = True
                            print("Listen to INPUT " + str(Pin))
                        if Cmd == "ON" and LNout[Pin]:
                            LNout_update(Pin, "ON")
                            print("LNDI-" + str(Pin) + " set to ON")
                        if Cmd == "OFF" and LNout[Pin]:
                            LNout_update(Pin, "OFF")
                            print("LNDI-" + str(Pin) + " set to OFF")

                    if StepM1:
                        GPIO.setup(11, GPIO.OUT)
                        GPIO.setup(7, GPIO.OUT)
                        GPIO.setup(16, GPIO.OUT)
                        GPIO.setup(26, GPIO.OUT)
                        stepM1 = StepMotor.Motor(11,7,16,26)
                        stepM1.init()

                    if StepM2:
                        GPIO.setup(4, GPIO.OUT)
                        GPIO.setup(25, GPIO.OUT)
                        GPIO.setup(24, GPIO.OUT)
                        GPIO.setup(23, GPIO.OUT)
                        stepM2 = StepMotor.Motor(4,25,24,23)
                        stepM2.init()

                    if StepMode:
                        In = StepMode[0]
                        mode = int(In[0])
                        Index = In[1]
                        Steps = 256
                        if Index == "A":
                            if mode == 1:
                                stepM1.setFullStepDrive()
                                print("Step Motor A: Full Step Drive")
                            elif mode == 2:
                                stepM1.setWaveDrive()
                                print("Step Motor A: Wave Drive")
                                
                            stepM1.turn(Steps, stepM1.CLOCKWISE)
                            time.sleep(0.1)
                            stepM1.turn(Steps, stepM1.ANTICLOCKWISE)
                            time.sleep(0.1)
                            
                        elif Index == "B":
                            if mode == 1:
                                stepM2.setFullStepDrive()
                                print("Step Motor B: Full Step Drive")                               
                            elif mode == 2:
                                stepM2.setWaveDrive()
                                print("Step Motor B: Wave Drive")

                            stepM2.turn(Steps, stepM1.CLOCKWISE)
                            time.sleep(0.1)
                            stepM2.turn(Steps, stepM1.ANTICLOCKWISE)
                            time.sleep(0.1)
                                                              
                    if StepM:
                        In = StepM[0]
                        Speed = int(In[0])
                        Speed = Speed / 1000.0
                        stepM1.setSpeed(Speed)
                        stepM2.setSpeed(Speed)
                        Index = In[1]
                        Steps = int(In[2])
                        Dir = In[3]
                        if Index == "A":                            
                            if Dir == "P":
                                stepM1.turn(Steps, stepM1.CLOCKWISE)
                                time.sleep(0.1)
                            elif Dir == "N":
                                stepM1.turn(Steps, stepM1.ANTICLOCKWISE)
                                time.sleep(0.1)
                        elif Index == "B":
                            if Dir == "P":
                                stepM2.turn(Steps, stepM2.CLOCKWISE)
                                time.sleep(0.1)
                            elif Dir == "N":
                                stepM2.turn(Steps, stepM2.ANTICLOCKWISE)
                                time.sleep(0.1)

                    if Trigger:
                        Trigger = int(Trigger[0])
                        GPIO_Trigger = Trigger
                        GPIO.setup(GPIO_Trigger, GPIO.OUT)
                        print("GPIO {0} is set to trigger".format(Trigger))

                    if Echo:
                        Echo = int(Echo[0])
                        GPIO_Echo = Echo
                        GPIO.setup(GPIO_Echo, GPIO.IN)
                        print("GPIO {0} is set to echo".format(Echo))

                    if DHT:
                        DHT = DHT[0]
                        typeDHT = int(DHT[0])
                        pinDHT = int(DHT[1])

                        if typeDHT not in [11, 22, 2302]:
                            print("invalid type, only DHT11, DHT22 and AM2302 are supported.")
                            sys.exit(3)

                        if pinDHT <= 0 | pinDHT > 40:
                            print("invalid GPIO pin#")
                            sys.exit(3)

                        print("Detect on wiringPi PIN #{0} for DHT sensor".format(pinDHT))
                        
                        DHTreader.init()
                        Addons["DHTreader"] = True

                    if AD and Addons["PCF8591"]:
                        channel = int(AD[0])
                        if channel in [0, 1, 2, 3]:
                            value = IOboard.readPCF8591(channel)
                            inV = 3.3*value/255
                            print("I2C PCF8591: channel {0} input: {1} voltage.".format(channel, inV))
                            sendCmd("sensor-update readPCF8591 %.2f" % (inV))
                        else:
                            print("I2C PCF8591 should read from channel [0 | 1 | 2 | 3]")
                                        
                    if DA and Addons["PCF8591"]:
                        out = int(DA[0])
                        IOboard.writePCF8591(out)
                        outV = 3.3*out/255
                        print("I2C PCF8591: analog output: {0} voltage.".format(outV))
                        sendCmd("sensor-update writePCF8591 %.2f" % (outV))

                    if Sound: 
                        soundPin = int(Sound[0])
                        try:
                            GPIO.setup(soundPin, GPIO.IN, GPIO.PUD_DOWN)
                            print("Sound detect on GPIO %d is %d" %(soundPin, GPIO.input(soundPin)))
                            GPIO.add_event_detect(soundPin,GPIO.FALLING,callback = soundDetect,bouncetime = 300)
                        except:
                            print("Failed to setup the Sound sensor")

                    if Touch:
                        touchPin = int(Touch[0])
                        try:
                            GPIO.setup(touchPin, GPIO.IN, GPIO.PUD_DOWN)
                            print("Touch detect on GPIO %d is %d" %(touchPin, GPIO.input(touchPin)))
                            Addons["TouchSensor"] = True
                        except:
                            print("Failed to setup the Touch sensor")

                    if Tilt:
                        tiltPin = int(Tilt[0])
                        try:
                            GPIO.setup(tiltPin, GPIO.IN, GPIO.PUD_DOWN)
                            print("Tilt detect on GPIO %d is %d" %(tiltPin, GPIO.input(tiltPin)))
                            Addons["TiltSensor"] = True
                        except:
                            print("Failed to setup the Tilt sensor")

                    if Light:
                        Light = Light[0]
                        lightPin = int(Light[0])
                        lightAIN = int(Light[1])
                        try:
                            GPIO.setup(lightPin, GPIO.IN, GPIO.PUD_DOWN)
                            print("Light detect on GPIO %d is %d" %(lightPin, GPIO.input(lightPin)))
                            Addons["LightSensor"] = True
                        except:
                            print("Failed to setup the Light sensor")

                                                                                                                
        except KeyboardInterrupt:
            SPIsend(SPI_SLAVE_ADDR, SPI_IOCONA, 0x00)
            SPIsend(SPI_SLAVE_ADDR, SPI_IOCONB, 0x00)
            SPIsend(SPI_SLAVE_ADDR, SPI_GPIOA, 0x00)
            
            print("Ctrl+C, ending main thread")
            GPIO.cleanup()
            os.system("clear")
            print ("GPIO Cleaned up and exiting.")
            sys.exit()
