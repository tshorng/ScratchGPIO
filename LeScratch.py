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
from Tkinter import Tk
from array import array
import RPi.GPIO as GPIO


root = Tk()
root.withdraw()

Bus = smbus.SMBus(2)
PORT = 42001
HOST = "localhost"
Sent = 0
Finished = 0
Failed = 0
Debug = False

Connected = False
Setup = False
I2Csetup = False

I2Inputs = []
SPInputs = []
GPIOset = [False] * 35
GPIOdir = ["OUT"] * 35
LNout = [False] * 8
LNin = [False] * 8

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

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(True)
                   
LedPattern = ("0b00000001", "0b00000010", "0b00000100", "0b00001000", \
              "0b00010000", "0b00100000", "0b01000000", "0b10000000")

Addons = {
    "RTC": False,
    "0x20": False,
    "0x21": False,
    "0x22": False,
    "I2CButton": False,
    "LNdigital": False,
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

        
##################################################################################


def RtcTest():
    try:
        Bus.read_byte_data(0x68, 0x00)# To wake it up if is sleeping.
        Read = Bus.read_byte_data(0x68, 0x00)
        if Read != 0x00:
            Addons["RTC"] = True
    except:
        Addons["RTC"] = False


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
        RtcTest()
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
  

def I2CsetFunc():
    Address = [0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27]
    for Addr in Address:
        try:
            Bus.write_byte_data(Addr, 0x00, 0x00)
            Bus.write_byte_data(Addr, 0x01, 0x00)
            Bus.write_byte_data(Addr, 0x0A, 0x00)
        except:
            print (str(Addr) + " not availible.")

            
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
   
                    if Data[A] == "RTC":
                        Addons["RTC"] = True
                        os.system("clear")
                        List()                                

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
                                                                           

                if Data[0] == "SENSOR-UPDATE":
                    Data[0] = ""
                for A in range(len(Data)):
                    PinsOff = re.findall(r'PINSOFF', Data[A])
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

                    if PinsOff:
                        Pins = [
                        4, 7, 8, 9, 10, 11, 17, 18, 22,
                        23, 24, 25, 27, 28, 29, 30, 31
                        ]
                        for A in Pins:
                            try:
                                GPIO.output(A, 0)
                            except:
                                continue

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
                            GPIO.output(Pin, 1)
                            print(str(Pin) + " set to ON")
                        if Cmd == "OFF" and GPIOset[Pin]:
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
                                                    		    
        except KeyboardInterrupt:
            SPIsend(SPI_SLAVE_ADDR, SPI_IOCONA, 0x00)
            SPIsend(SPI_SLAVE_ADDR, SPI_IOCONB, 0x00)
            SPIsend(SPI_SLAVE_ADDR, SPI_GPIOA, 0x00)
            
            print("Ctrl+C, ending main thread")
            GPIO.cleanup()
            os.system("clear")
            print ("GPIO Cleaned up and exiting.")
            Sent = "Sent: {0}".format(Sent)
            Fin = " Finished: {0}".format(Finished)
            Failed = " Failed: {0}".format(Failed)
            if Failed == 0:
                print(Sent + Fin + Failed)
            sys.exit()
