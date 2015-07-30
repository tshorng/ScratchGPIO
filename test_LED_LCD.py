import IOboard

address = 0x68
IOboard.init(address)
LCDhandler = IOboard.initLCD(2, 16, 8)
IOboard.writeLCD(LCDhandler, 0, 0, "Hello World")