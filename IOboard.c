/****  Python module for LCD, Data Flash, A/D D/A converters on IOboard

gcc -shared -I/usr/include/python2.7/ -lpython2.7 -o IOboard.so IOboard.c -I/usr/local/include -L/usr/local/lib -lwiringPi -lwiringPiDev

 ****/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <Python.h>
#include <errno.h>
#include <wiringPi.h>
#include <lcd.h>


#define I2C_BASE 100

int readPCF8591(int channel)
{
	int value = analogRead(I2C_BASE + channel);
	printf("I2C PCF8591: channel %d input: %d per 255 * Standard. \n", channel, value);
	return value;
}

int writePCF8591(int out)
{
	analogWrite(I2C_BASE, out);
	printf("I2C PCF8591: analog output: %d per 255 * Standard. \n", out);
}

unsigned char readAT45Buff(void)
{
	int ret;
	unsigned char rBuf[6] = {0xD4, 0xff, 0x00, 0x02, 0xff, 0xff}; //read data from buffer 1

	ret = wiringPiSPIDataRW(0, rBuf, sizeof(rBuf));
	if(ret < 0)
	{
		printf("Read data from the AT45DB041D failed!\n");
		return;
	}
	return rBuf[5];
}

void writeAT45Buff(unsigned char val)
{
	int ret;
	unsigned char wBuf[5] = {0x84, 0xff, 0x00, 0x02,0xff}; //write data to buffer 1

	wBuf[4] = val;

	printf("SPI: write data: 0x%02x\n", wBuf[4]);

	ret = wiringPiSPIDataRW(0, wBuf, sizeof(wBuf));
	if(ret < 0)
	{
		printf("Write data to the AT45DB041D failed!\n");
		return;
	}
}

int initLCD(int rows, int cols, int bits)
{
	int LCDhandler;
	LCDhandler = lcdInit (rows, cols, bits,21 ,23 ,0,1,2,3,4,5,6,7) ;
	if(LCDhandler < 0)
	{
		printf("LCD initialization failed!\n");
		return;
	}
        lcdPosition (LCDhandler, 0, 0) ; lcdPuts (LCDhandler, "Hello World") ;
        return LCDhandler;
}

void writeLCD(int LCDhandler, int x, int y, char *argv)
{
	lcdPosition (LCDhandler, x, y) ; 
	lcdPuts (LCDhandler, argv) ;
}

static PyObject *
IOboard_init(PyObject *self, PyObject *args)
{
	int address;
	if (!PyArg_ParseTuple(args, "i", &address))
		return NULL;

	int fd = wiringPiSPISetup(0, 5000000); //channel:0  5M
	if(fd < 0)
	{
		printf("Open the SPI device failed!\n");
		return NULL;
	}

	pcf8591Setup(I2C_BASE, address);
	return Py_BuildValue("i", wiringPiSetup());
}

static PyObject *
IOBoard_readPCF8591(PyObject *self, PyObject *args)
{
	int channel, value;
	if (!PyArg_ParseTuple(args, "i", &channel))
		return NULL;
	value = readPCF8591(channel);
	return Py_BuildValue("i", value);
}

static PyObject *
IOBoard_writePCF8591(PyObject *self, PyObject *args)
{
	int out;
	if (!PyArg_ParseTuple(args, "i", &out))
		return NULL;
	writePCF8591(out);
	return Py_BuildValue("");
}

static PyObject *
IOBoard_readAT45Buff(PyObject *self, PyObject *args)
{
	int value = readAT45Buff();
	return Py_BuildValue("i", value);
}

static PyObject *
IOBoard_writeAT45Buff(PyObject *self, PyObject *args)
{
	int out;
	if (!PyArg_ParseTuple(args, "i", &out))
		return NULL;
	writeAT45Buff(out);
	return Py_BuildValue("");
}

static PyObject *
IOBoard_initLCD(PyObject *self, PyObject *args)
{
	int rows, cols, bits;
	if (!PyArg_ParseTuple(args, "iii", &rows, &cols, &bits))
		return NULL;
	int LCDhandler;
	LCDhandler = initLCD(rows, cols, bits);
	return Py_BuildValue("i", LCDhandler);
}

static PyObject *
IOBoard_writeLCD(PyObject *self, PyObject *args)
{
	int LCDhandler, x, y;
        char *msg;
	if (!PyArg_ParseTuple(args, "iiis", &LCDhandler, &x, &y, &msg))
		return NULL;
	writeLCD(LCDhandler, x, y, msg);
	return Py_BuildValue("");
}

static PyObject *
IOBoard_clearLCD(PyObject *self, PyObject *args)
{
	int LCDhandler;
	if (!PyArg_ParseTuple(args, "i", &LCDhandler))
		return NULL;
	lcdClear(LCDhandler);
	return Py_BuildValue("");
}

static PyMethodDef IOboardMethods[] = {
	{"init", IOboard_init, METH_VARARGS,
	 "initialize IOBoard wiringPi setups"},
	{"readPCF8591", IOBoard_readPCF8591, METH_VARARGS,
	 "read PCF8591 Analog input to Digital value"},
	{"writePCF8591", IOBoard_writePCF8591, METH_VARARGS,
	 "write PCF8591 Digital value to Analog output"},
	{"readAT45Buff", IOBoard_readAT45Buff, METH_VARARGS,
	 "read AT45Buff Data Flash"},
	{"writeAT45Buff", IOBoard_writeAT45Buff, METH_VARARGS,
	 "write AT45Buff Data Flash"},
	{"initLCD", IOBoard_initLCD, METH_VARARGS,
	 "initialize LCD1602 to display Data"},	 
	{"writeLCD", IOBoard_writeLCD, METH_VARARGS,
	 "write data to LCD1602"},	
	{"clearLCD", IOBoard_clearLCD, METH_VARARGS,
	 "clear data on screen LCD1602"},	
	 {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC
initIOboard(void)
{
    PyObject *m;

    m = Py_InitModule("IOboard", IOboardMethods);
    if (m == NULL)
        return;
}
