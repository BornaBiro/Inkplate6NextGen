/*************************************************** 
This library is used for controling ED060SC7 epaper panel on e-radionica's Inkplate6NextGen dev board (we are still working on it!).
If you don't know what Inkplate is, check it out here: https://inkplate.io/

Author: Borna Biro ( https://github.com/BornaBiro/ )
Organization: e-radionica.com / TAVU

This library uses Adafruit GFX library (https://github.com/adafruit/Adafruit-GFX-Library) made by Adafruit Industries.

NOTE: This library is still heavily in progress, so there is still some bugs. Use it on your own risk!
 ****************************************************/

#include <stdlib.h>

#include "Adafruit_GFX.h"
#include "Inkplate6NextGen.h"
//SPIClass spi2(HSPI);
SdFat sd(&SPI);


//--------------------------USER FUNCTIONS--------------------------------------------
Inkplate::Inkplate(uint8_t _mode) : Adafruit_GFX(E_INK_WIDTH, E_INK_HEIGHT)
{
    _displayMode = _mode;
}

void Inkplate::begin(void)
{
    if(_beginDone == 1) return;
    Wire.setSDA(TPS_SDA);
    Wire.setSCL(TPS_SCL);
    Wire.begin();
    //memset(mcpRegsInt, 0, 22);
    //memset(mcpRegsEx, 0, 22);
    //mcpBegin(MCP23017_INT_ADDR, mcpRegsInt);
    //mcpBegin(MCP23017_EXT_ADDR, mcpRegsEx);
    //pinModeInternal(MCP23017_INT_ADDR, mcpRegsInt, VCOM, OUTPUT);
    //pinModeInternal(MCP23017_INT_ADDR, mcpRegsInt, PWRUP, OUTPUT);
    //pinModeInternal(MCP23017_INT_ADDR, mcpRegsInt, WAKEUP, OUTPUT);
    //pinModeInternal(MCP23017_INT_ADDR, mcpRegsInt, GPIO0_ENABLE, OUTPUT);
    //digitalWriteInternal(MCP23017_INT_ADDR, mcpRegsInt, GPIO0_ENABLE, HIGH);
  
    WAKEUP_SET;
    delay(1);
    Wire.beginTransmission(0x48);
    Wire.write(0x09);
    Wire.write(B00011011); // Power up seq.
    Wire.write(B00000000); // Power up delay (3mS per rail)
    Wire.write(B00011011); // Power down seq.
    Wire.write(B00000000); // Power down delay (6mS per rail)
    Wire.endTransmission();
    delay(1);
    WAKEUP_CLEAR;
  
    // Set all pins of seconds I/O expander to outputs, low.
    // For some reason, it draw more current in deep sleep when pins are set as inputs...
    //for(int i = 0; i < 15; i++)
    //{
    //    pinModeInternal(MCP23017_EXT_ADDR, mcpRegsInt, i, OUTPUT);
    //    digitalWriteInternal(MCP23017_EXT_ADDR, mcpRegsInt, i, LOW);
    //}
  
    // For same reason, unused pins of first I/O expander have to be also set as outputs, low.
    //pinModeInternal(MCP23017_INT_ADDR, mcpRegsInt, 13, OUTPUT);
    //pinModeInternal(MCP23017_INT_ADDR, mcpRegsInt, 14, OUTPUT);
    //pinModeInternal(MCP23017_INT_ADDR, mcpRegsInt, 15, OUTPUT);
    //digitalWriteInternal(MCP23017_INT_ADDR, mcpRegsInt, 13, LOW);
    //digitalWriteInternal(MCP23017_INT_ADDR, mcpRegsInt, 14, LOW);
    //digitalWriteInternal(MCP23017_INT_ADDR, mcpRegsInt, 15, LOW);
    
    pinMode(PE9, OUTPUT);
    pinMode(PG12, OUTPUT);
    pinMode(PF3, OUTPUT);

    // CONTROL PINS
    pinMode(PD11, OUTPUT);
    pinMode(PC9, OUTPUT);
    pinMode(PC10, OUTPUT);
    pinMode(PG3, OUTPUT);
    pinMode(PC12, OUTPUT);
    pinMode(PE2, OUTPUT);
    pinMode(PE3, OUTPUT);

    // DATA PINS
    pinMode(PD0, OUTPUT); //D0
    pinMode(PD1, OUTPUT);
    pinMode(PD2, OUTPUT);
    pinMode(PD3, OUTPUT);
    pinMode(PD4, OUTPUT);
    pinMode(PD5, OUTPUT);
    pinMode(PD6, OUTPUT);
    pinMode(PD7, OUTPUT); //D7
  
    // TOUCHPAD PINS
    //pinModeInternal(MCP23017_INT_ADDR, mcpRegsInt, 10, INPUT);
    //pinModeInternal(MCP23017_INT_ADDR, mcpRegsInt, 11, INPUT);
    //pinModeInternal(MCP23017_INT_ADDR, mcpRegsInt, 12, INPUT);
  
    // Battery voltage Switch MOSFET
    //pinModeInternal(MCP23017_INT_ADDR, mcpRegsInt, 9, OUTPUT);
  
    D_memory_new = (uint8_t*)malloc(E_INK_WIDTH * E_INK_HEIGHT / 8);
    //_partial = (uint8_t*)ps_malloc(E_INK_WIDTH * E_INK_HEIGHT / 8);
    //_pBuffer = (uint8_t*) ps_malloc(E_INK_WIDTH * E_INK_HEIGHT / 4);
    D_memory4Bit = (uint8_t*)malloc(E_INK_WIDTH * E_INK_HEIGHT / 2);
    GLUT = (uint32_t*)malloc(256 * 8 * sizeof(uint32_t));
    GLUT2 = (uint32_t*)malloc(256 * 8 * sizeof(uint32_t));
    if (D_memory_new == NULL  || D_memory4Bit == NULL || GLUT == NULL || GLUT2 == NULL)
    {
        Serial.print("Memory alloc. fail");
        do
        {
            delay(100);
        }
        while (true);
    }
    memset(D_memory_new, 0, E_INK_WIDTH * E_INK_HEIGHT / 8);
    //memset(_partial, 0, E_INK_WIDTH * E_INK_HEIGHT / 8);
    //memset(_pBuffer, 0, E_INK_WIDTH * E_INK_HEIGHT / 4);
    memset(D_memory4Bit, 255, E_INK_WIDTH * E_INK_HEIGHT / 2);
  
    for (int j = 0; j < 8; ++j) {
        for (uint32_t i = 0; i < 256; ++i) {
        GLUT[j*256+i] = (waveform3Bit[i & 0x07][j] << 2) | (waveform3Bit[(i >> 4) & 0x07][j]);
        GLUT2[j*256+i] = ((waveform3Bit[i & 0x07][j] << 2) | (waveform3Bit[(i >> 4) & 0x07][j])) << 4;
        }
    }
  
    _beginDone = 1;
}

// Draw function, used by Adafruit GFX.
void Inkplate::drawPixel(int16_t x0, int16_t y0, uint16_t color)
{
    if (x0 > width() - 1 || y0 > height() - 1 || x0 < 0 || y0 < 0)
        return;

    switch (rotation)
    {
    case 1:
        _swap_int16_t(x0, y0);
        x0 = height() - x0 - 1;
        break;
    case 2:
        x0 = width() - x0 - 1;
        y0 = height() - y0 - 1;
        break;
    case 3:
        _swap_int16_t(x0, y0);
        y0 = width() - y0 - 1;
        break;
    }

    if (_displayMode == 0) {
        int x = x0 / 8;
        int x_sub = x0 % 8;
        uint8_t temp = *(_partial + (E_INK_WIDTH/8 * y0) + x);
        *(_partial + (E_INK_WIDTH/8 * y0) + x) = ~pixelMaskLUT[x_sub] & temp | (color ? pixelMaskLUT[x_sub] : 0);
    } else {
        color &= 7;
        int x = x0 / 2;
        int x_sub = x0 % 2;
        uint8_t temp;
        temp = *(D_memory4Bit + E_INK_WIDTH/2 * y0 + x);
        *(D_memory4Bit + E_INK_WIDTH/2 * y0 + x) = pixelMaskGLUT[x_sub] & temp | (x_sub ? color : color << 4);
    }
}

void Inkplate::clearDisplay()
{
    // Clear 1 bit per pixel display buffer
    if (_displayMode == 0) memset(_partial, 0, E_INK_WIDTH * E_INK_HEIGHT/8);

    // Clear 3 bit per pixel display buffer
    if (_displayMode == 1) memset(D_memory4Bit, 255, E_INK_WIDTH * E_INK_HEIGHT/2);
}

// Function that displays content from RAM to screen
void Inkplate::display()
{
    if (_displayMode == 0) display1b();
    if (_displayMode == 1) display3b();
}

void Inkplate::partialUpdate()
{
    if (_displayMode == 1) return;
    if (_blockPartial == 1)
    {
        display1b();
        return;
    }
    
    uint32_t _pos = (E_INK_WIDTH * E_INK_HEIGHT / 8) - 1;
    uint8_t data;
    uint8_t diffw, diffb;
    uint32_t n = (E_INK_WIDTH * E_INK_HEIGHT / 4) - 1;
    uint8_t dram;
  
    for (int i = 0; i < E_INK_HEIGHT; i++) {
        for (int j = 0; j < E_INK_WIDTH / 8; j++) {
            diffw = ((*(D_memory_new + _pos))^(*(_partial + _pos)))&(~(*(_partial + _pos)));
            diffb = ((*(D_memory_new + _pos))^(*(_partial + _pos)))&((*(_partial + _pos)));
            _pos--;
            *(_pBuffer + n) = LUTW[diffw>>4] & (LUTB[diffb>>4]);
            n--;
            *(_pBuffer + n) = LUTW[diffw&0x0F] & (LUTB[diffb&0x0F]);
            n--;
        }
    }	  
   
    einkOn();
    for (int k = 0; k < 5; k++)
    {
        vscan_start();
        n = (E_INK_WIDTH * E_INK_HEIGHT / 4) - 1;
        for (int i = 0; i < E_INK_HEIGHT; i++)
        {
            data = *(_pBuffer + n);
            hscan_start(data);
            n--;
        for (int j = 0; j < ((E_INK_WIDTH / 4) - 1); j++)
        {
            data = *(_pBuffer + n);
            GPIOD -> BSRR = (data) | CL;
            GPIOD -> BSRR  = (DATA | CL) << 16;
            n--;
        }
        GPIOD -> BSRR = CL;
        GPIOD -> BSRR = (DATA | CL) << 16;
        vscan_end();
        }
        delayMicroseconds(230);
    }
    /*
        for (int k = 0; k < 1; k++)
        {
            vscan_start();
            _pos = 59999;
            for (int i = 0; i < 600; i++) {
            data = discharge[(*(D_memory_new + _pos) >> 4)];
            _send = ((data & B00000011) << 4) | (((data & B00001100) >> 2) << 18) | (((data & B00010000) >> 4) << 23) | (((data & B11100000) >> 5) << 25);
            hscan_start(_send);
            data = discharge[*(D_memory_new + _pos) & 0x0F];
            _send = ((data & B00000011) << 4) | (((data & B00001100) >> 2) << 18) | (((data & B00010000) >> 4) << 23) | (((data & B11100000) >> 5) << 25);
            GPIO.out_w1ts = (_send) | CL;
            GPIO.out_w1tc = DATA | CL;
            _pos--;
            for (int j = 0; j < 99; j++)
            {
                data = discharge[(*(D_memory_new + _pos) >> 4)];
                _send = ((data & B00000011) << 4) | (((data & B00001100) >> 2) << 18) | (((data & B00010000) >> 4) << 23) | (((data & B11100000) >> 5) << 25);
                GPIO.out_w1ts = (_send) | CL;
                GPIO.out_w1tc = DATA | CL;
                data = discharge[*(D_memory_new + _pos) & 0x0F];
                _send = ((data & B00000011) << 4) | (((data & B00001100) >> 2) << 18) | (((data & B00010000) >> 4) << 23) | (((data & B11100000) >> 5) << 25);
                GPIO.out_w1ts = (_send) | CL;
                GPIO.out_w1tc = DATA | CL;
                _pos--;
            }
            GPIO.out_w1ts = (_send) | CL;
            GPIO.out_w1tc = DATA | CL;
            vscan_end();
        }
        delayMicroseconds(230);
    }
    */
    cleanFast(2, 2);
    cleanFast(3, 1);
    vscan_start();
    einkOff();
    for (int i = 0; i < (E_INK_WIDTH * E_INK_HEIGHT / 8); i++)
    {
        *(D_memory_new + i) &= *(_partial + i);
        *(D_memory_new + i) |= (*(_partial + i));
    }
}

void Inkplate::drawBitmap3Bit(int16_t _x, int16_t _y, const unsigned char* _p, int16_t _w, int16_t _h)
{
    if (_displayMode != INKPLATE_3BIT) return;
    uint8_t  _rem = _w % 2;
    int i, j;
    int xSize = _w / 2 + _rem;

    for (i = 0; i < _h; i++)
    {
        for (j = 0; j < xSize - 1; j++)
        {
            drawPixel((j * 2) + _x, i + _y, (*(_p + xSize * (i) + j) >> 4) >> 1);
            drawPixel((j * 2) + 1 + _x, i + _y, (*(_p + xSize * (i) + j) & 0xff) >> 1);
        }
        drawPixel((j * 2) + _x, i + _y, (*(_p + xSize * (i) + j) >> 4)>>1);
        if (_rem == 0) drawPixel((j * 2) + 1 + _x, i + _y, (*(_p + xSize * (i) + j) & 0xff) >> 1);
    }
}

void Inkplate::setRotation(uint8_t r)
{
    rotation = (r & 3);
    switch (rotation)
    {
    case 0:
    case 2:
        _width = E_INK_WIDTH;
        _height = E_INK_HEIGHT;
        break;
    case 1:
    case 3:
        _width = E_INK_HEIGHT;
        _height = E_INK_WIDTH;
        break;
    }
}

//Turn off epapewr supply and put all digital IO pins in high Z state
// Turn off epaper power supply and put all digital IO pins in high Z state
void Inkplate::einkOff()
{
    if (getPanelState() == 0)
        return;
    OE_CLEAR;
    GMOD_CLEAR;
    GPIOD -> BSRR = DATA << 16;
    CKV_CLEAR;
    SPH_CLEAR;
    SPV_CLEAR;
    LE_CLEAR;
    CL_CLEAR;

    VCOM_CLEAR;
    delay(6);
    PWRUP_CLEAR;
    WAKEUP_CLEAR;
    
    unsigned long timer = millis();
    do
    {
        delay(1);
    }
    while ((readPowerGood() != 0) && (millis() - timer) < 250);

    //pinsZstate();
    setPanelState(0);
}

// Turn on supply for epaper display (TPS65186) [+15 VDC, -15VDC, +22VDC, -20VDC, +3.3VDC, VCOM]
void Inkplate::einkOn()
{
    if (getPanelState() == 1)
        return;
    WAKEUP_SET;
    delay(1);
    PWRUP_SET;

    // Enable all rails
    Wire.beginTransmission(0x48);
    Wire.write(0x01);
    Wire.write(B00111111);
    Wire.endTransmission();
    pinsAsOutputs();
    LE_CLEAR;
    OE_CLEAR;
    CL_CLEAR;
    SPH_SET;
    GMOD_SET;
    SPV_SET;
    CKV_CLEAR;
    OE_CLEAR;
    VCOM_SET;

    unsigned long timer = millis();
    do
    {
        delay(1);
    }
    while ((readPowerGood() != PWR_GOOD_OK) && (millis() - timer) < 250);
	if ((millis() - timer) >= 250)
    {
        WAKEUP_CLEAR;
		VCOM_CLEAR;
		PWRUP_CLEAR;
		return;
    }

    OE_SET;
    setPanelState(1);
}

uint8_t Inkplate::readPowerGood()
{
    Wire.beginTransmission(0x48);
    Wire.write(0x0F);
    Wire.endTransmission();
	
    Wire.requestFrom(0x48, 1);
    return Wire.read();
}

void Inkplate::selectDisplayMode(uint8_t _mode)
{
	if(_mode != _displayMode) {
		_displayMode = _mode & 1;
		memset(D_memory_new, 0, E_INK_WIDTH * E_INK_HEIGHT / 8);
		memset(_partial, 0, E_INK_WIDTH * E_INK_HEIGHT / 8);
		memset(_pBuffer, 0, E_INK_WIDTH * E_INK_HEIGHT / 4);
		memset(D_memory4Bit, 255, E_INK_WIDTH * E_INK_HEIGHT / 2);
		_blockPartial = 1;
	}
}

uint8_t Inkplate::getDisplayMode()
{
  return _displayMode;
}

int Inkplate::drawBitmapFromSD(SdFile* p, int x, int y)
{
	if(sdCardOk == 0) return 0;
	struct bitmapHeader bmpHeader;
	readBmpHeader(p, &bmpHeader);
	if (bmpHeader.signature != 0x4D42 || bmpHeader.compression != 0 || !(bmpHeader.color == 1 || bmpHeader.color == 24)) return 0;

	if ((bmpHeader.color == 24 || bmpHeader.color == 32) && getDisplayMode() != INKPLATE_3BIT) {
		selectDisplayMode(INKPLATE_3BIT);
	}

	if (bmpHeader.color == 1 && getDisplayMode() != INKPLATE_1BIT) {
		selectDisplayMode(INKPLATE_1BIT);
	}
  
	if (bmpHeader.color == 1) drawMonochromeBitmap(p, bmpHeader, x, y);
	if (bmpHeader.color == 24) drawGrayscaleBitmap24(p, bmpHeader, x, y);

  return 1;
}

int Inkplate::drawBitmapFromSD(char* fileName, int x, int y)
{
  if(sdCardOk == 0) return 0;
  SdFile dat;
  if (dat.open(fileName, O_RDONLY)) {
    return drawBitmapFromSD(&dat, x, y);
  } else {
    return 0;
  }
}

int Inkplate::sdCardInit()
{
	SPI.begin(10);
	sdCardOk = sd.begin(10, SD_SCK_MHZ(25));
	return sdCardOk;
}

SdFat Inkplate::getSdFat()
{
	return sd;
}

SPIClass Inkplate::getSPI()
{
	return spi2;
}

uint8_t Inkplate::getPanelState()
{
    return _panelOn;
}

void Inkplate::setPanelState(uint8_t state)
{
    _panelOn = state;
}

uint8_t Inkplate::readTouchpad(uint8_t _pad)
{
    //return digitalReadInternal(MCP23017_INT_ADDR, mcpRegsInt, (_pad&3)+10);
}

int8_t Inkplate::readTemperature()
{
    int8_t temp;
    if(getPanelState() == 0)
    {
        WAKEUP_SET;
        PWRUP_SET;
        delay(5);
    }
    Wire.beginTransmission(0x48);
    Wire.write(0x0D);
    Wire.write(B10000000);
    Wire.endTransmission();
    delay(5);

    Wire.beginTransmission(0x48);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.requestFrom(0x48, 1);
    temp = Wire.read();
    if(getPanelState() == 0)
    {
        PWRUP_CLEAR;
        WAKEUP_CLEAR;
        delay(5);
    }
    return temp;
}

double Inkplate::readBattery()
{
    //digitalWriteInternal(MCP23017_INT_ADDR, mcpRegsInt, 9, HIGH);
    //delay(1);
    //int adc = analogRead(35);
    //digitalWriteInternal(MCP23017_INT_ADDR, mcpRegsInt, 9, LOW);
    // Calculate the voltage using the following formula
    // 1.1V is internal ADC reference of ESP32, 3.548133892 is 11dB in linear scale (Analog signal is attenuated by 11dB before ESP32 ADC input)
    //return (double(adc) / 4095 * 1.1 * 3.548133892 * 2);
    return -1;
}

//--------------------------LOW LEVEL STUFF--------------------------------------------
void Inkplate::vscan_start()
{
    CKV_SET;
    delayMicroseconds(7);
    SPV_CLEAR;
    delayMicroseconds(10);
    CKV_CLEAR;
    delayMicroseconds(1);
    CKV_SET;
    delayMicroseconds(8);
    SPV_SET;
    delayMicroseconds(10);
    CKV_CLEAR;
    delayMicroseconds(1);
    CKV_SET;
    delayMicroseconds(18);
    CKV_CLEAR;
    delayMicroseconds(0);
    CKV_SET;
    delayMicroseconds(18);
    CKV_CLEAR;
    delayMicroseconds(1);
    CKV_SET;
    delayMicroseconds(18);
}

void Inkplate::hscan_start(uint32_t _d)
{
    SPH_CLEAR;
    GPIOD -> BSRR = (_d) | CL;
    GPIOD -> BSRR  = (DATA | CL) << 16;
    SPH_SET;
    CKV_SET;
}

void Inkplate::vscan_end() {
    CKV_CLEAR;
    LE_SET;
    LE_CLEAR;
    delayMicroseconds(1);
}

// Clears content from epaper diplay as fast as ESP32 can.
void Inkplate::cleanFast(uint8_t c, uint8_t rep)
{
    einkOn();
    uint8_t data;
    if (c == 0)
    {
        data = B10101010;     //White
    } 
    else if (c == 1)
    {
        data = B01010101;     //Black
    }
    else if (c == 2)
    {
        data = B00000000;     //Discharge
    }
    else if (c == 3)
    {
        data = B11111111;	  //Skip
    }
  
    uint32_t _send = ((data & B00000011) << 4) | (((data & B00001100) >> 2) << 18) | (((data & B00010000) >> 4) << 23) | (((data & B11100000) >> 5) << 25);;
    for (int k = 0; k < rep; k++)
    {
        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; i++)
        {
            hscan_start(_send);
            GPIOD -> BSRR  = (_send) | CL;
            GPIOD -> BSRR  = CL << 16;
            for (int j = 0; j < (E_INK_WIDTH / 8) - 1; j++)
            {
                GPIOD -> BSRR = CL;
                GPIOD -> BSRR = CL << 16; 
                GPIOD -> BSRR = CL;
                GPIOD -> BSRR = CL << 16;
            }
            GPIOD -> BSRR = (_send) | CL;
            GPIOD -> BSRR = (DATA | CL) << 16;
            vscan_end();
        }
        delayMicroseconds(230);
    }
}

void Inkplate::pinsZstate()
{
    // CONTROL PINS
    pinMode(PD11, INPUT);
    pinMode(PC9, INPUT);
    pinMode(PC10, INPUT);
    pinMode(PG3, INPUT);
    pinMode(PC12, INPUT);
    pinMode(PE2, INPUT);
    pinMode(PE3, INPUT);

    // DATA PINS
    pinMode(PD0, INPUT); //D0
    pinMode(PD1, INPUT);
    pinMode(PD2, INPUT);
    pinMode(PD3, INPUT);
    pinMode(PD4, INPUT);
    pinMode(PD5, INPUT);
    pinMode(PD6, INPUT);
    pinMode(PD7, INPUT); //D7
}

void Inkplate::pinsAsOutputs()
{
    // CONTROL PINS
    pinMode(PD11, OUTPUT);
    pinMode(PC9, OUTPUT);
    pinMode(PC10, OUTPUT);
    pinMode(PG3, OUTPUT);
    pinMode(PC12, OUTPUT);
    pinMode(PE2, OUTPUT);
    pinMode(PE3, OUTPUT);

    // DATA PINS
    pinMode(PD0, OUTPUT); //D0
    pinMode(PD1, OUTPUT);
    pinMode(PD2, OUTPUT);
    pinMode(PD3, OUTPUT);
    pinMode(PD4, OUTPUT);
    pinMode(PD5, OUTPUT);
    pinMode(PD6, OUTPUT);
    pinMode(PD7, OUTPUT); //D7
}

//--------------------------PRIVATE FUNCTIONS--------------------------------------------
// Display content from RAM to display (1 bit per pixel,. monochrome picture).
void Inkplate::display1b()
{
    for(int i = 0; i<(E_INK_HEIGHT * E_INK_WIDTH) / 8; i++)
    {
        *(D_memory_new + i) &= *(_partial + i);
        *(D_memory_new + i) |= (*(_partial + i));
    }
    uint32_t _pos;
    uint8_t data;
    uint8_t dram;
    einkOn();
    cleanFast(0, 17);
    cleanFast(1, 17);
    cleanFast(0, 17);
    cleanFast(1, 17);
      for (int k = 0; k < 4; k++)
      {
        _pos = (E_INK_HEIGHT * E_INK_WIDTH / 8) - 1;
        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; i++)
        {
        dram = ~(*(D_memory_new + _pos));
        data = LUTW[(dram >> 4) & 0x0F];
        hscan_start(data);
        data = LUTW[dram & 0x0F];
        GPIOD -> BSRR = data | CL;
        GPIOD -> BSRR = (DATA | CL) << 16;
        _pos--;
        for (int j = 0; j < ((E_INK_WIDTH / 8) - 1); j++)
        {
            dram = ~(*(D_memory_new + _pos));
            data = LUTW[(dram >> 4)&0x0F];
            GPIOD -> BSRR = data | CL;
            GPIOD -> BSRR = (DATA | CL) << 16;
            data = LUTW[dram & 0x0F];
            GPIOD -> BSRR = data | CL;
            GPIOD -> BSRR = (DATA | CL) << 16;
            _pos--;
        }
        GPIOD -> BSRR = CL;
        GPIOD -> BSRR = (DATA | CL) << 16;
        vscan_end();
        }
        delayMicroseconds(230);
    }
  
	_pos = (E_INK_HEIGHT * E_INK_WIDTH / 8) - 1;
    vscan_start();
    for (int i = 0; i < E_INK_HEIGHT; i++)
    {
	  dram = *(D_memory_new + _pos);
      data = LUT2[(dram >> 4) & 0x0F];
	  hscan_start(data);
	  data = LUT2[dram & 0x0F];
	  GPIOD -> BSRR = (data) | CL;
      GPIOD -> BSRR = (DATA | CL) << 16;
	  _pos--;
      for (int j = 0; j < ((E_INK_WIDTH / 8)-1); j++)
      {
		dram = *(D_memory_new + _pos);
        data = LUT2[(dram >> 4) & 0x0F];
		GPIOD -> BSRR = (data) | CL;
        GPIOD -> BSRR = (DATA | CL) << 16;
        data = LUT2[dram & 0x0F];
		GPIOD -> BSRR = (data) | CL;
        GPIOD -> BSRR = (DATA | CL) << 16;
		_pos--;
      }
	  GPIOD -> BSRR = CL;
      GPIOD -> BSRR = (DATA | CL) << 16;
	  vscan_end();
    }
    delayMicroseconds(230);
    
    cleanFast(2, 2);
    cleanFast(3, 1);
    vscan_start();
    einkOff();
    _blockPartial = 0;
}

// Display content from RAM to display (3 bit per pixel,. 8 level of grayscale, STILL IN PROGRESSS, we need correct wavefrom to get good picture, use it only for pictures not for GFX).
void Inkplate::display3b()
{
    einkOn();
    cleanFast(0, 17);
    cleanFast(1, 17);
    cleanFast(0, 17);
    cleanFast(1, 17);
  
    for (int k = 0; k < 8; k++)
    {
        uint8_t *dp = D_memory4Bit + (E_INK_HEIGHT * E_INK_WIDTH/2);
	  
        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; i++)
        {
            hscan_start((GLUT2[k*256+(*(--dp))] | GLUT[k*256+(*(--dp))]));
            GPIOD -> BSRR = (GLUT2[k*256+(*(--dp))] | GLUT[k*256+(*(--dp))]) | CL;
            GPIOD -> BSRR = (DATA | CL) << 16;
		
            for (int j = 0; j < ((E_INK_WIDTH/8)-1); j++)
            {
                GPIOD -> BSRR = (GLUT2[k*256+(*(--dp))] | GLUT[k*256+(*(--dp))]) | CL;
                GPIOD -> BSRR = (DATA | CL) << 16;
                GPIOD -> BSRR = (GLUT2[k*256+(*(--dp))] | GLUT[k*256+(*(--dp))]) | CL;
                GPIOD -> BSRR = (DATA | CL) << 16;
            }
        
            GPIOD -> BSRR = CL;
            GPIOD -> BSRR = (DATA | CL) << 16;
            vscan_end();
        }
        delayMicroseconds(230);
    }
    cleanFast(3, 1);
    vscan_start();
    einkOff();
}

uint32_t Inkplate::read32(uint8_t* c)
{
    return (*(c) | (*(c + 1) << 8) | (*(c + 2) << 16) | (*(c + 3) << 24));
}

uint16_t Inkplate::read16(uint8_t* c)
{
    return (*(c) | (*(c + 1) << 8));
}

void Inkplate::readBmpHeader(SdFile *_f, struct bitmapHeader *_h)
{
  uint8_t header[100];
  _f->rewind();
  _f->read(header, 100);
  _h->signature = read16(header + 0);
  _h->fileSize = read32(header + 2);
  _h->startRAW = read32(header + 10);
  _h->dibHeaderSize = read32(header + 14);
  _h->width = read32(header + 18);
  _h->height = read32(header + 22);
  _h->color = read16(header + 28);
  _h->compression = read32(header + 30);
  return;
}

int Inkplate::drawMonochromeBitmap(SdFile *f, struct bitmapHeader bmpHeader, int x, int y)
{
  int w = bmpHeader.width;
  int h = bmpHeader.height;
  uint8_t paddingBits = w % 32;
  w /= 32;

  f->seekSet(bmpHeader.startRAW);
  int i, j;
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      uint32_t pixelRow = f->read() << 24 | f->read() << 16 | f->read() << 8 | f->read();
      for (int n = 0; n < 32; n++) {
        drawPixel((i * 32) + n + x, h - j + y, !(pixelRow & (1ULL << (31 - n))));
      }
    }
    if (paddingBits) {
      uint32_t pixelRow = f->read() << 24 | f->read() << 16 | f->read() << 8 | f->read();
      for (int n = 0; n < paddingBits; n++) {
        drawPixel((i * 32) + n + x, h - j + y, !(pixelRow & (1ULL << (31 - n))));
      }
    }
  }
  f->close();
  return 1;
}

int Inkplate::drawGrayscaleBitmap24(SdFile *f, struct bitmapHeader bmpHeader, int x, int y)
{
  int w = bmpHeader.width;
  int h = bmpHeader.height;
  char padding = w % 4;
  f->seekSet(bmpHeader.startRAW);
  int i, j;
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      //This is the proper way of converting True Color (24 Bit RGB) bitmap file into grayscale, but it takes waaay too much time (full size picture takes about 17s to decode!)
      //float px = (0.2126 * (readByteFromSD(&file) / 255.0)) + (0.7152 * (readByteFromSD(&file) / 255.0)) + (0.0722 * (readByteFromSD(&file) / 255.0));
      //px = pow(px, 1.5);
      //display.drawPixel(i + x, h - j + y, (uint8_t)(px*7));

      //So then, we are convertng it to grayscale. With this metod, it is still slow (full size image takes 4 seconds), but much beter than prev mentioned method.
      uint8_t px = (f->read() * 2126 / 10000) + (f->read() * 7152 / 10000) + (f->read() * 722 / 10000);
	  drawPixel(i + x, h - j + y, px>>5);
    }
    if (padding) {
      for (int p = 0; p < padding; p++) {
        f->read();
      }
    }
  }
  f->close();
  return 1;
}


// MOT USED ON STM32. STM32 HAS ENOUGH PINS! COMMENT ALL!!!
/*
//----------------------------MCP23017 functions----------------------------
bool Inkplate::mcpBegin(uint8_t _addr, uint8_t* _r)
{
    Wire.beginTransmission(_addr);
    int error = Wire.endTransmission();
    if (error) return false;
    readMCPRegisters(_addr, _r);
    _r[0] = 0xff;
    _r[1] = 0xff;
    updateAllRegisters(_addr, _r);
    return true;
}

void Inkplate::readMCPRegisters(uint8_t _addr, uint8_t *k)
{
    Wire.beginTransmission(_addr);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(_addr, (uint8_t)22);
    for (int i = 0; i < 22; i++)
    {
        k[i] = Wire.read();
    }
}

void Inkplate::readMCPRegisters(uint8_t _addr, uint8_t _regName, uint8_t *k, uint8_t _n)
{
    Wire.beginTransmission(_addr);
    Wire.write(_regName);
    Wire.endTransmission();

    Wire.requestFrom(_addr, _n);
    for (int i = 0; i < _n; i++)
    {
        k[_regName + i] = Wire.read();
    }
}

void Inkplate::readMCPRegister(uint8_t _addr, uint8_t _regName, uint8_t *k)
{
    Wire.beginTransmission(_addr);
    Wire.write(_regName);
    Wire.endTransmission();
    Wire.requestFrom(_addr, (uint8_t)1);
    k[_regName] = Wire.read();
}

void Inkplate::updateAllRegisters(uint8_t _addr, uint8_t *k)
{
    Wire.beginTransmission(_addr);
    Wire.write(0x00);
    for (int i = 0; i < 22; i++)
    {
        Wire.write(k[i]);
    }
    Wire.endTransmission();
}

void Inkplate::updateRegister(uint8_t _addr, uint8_t _regName, uint8_t _d)
{
    Wire.beginTransmission(_addr);
    Wire.write(_regName);
    Wire.write(_d);
    Wire.endTransmission();
}

void Inkplate::updateRegister(uint8_t _addr, uint8_t _regName, uint8_t *k, uint8_t _n)
{
    Wire.beginTransmission(_addr);
    Wire.write(_regName);
    for (int i = 0; i < _n; i++) {
        Wire.write(k[_regName + i]);
    }
    Wire.endTransmission();
}

void Inkplate::pinModeInternal(uint8_t _addr, uint8_t* _r, uint8_t _pin, uint8_t _mode)
{
    uint8_t _port = (_pin / 8) & 1;
    uint8_t _p = _pin % 8;

    switch (_mode)
    {
    case INPUT:
        _r[MCP23017_IODIRA + _port] |= 1 << _p;   //Set it to input
        _r[MCP23017_GPPUA + _port] &= ~(1 << _p); //Disable pullup on that pin
        updateRegister(_addr, MCP23017_IODIRA + _port, _r[MCP23017_IODIRA + _port]);
        updateRegister(_addr, MCP23017_GPPUA + _port, _r[MCP23017_GPPUA + _port]);
        break;

    case INPUT_PULLUP:
        _r[MCP23017_IODIRA + _port] |= 1 << _p;   //Set it to input
        _r[MCP23017_GPPUA + _port] |= 1 << _p;    //Enable pullup on that pin
        updateRegister(_addr, MCP23017_IODIRA + _port, _r[MCP23017_IODIRA + _port]);
        updateRegister(_addr, MCP23017_GPPUA + _port, _r[MCP23017_GPPUA + _port]);
        break;

    case OUTPUT:
        _r[MCP23017_IODIRA + _port] &= ~(1 << _p); //Set it to output
        _r[MCP23017_GPPUA + _port] &= ~(1 << _p); //Disable pullup on that pin
        updateRegister(_addr, MCP23017_IODIRA + _port, _r[MCP23017_IODIRA + _port]);
        updateRegister(_addr, MCP23017_GPPUA + _port, _r[MCP23017_GPPUA + _port]);
        break;
    }
}

void Inkplate::digitalWriteInternal(uint8_t _addr, uint8_t* _r, uint8_t _pin, uint8_t _state)
{
    uint8_t _port = (_pin / 8) & 1;
    uint8_t _p = _pin % 8;

    if (_r[MCP23017_IODIRA + _port] & (1 << _p)) return; //Check if the pin is set as an output
    _state ? (_r[MCP23017_GPIOA + _port] |= (1 << _p)) : (_r[MCP23017_GPIOA + _port] &= ~(1 << _p));
    updateRegister(_addr, MCP23017_GPIOA + _port, _r[MCP23017_GPIOA + _port]);
}

uint8_t Inkplate::digitalReadInternal(uint8_t _addr, uint8_t* _r, uint8_t _pin)
{
    uint8_t _port = (_pin / 8) & 1;
    uint8_t _p = _pin % 8;
    readMCPRegister(_addr, MCP23017_GPIOA + _port, _r);
    return (_r[MCP23017_GPIOA + _port] & (1 << _p)) ? HIGH : LOW;
}

void Inkplate::setIntOutputInternal(uint8_t _addr, uint8_t* _r, uint8_t intPort, uint8_t mirroring, uint8_t openDrain, uint8_t polarity)
{
    intPort &= 1;
    mirroring &= 1;
    openDrain &= 1;
    polarity &= 1;
    _r[MCP23017_IOCONA + intPort] = (_r[MCP23017_IOCONA + intPort] & ~(1 << 6)) | (mirroring << 6);
    _r[MCP23017_IOCONA + intPort] = (_r[MCP23017_IOCONA + intPort] & ~(1 << 2)) | (openDrain << 2);
    _r[MCP23017_IOCONA + intPort] = (_r[MCP23017_IOCONA + intPort] & ~(1 << 1)) | (polarity << 1);
    updateRegister(_addr, MCP23017_IOCONA + intPort, _r[MCP23017_IOCONA + intPort]);
}

void Inkplate::setIntPinInternal(uint8_t _addr, uint8_t* _r, uint8_t _pin, uint8_t _mode)
{
    uint8_t _port = (_pin / 8) & 1;
    uint8_t _p = _pin % 8;

    switch (_mode)
    {
    case CHANGE:
        _r[MCP23017_INTCONA + _port] &= ~(1 << _p);
        break;

    case FALLING:
        _r[MCP23017_INTCONA + _port] |= (1 << _p);
        _r[MCP23017_DEFVALA + _port] |= (1 << _p);
        break;

    case RISING:
        _r[MCP23017_INTCONA + _port] |= (1 << _p);
        _r[MCP23017_DEFVALA + _port] &= ~(1 << _p);
        break;
    }
    _r[MCP23017_GPINTENA + _port] |= (1 << _p);
    updateRegister(_addr, MCP23017_GPINTENA, _r, 6);
}

void Inkplate::removeIntPinInternal(uint8_t _addr, uint8_t* _r, uint8_t _pin)
{
    uint8_t _port = (_pin / 8) & 1;
    uint8_t _p = _pin % 8;
    _r[MCP23017_GPINTENA + _port] &= ~(1 << _p);
    updateRegister(_addr, MCP23017_GPINTENA, _r, 2);
}

uint16_t Inkplate::getINTInternal(uint8_t _addr, uint8_t* _r)
{
    readMCPRegisters(_addr, MCP23017_INTFA, _r, 2);
    return ((_r[MCP23017_INTFB] << 8) | _r[MCP23017_INTFA]);
}

uint16_t Inkplate::getINTstateInternal(uint8_t _addr, uint8_t* _r)
{
    readMCPRegisters(_addr, MCP23017_INTCAPA, _r, 2);
    return ((_r[MCP23017_INTCAPB] << 8) | _r[MCP23017_INTCAPA]);
}

void Inkplate::setPortsInternal(uint8_t _addr, uint8_t* _r, uint16_t _d)
{
    _r[MCP23017_GPIOA] = _d & 0xff;
    _r[MCP23017_GPIOB] = (_d >> 8) & 0xff;
    updateRegister(_addr, MCP23017_GPIOA, _r, 2);
}

uint16_t Inkplate::getPortsInternal(uint8_t _addr, uint8_t* _r)
{
    readMCPRegisters(_addr, MCP23017_GPIOA, _r, 2);
    return ((_r[MCP23017_GPIOB] << 8) | (_r[MCP23017_GPIOA]));
}

//---------------------Functions that are used by user (visible outside this library)----------------------------
void Inkplate::pinModeMCP(uint8_t _pin, uint8_t _mode)
{
	pinModeInternal(MCP23017_EXT_ADDR, mcpRegsEx, _pin, _mode);
}

void Inkplate::digitalWriteMCP(uint8_t _pin, uint8_t _state)
{
	digitalWriteInternal(MCP23017_EXT_ADDR, mcpRegsEx, _pin, _state);
}

uint8_t Inkplate::digitalReadMCP(uint8_t _pin)
{
	return digitalReadInternal(MCP23017_EXT_ADDR, mcpRegsEx, _pin);
}

void Inkplate::setIntOutput(uint8_t intPort, uint8_t mirroring, uint8_t openDrain, uint8_t polarity)
{
	setIntOutputInternal(MCP23017_EXT_ADDR, mcpRegsEx, intPort, mirroring, openDrain, polarity);
}

void Inkplate::setIntPin(uint8_t _pin, uint8_t _mode)
{
	setIntPinInternal(MCP23017_EXT_ADDR, mcpRegsEx, _pin, _mode);
}

void Inkplate::removeIntPin(uint8_t _pin)
{
	removeIntPinInternal(MCP23017_EXT_ADDR, mcpRegsEx, _pin);
}

uint16_t Inkplate::getINT()
{
	return getINTInternal(MCP23017_EXT_ADDR, mcpRegsEx);
}

uint16_t Inkplate::getINTstate()
{
	return getINTstateInternal(MCP23017_EXT_ADDR, mcpRegsEx);
}

void Inkplate::setPorts(uint16_t _d)
{
	setPortsInternal(MCP23017_EXT_ADDR, mcpRegsEx, _d);
}

uint16_t Inkplate::getPorts()
{
	return getPortsInternal(MCP23017_EXT_ADDR, mcpRegsEx);
}
*/
// --------------------------------------------END OF MCP FUNCTIONS--------------------------------------------