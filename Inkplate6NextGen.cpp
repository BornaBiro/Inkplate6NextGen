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
  
    //D_memory_new = (uint8_t*)malloc(E_INK_WIDTH * E_INK_HEIGHT / 8);
    //_partial = (uint8_t*)ps_malloc(E_INK_WIDTH * E_INK_HEIGHT / 8);
    //_pBuffer = (uint8_t*) ps_malloc(E_INK_WIDTH * E_INK_HEIGHT / 4);
    //D_memory4Bit = (uint8_t*)malloc(E_INK_WIDTH * E_INK_HEIGHT / 2);
    imageBuffer = (uint8_t*)malloc(E_INK_WIDTH * E_INK_HEIGHT / 2);
    partialBuffer = (uint8_t*)malloc(E_INK_WIDTH * E_INK_HEIGHT / 2);
    GLUT = (uint8_t*)malloc(256 * 16 * sizeof(uint8_t));
    GLUT2 = (uint8_t*)malloc(256 * 16 * sizeof(uint8_t));
    if (imageBuffer == NULL || partialBuffer == NULL || GLUT == NULL || GLUT2 == NULL)
    {
        Serial.print("Memory alloc. fail");
        do
        {
            delay(100);
        }
        while (true);
    }
    //memset(D_memory_new, 0, E_INK_WIDTH * E_INK_HEIGHT / 8);
    //memset(_partial, 0, E_INK_WIDTH * E_INK_HEIGHT / 8);
    //memset(_pBuffer, 0, E_INK_WIDTH * E_INK_HEIGHT / 4);
    //memset(D_memory4Bit, 255, E_INK_WIDTH * E_INK_HEIGHT / 2);
    memset(imageBuffer, _displayMode?255:0, E_INK_WIDTH * E_INK_HEIGHT / 2);
    memset(partialBuffer, _displayMode?255:0, E_INK_WIDTH * E_INK_HEIGHT / 2);
  
    for (int j = 0; j < 16; ++j) {
        for (uint32_t i = 0; i < 256; ++i) {
        GLUT[j*256+i] = (waveform3Bit[i & 0x0f][j] << 2) | (waveform3Bit[(i >> 4) & 0x0f][j]);
        GLUT2[j*256+i] = ((waveform3Bit[i & 0x0f][j] << 2) | (waveform3Bit[(i >> 4) & 0x0f][j])) << 4;
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
        //uint8_t temp = *(_partial + (E_INK_WIDTH/8 * y0) + x);
        //*(_partial + (E_INK_WIDTH/8 * y0) + x) = ~pixelMaskLUT[x_sub] & temp | (color ? pixelMaskLUT[x_sub] : 0);
        //uint8_t temp = *(D_memory_new + (E_INK_WIDTH/8 * y0) + x);
        //*(D_memory_new + (E_INK_WIDTH/8 * y0) + x) = ~pixelMaskLUT[x_sub] & temp | (color ? pixelMaskLUT[x_sub] : 0);
        
        
        //uint8_t temp = *(imageBuffer + (E_INK_WIDTH/8 * y0) + x);
        //*(imageBuffer + (E_INK_WIDTH/8 * y0) + x) = ~pixelMaskLUT[x_sub] & temp | (color ? pixelMaskLUT[x_sub] : 0);
        uint8_t temp = *(partialBuffer + (E_INK_WIDTH/8 * y0) + x);
        *(partialBuffer + (E_INK_WIDTH/8 * y0) + x) = ~pixelMaskLUT[x_sub] & temp | (color ? pixelMaskLUT[x_sub] : 0);
    } else {
        color &= 0x0f;
        int x = x0 / 2;
        int x_sub = x0 % 2;
        uint8_t temp;
        //temp = *(D_memory4Bit + E_INK_WIDTH/2 * y0 + x);
        //*(D_memory4Bit + E_INK_WIDTH/2 * y0 + x) = pixelMaskGLUT[x_sub] & temp | (x_sub ? color : color << 4);
        temp = *(imageBuffer + E_INK_WIDTH/2 * y0 + x);
        *(imageBuffer + E_INK_WIDTH/2 * y0 + x) = pixelMaskGLUT[x_sub] & temp | (x_sub ? color : color << 4);
    }
}

void Inkplate::clearDisplay()
{
    // Clear 1 bit per pixel display buffer
    //if (_displayMode == 0) memset(_partial, 0, E_INK_WIDTH * E_INK_HEIGHT/8);
    if (_displayMode == 0) memset(partialBuffer, 0, E_INK_WIDTH * E_INK_HEIGHT/8);
    // Clear 3 bit per pixel display buffer
    if (_displayMode == 1) memset(imageBuffer, 255, E_INK_WIDTH * E_INK_HEIGHT/2);
}

// Function that displays content from RAM to screen
void Inkplate::display()
{
    if (_displayMode == 0) display1b();
    if (_displayMode == 1) display3b();
}

void Inkplate::partialUpdate(uint8_t _leaveOn)
{
    if (_displayMode != 0) return;
    if (_blockPartial == 1)
    {
        display1b();
        return;
    }
    
    einkOn();
    uint8_t *_pos;
    uint8_t *_partialPos;
    uint8_t diffw;
    uint8_t diffb;
    for (int k = 0; k < 10; ++k)
    {
        uint8_t *_pos = imageBuffer + (E_INK_WIDTH * E_INK_HEIGHT / 8) - 1;
        uint8_t *_partialPos = partialBuffer  + (E_INK_WIDTH * E_INK_HEIGHT / 8) - 1;
        vscan_start();
        for (int i = 0; i < 600; ++i)
        {
            diffw = *(_pos) & ~*(_partialPos);                      //Calculate differences in white pixels
            diffb = ~*(_pos) & *(_partialPos);                      //Calculate differences in black pixels    
            --_pos;                                                 //Move address pointers
            --_partialPos;
            hscan_start(LUTW[diffw >> 4] & (LUTB[diffb >> 4]));                     //Start sending first pixel byte to panel
            GPIOD -> BSRR  = (LUTW[diffw & 0x0F] & (LUTB[diffb & 0x0F])) | CL;      //Followed by second one
            GPIOD -> BSRR  = (CL | DATA) << 16;                                     //Clock it!
            for (int j = 0; j < 99; ++j)                                            //Now do all that for whole row
            {
                diffw = *(_pos) & ~*(_partialPos);
                diffb = ~*(_pos) & *(_partialPos);
                --_pos;
                --_partialPos;
                GPIOD -> BSRR = LUTW[diffw >> 4] & (LUTB[diffb >> 4]) | CL;          //First 4 pixels
                GPIOD -> BSRR = (CL | DATA) << 16;
                GPIOD -> BSRR = LUTW[diffw & 0x0F] & (LUTB[diffb & 0x0F]) | CL;      //Last for pixels
                GPIOD -> BSRR = (CL | DATA) << 16;
            }
            GPIOD -> BSRR = CL;                                                 //Clock last bit of data
            GPIOD -> BSRR = (DATA | CL) << 16;
            vscan_end();                                                        //Vrite one row to panel
        }
        delayMicroseconds(230);                                                 //Wait 230uS before new frame
    }

    cleanFast(2, 2);
    cleanFast(3, 1);
    vscan_start();
    if (!_leaveOn) einkOff();
    
    //After update, copy differences to screen buffer
    for (int i = 0; i < (E_INK_WIDTH * E_INK_HEIGHT / 8); i++)
    {
        *(imageBuffer + i) &= *(partialBuffer + i);
        *(imageBuffer + i) |= (*(partialBuffer + i));
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
            drawPixel((j * 2) + _x, i + _y, (*(_p + xSize * (i) + j) >> 4));
            drawPixel((j * 2) + 1 + _x, i + _y, (*(_p + xSize * (i) + j) & 0xff));
        }
        drawPixel((j * 2) + _x, i + _y, (*(_p + xSize * (i) + j) >> 4));
        if (_rem == 0) drawPixel((j * 2) + 1 + _x, i + _y, (*(_p + xSize * (i) + j) & 0xff));
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
    delay(1);
    // Enable all rails
    Wire.beginTransmission(0x48);
    Wire.write(0x01);
    Wire.write(B00111111);
    Wire.endTransmission();
    Wire.beginTransmission(0x48);
    Wire.write(0x03);
    Wire.write(193); // VCOM in mV
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
	//if(_mode != _displayMode) {
		//_displayMode = _mode & 1;
		//memset(D_memory_new, 0, E_INK_WIDTH * E_INK_HEIGHT / 8);
		//memset(_partial, 0, E_INK_WIDTH * E_INK_HEIGHT / 8);
		//memset(_pBuffer, 0, E_INK_WIDTH * E_INK_HEIGHT / 4);
		//memset(D_memory4Bit, 255, E_INK_WIDTH * E_INK_HEIGHT / 2);
		//_blockPartial = 1;
	//}
    _displayMode = _mode & 1;
    _blockPartial = 1;
    if (_displayMode == 0) memset(imageBuffer, 0, E_INK_WIDTH * E_INK_HEIGHT / 8);
    if (_displayMode == 1) memset(imageBuffer, 255, E_INK_WIDTH * E_INK_HEIGHT / 2);
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
    return 0;
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
    delayMicroseconds(1);
    CKV_SET;
    delayMicroseconds(18);
    CKV_CLEAR;
    delayMicroseconds(1);
    CKV_SET;
    delayMicroseconds(18);
}

void Inkplate::hscan_start(uint8_t _d)
{
    SPH_CLEAR;
    GPIOD -> BSRR = (_d) | CL;
    GPIOD -> BSRR = (DATA | CL) << 16;
    SPH_SET;
    CKV_SET;
}

void Inkplate::vscan_end() {
    CKV_CLEAR;
    LE_SET;
    LE_CLEAR;
    //delayMicroseconds(1);
    int32_t start  = dwt_getCycles();
    int32_t cycles = int32_t(0.7 * (SystemCoreClock / 1000000));
    while ((int32_t)dwt_getCycles() - start < cycles);
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
  
    //uint32_t _send = ((data & B00000011) << 4) | (((data & B00001100) >> 2) << 18) | (((data & B00010000) >> 4) << 23) | (((data & B11100000) >> 5) << 25);;
    for (int k = 0; k < rep; ++k)
    {
        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; ++i)
        {
            hscan_start(data);
            GPIOD -> BSRR  = (data) | CL;
            GPIOD -> BSRR  = CL << 16;
            for (int j = 0; j < (E_INK_WIDTH / 8) - 1; ++j)
            {
                GPIOD -> BSRR = CL;
                GPIOD -> BSRR = CL << 16; 
                GPIOD -> BSRR = CL;
                GPIOD -> BSRR = CL << 16;
            }
            GPIOD -> BSRR = (data) | CL;
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
    pinMode(PE4, INPUT);
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
    pinMode(PE4, OUTPUT);
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
    uint8_t *_pos;
    uint8_t data;
    einkOn();
    cleanFast(0, 5);
    
    for (int k = 0; k < 5; ++k)
    {
        _pos = imageBuffer + (E_INK_HEIGHT * E_INK_WIDTH / 8) - 1;
        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; ++i)
        {
        data = LUT2[(~(*_pos) >> 4) & 0x0F];
        hscan_start(data);
        data = LUT2[~(*_pos) & 0x0F];
        GPIOD -> BSRR = data | CL;
        GPIOD -> BSRR = (DATA | CL) << 16;
        --_pos;
        for (int j = 0; j < ((E_INK_WIDTH / 8) - 1); ++j)
        {
            data = LUT2[(~(*_pos) >> 4)&0x0F];
            GPIOD -> BSRR = data | CL;
            GPIOD -> BSRR = (DATA | CL) << 16;
            data = LUT2[~(*_pos) & 0x0F];
            GPIOD -> BSRR = data | CL;
            GPIOD -> BSRR = (DATA | CL) << 16;
            --_pos;
        }
        GPIOD -> BSRR = CL;
        GPIOD -> BSRR = (DATA | CL) << 16;
        vscan_end();
        }
        delayMicroseconds(230);
    }
    cleanFast(1, 5);
    cleanFast(2, 2);
    
    //Full update? Copy everything in screen buffer before refresh!
    for(int i = 0; i<(E_INK_HEIGHT * E_INK_WIDTH) / 8; i++)
    {
        *(imageBuffer + i) &= *(partialBuffer + i);
        *(imageBuffer + i) |= (*(partialBuffer + i));
    }
    
    /*
    cleanFast(0, 20);
    cleanFast(2, 2);
    cleanFast(1, 20);
    cleanFast(2, 2);
    cleanFast(0, 20);
    cleanFast(2, 2);
    cleanFast(1, 20);
    cleanFast(2, 2);
    */
    
    for (int k = 0; k < 20; ++k)
    {
        _pos = imageBuffer + (E_INK_HEIGHT * E_INK_WIDTH / 8) - 1;
        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; ++i)
        {
        data = LUT2[(~(*_pos) >> 4) & 0x0F];
        hscan_start(data);
        data = LUT2[~(*_pos) & 0x0F];
        GPIOD -> BSRR = data | CL;
        GPIOD -> BSRR = (DATA | CL) << 16;
        --_pos;
        for (int j = 0; j < ((E_INK_WIDTH / 8) - 1); ++j)
        {
            data = LUT2[(~(*_pos) >> 4)&0x0F];
            GPIOD -> BSRR = data | CL;
            GPIOD -> BSRR = (DATA | CL) << 16;
            data = LUT2[~(*_pos) & 0x0F];
            GPIOD -> BSRR = data | CL;
            GPIOD -> BSRR = (DATA | CL) << 16;
            --_pos;
        }
        GPIOD -> BSRR = CL;
        GPIOD -> BSRR = (DATA | CL) << 16;
        vscan_end();
        }
        delayMicroseconds(230);
    }
    
    //cleanFast(1, 5);
    cleanFast(2, 2);
    for (int k = 0; k < 20; ++k)
      {
        _pos = imageBuffer + (E_INK_HEIGHT * E_INK_WIDTH / 8) - 1;
        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; ++i)
        {
        data = LUT2[((*_pos) >> 4) & 0x0F];
        hscan_start(data);
        data = LUT2[(*_pos) & 0x0F];
        GPIOD -> BSRR = data | CL;
        GPIOD -> BSRR = (DATA | CL) << 16;
        --_pos;
        for (int j = 0; j < ((E_INK_WIDTH / 8) - 1); ++j)
        {
            data = LUT2[((*_pos) >> 4)&0x0F];
            GPIOD -> BSRR = data | CL;
            GPIOD -> BSRR = (DATA | CL) << 16;
            data = LUT2[(*_pos) & 0x0F];
            GPIOD -> BSRR = data | CL;
            GPIOD -> BSRR = (DATA | CL) << 16;
            --_pos;
        }
        GPIOD -> BSRR = CL;
        GPIOD -> BSRR = (DATA | CL) << 16;
        vscan_end();
        }
        delayMicroseconds(230);
    }
    
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
    cleanFast(0, 20);
    cleanFast(1, 20);
    cleanFast(0, 20);
    cleanFast(1, 20);
    for (int k = 0; k < 16; k++)
    {
        //uint8_t *dp = D_memory4Bit + (E_INK_HEIGHT * E_INK_WIDTH/2);
        uint8_t *dp = imageBuffer + (E_INK_HEIGHT * E_INK_WIDTH/2);
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
    cleanFast(2, 1);
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