/***************************************************
This library is used for controling ED060SC7 epaper panel on e-radionica's Inkplate6NextGen dev board (we are still
working on it!). If you don't know what Inkplate is, check it out here: https://inkplate.io/

Author: Borna Biro ( https://github.com/BornaBiro/ )
Organization: e-radionica.com / TAVU

This library uses Adafruit GFX library (https://github.com/adafruit/Adafruit-GFX-Library) made by Adafruit Industries.

NOTE: This library is still heavily in progress, so there is still some bugs. Use it on your own risk!
 ****************************************************/

#include <stdlib.h>
#include "Inkplate6NextGen.h"

    // __attribute__((section(".DTCMRAM_section"))) uint8_t LUT2[16] = {B10101010, B10101001, B10100110, B10100101, B10011010, B10011001, B10010110, B10010101,
    //                           B01101010, B01101001, B01100110, B01100101, B01011010, B01011001, B01010110, B01010101};
    // __attribute__((section(".DTCMRAM_section"))) uint8_t LUTW[16] = {B11111111, B11111110, B11111011, B11111010, B11101111, B11101110, B11101011, B11101010,
    //                           B10111111, B10111110, B10111011, B10111010, B10101111, B10101110, B10101011, B10101010};
    // __attribute__((section(".DTCMRAM_section"))) uint8_t LUTB[16] = {B11111111, B11111101, B11110111, B11110101, B11011111, B11011101, B11010111, B11010101,
    //                           B01111111, B01111101, B01110111, B01110101, B01011111, B01011101, B01010111, B01010101};



// SPIClass spi2(HSPI);
SdFat sd(&SPI);
SRAM_HandleTypeDef hsram1;
SRAM_HandleTypeDef hsram2;
static uint32_t FMC_Initialized = 0;
static uint32_t FMC_DeInitialized = 0;

//--------------------------USER FUNCTIONS--------------------------------------------
Inkplate::Inkplate(uint8_t _mode) : Adafruit_GFX(E_INK_WIDTH, E_INK_HEIGHT)
{
    _displayMode = _mode;
}

void Inkplate::begin(void)
{
    if (_beginDone == 1)
        return;

    // For some reason, HAL doesn't initalize GPIOF properly, so it has to be done with Arduino pinMode() function.
    pinMode(PF0, OUTPUT);

    // Enable the external RAM (inverse logic due P-MOS)
    pinMode(RAM_EN, OUTPUT);
    digitalWrite(RAM_EN, LOW);

    // Init STM32 FMC (Flexible memory controller) for faster pushing data to panel using Hardware (similar to ESP32 I2S
    // Parallel, but much faster and better).
    stm32FmcInit();

    // Init I2C (Arduino Wire Library).
    Wire.begin();

    // Set TPS control pins to outputs.
    pinMode(TPS_PWR_PIN, OUTPUT);
    pinMode(TPS_WAKE_PIN, OUTPUT);
    pinMode(TPS_VCOMCTRL_PIN, OUTPUT);

    // Send proper power up sequence for EPD.
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

    // CONTROL PINS
    // PD11 -> Clock pin of EPD. Not used anymore, because of FMC
    // pinMode(PD11, OUTPUT);
    // PAXY Not Defined in Arduino IDE because of USB OTG and Ethernet on Nucleo Board! Switched to PB & PG port.
    pinMode(EPD_CKV, OUTPUT);
    pinMode(EPD_SPV, OUTPUT);
    pinMode(EPD_SPH, OUTPUT);
    pinMode(EPD_OE, OUTPUT);
    pinMode(EPD_GMODE, OUTPUT);
    pinMode(EPD_LE, OUTPUT);

    // Make fast Grayscale LUT to convert 4 pixels into proper waveform data for EPD.
    GLUT = (uint8_t *)malloc(256 * 15 * sizeof(uint8_t));
    GLUT2 = (uint8_t *)malloc(256 * 15 * sizeof(uint8_t));

    for (int j = 0; j < 15; ++j)
    {
        for (uint32_t i = 0; i < 256; ++i)
        {
            GLUT[j * 256 + i] = (waveform3Bit2[i & 0x0f][j] << 2) | (waveform3Bit2[(i >> 4) & 0x0f][j]);
            GLUT2[j * 256 + i] = ((waveform3Bit2[i & 0x0f][j] << 2) | (waveform3Bit2[(i >> 4) & 0x0f][j])) << 4;
        }
    }

    // Clean frame buffers.
    clearDisplay();

    // Block multiple inits.
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

    if (_displayMode == 0)
    {
        int x = x0 / 8;
        int x_sub = x0 % 8;

        uint8_t temp = *(partialBuffer + (E_INK_WIDTH / 8 * y0) + x);
        *(partialBuffer + (E_INK_WIDTH / 8 * y0) + x) = ~pixelMaskLUT[x_sub] & temp | (color ? pixelMaskLUT[x_sub] : 0);
    }
    else
    {
        color &= 0x0f;
        int x = x0 / 2;
        int x_sub = x0 % 2;
        uint8_t temp;

        temp = *(partialBuffer + E_INK_WIDTH / 2 * y0 + x);
        *(partialBuffer + E_INK_WIDTH / 2 * y0 + x) = pixelMaskGLUT[x_sub] & temp | (x_sub ? color << 4 : color);
    }
}

void Inkplate::clearDisplay()
{
    if (_displayMode == 0)
    {
        for (int i = 0; i < (E_INK_HEIGHT * E_INK_WIDTH / 8); i++)
        {
            partialBuffer[i] = 0;
        }
    }

    if (_displayMode == 1)
    {
        for (int i = 0; i < (E_INK_HEIGHT * E_INK_WIDTH / 2); i++)
        {
            partialBuffer[i] = 255;
        }
    }
}

// Function that displays content from RAM to screen
void Inkplate::display(uint8_t _leaveOn)
{
    if (_displayMode == 0)
        display1b(_leaveOn);
    if (_displayMode == 1)
        display3b(_leaveOn);
}

void Inkplate::partialUpdate(uint8_t _leaveOn, uint16_t startRowPos, uint16_t endRowPos)
{
    if (_displayMode != 0)
        return;
    if (_blockPartial == 1)
    {
        display1b(_leaveOn);
        return;
    }

    einkOn();
    __IO uint8_t *_pos;
    __IO uint8_t *_partialPos;
    uint8_t diffw;
    uint8_t diffb;
    for (int k = 0; k < 7; k++)
    {
        vscan_start();
        _pos = imageBuffer;
        _partialPos = partialBuffer;

        for (int i = 0; i < E_INK_HEIGHT; i++)
        {
            diffw = *(_pos) & ~*(_partialPos); // Calculate differences in white pixels
            diffb = ~*(_pos) & *(_partialPos); // Calculate differences in black pixels
            _pos++;                            // Move address pointers
            _partialPos++;
            hscan_start(LUTW[diffw >> 4] & (LUTB[diffb >> 4]),
                        LUTW[diffw & 0x0F] &
                            (LUTB[diffb & 0x0F]));          // Start sending first pixel byte to panel //Clock it!
            for (int j = 0; j < (E_INK_WIDTH / 8) - 1; j++) // Now do all that for whole row
            {
                diffw = *(_pos) & ~*(_partialPos);
                diffb = ~*(_pos) & *(_partialPos);
                _pos++;
                _partialPos++;
                *(__IO uint8_t *)(FMC_ADDRESS) = LUTW[diffw >> 4] & (LUTB[diffb >> 4]);
                *(__IO uint8_t *)(FMC_ADDRESS) = LUTW[diffw & 0x0F] & (LUTB[diffb & 0x0F]);
            }
            vscan_end(); // Vrite one row to panel
        }
    }

    cleanFast(2, 1);
    if (!_leaveOn)
        einkOff();

    // After update, copy differences to screen buffer
    for (int i = 0; i < (E_INK_HEIGHT * E_INK_WIDTH / 8); i++)
    {
        *(imageBuffer + i) = *(partialBuffer + i);
    }
}

void Inkplate::partialUpdate4Bit(uint8_t _leaveOn)
{
    // TO-DO!!! Do not alloce whole user RAM just to find the difference mask. Do it line by line!
    // And also check why image is darker on partial update compared to the full grayscale update!
    // Anyhow, this must be optimised!
    const uint8_t _cleanWaveform[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2,
                                      2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2};
    uint8_t _panelMask[E_INK_HEIGHT * E_INK_WIDTH / 4];
    uint8_t *_pPanelMask = _panelMask;
    memset(_panelMask, 0, sizeof(_panelMask));

    // We present you the ugliest code ever...needs A LOT of clean up and optimisation!
    // Create the mask for pixel difference
    __IO uint8_t *_pPartial = partialBuffer;
    __IO uint8_t *_pImage = imageBuffer;
    for (int i = 0; i < E_INK_HEIGHT; i++)
    {
        for (int j = 0; j < (E_INK_WIDTH / 4); j++) // Now do all that for whole row
        {
            uint8_t _diff = *_pPartial++ ^ *_pImage++;
            if (_diff & (0xf0))
                *_pPanelMask |= 0x30;
            if (_diff & (0x0f))
                *_pPanelMask |= 0xc0;
            _diff = *_pPartial++ ^ *_pImage++;
            if (_diff & (0xf0))
                *_pPanelMask |= 0x03;
            if (_diff & (0x0f))
                *_pPanelMask |= 0x0c;
            _pPanelMask++;
        }
    }

    for (int k = 0; k < 60; k++)
    {
        _pPanelMask = _panelMask;
        uint8_t _color;

        switch (_cleanWaveform[k])
        {
        case 0:
            _color = B10101010;
            break;
        case 1:
            _color = B01010101;
            break;
        case 2:
            _color = B00000000;
            break;
        case 3:
            _color = B11111111;
            break;
        }

        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; i++)
        {
            hscan_start(_color & *_pPanelMask++, _color & *_pPanelMask++); // Start sending first pixel byte to panel
            for (int j = 0; j < (E_INK_WIDTH / 8) - 1; j++)                // Now do all that for whole row
            {
                *(__IO uint8_t *)(FMC_ADDRESS) = _color & *_pPanelMask++;
                *(__IO uint8_t *)(FMC_ADDRESS) = _color & *_pPanelMask++;
            }
            vscan_end(); // Write one row to panel
        }
        delayMicroseconds(230); // Wait 230uS before new frame
    }

    for (int k = 0; k < 15; k++)
    {
        __IO uint8_t *dp = partialBuffer;
        _pPanelMask = _panelMask;
        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; i++)
        {
            hscan_start((GLUT2[k * 256 + (*(dp++))] | GLUT[k * 256 + (*(dp++))]),
                        (GLUT2[k * 256 + (*(dp++))] | GLUT[k * 256 + (*(dp++))]));
            for (int j = 0; j < ((E_INK_WIDTH / 8)) - 1; j++)
            {

                *(__IO uint8_t *)(FMC_ADDRESS) = (GLUT2[k * 256 + (*(dp++))] | GLUT[k * 256 + (*(dp++))]);
                *(__IO uint8_t *)(FMC_ADDRESS) = (GLUT2[k * 256 + (*(dp++))] | GLUT[k * 256 + (*(dp++))]);
            }
            vscan_end();
        }
        delayMicroseconds(230);
    }
    cleanFast(2, 2);
    cleanFast(3, 1);

    // After update, copy differences to screen buffer
    for (int i = 0; i < (E_INK_WIDTH * E_INK_HEIGHT / 2); i++)
    {
        *(imageBuffer + i) &= *(partialBuffer + i);
        *(imageBuffer + i) |= *(partialBuffer + i);
    }
}

void Inkplate::drawBitmap3Bit(int16_t _x, int16_t _y, const unsigned char *_p, int16_t _w, int16_t _h)
{
    if (_displayMode != INKPLATE_3BIT)
        return;
    uint8_t _rem = _w % 2;
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
        if (_rem == 0)
            drawPixel((j * 2) + 1 + _x, i + _y, (*(_p + xSize * (i) + j) & 0xff));
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

// Turn off epapewr supply and put all digital IO pins in high Z state
//  Turn off epaper power supply and put all digital IO pins in high Z state
void Inkplate::einkOff()
{
    if (getPanelState() == 0)
        return;
    OE_CLEAR;
    GMOD_CLEAR;
    CKV_CLEAR;
    SPH_CLEAR;
    SPV_CLEAR;
    LE_CLEAR;
    VCOM_CLEAR;
    PWRUP_CLEAR;
    WAKEUP_CLEAR;

    unsigned long timer = millis();
    do
    {
        delay(1);
    } while ((readPowerGood() != 0) && (millis() - timer) < 500);

    // pinsZstate();
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
    pinsAsOutputs();
    LE_CLEAR;
    SPH_SET;
    GMOD_SET;
    SPV_SET;
    CKV_CLEAR;
    OE_SET;
    VCOM_SET;

    unsigned long timer = millis();
    do
    {
        delay(1);
    } while ((readPowerGood() != PWR_GOOD_OK) && (millis() - timer) < 250);
    if ((millis() - timer) >= 250)
    {
        WAKEUP_CLEAR;
        VCOM_CLEAR;
        PWRUP_CLEAR;
        return;
    }

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
    _displayMode = _mode & 1;
    _blockPartial = 1;
}

uint8_t Inkplate::getDisplayMode()
{
    return _displayMode;
}

int Inkplate::drawBitmapFromSD(SdFile *p, int x, int y)
{
    if (sdCardOk == 0)
        return 0;
    struct bitmapHeader bmpHeader;
    readBmpHeader(p, &bmpHeader);
    if (bmpHeader.signature != 0x4D42 || bmpHeader.compression != 0 || !(bmpHeader.color == 1 || bmpHeader.color == 24))
        return 0;

    if ((bmpHeader.color == 24 || bmpHeader.color == 32) && getDisplayMode() != INKPLATE_3BIT)
    {
        selectDisplayMode(INKPLATE_3BIT);
    }

    if (bmpHeader.color == 1 && getDisplayMode() != INKPLATE_1BIT)
    {
        selectDisplayMode(INKPLATE_1BIT);
    }

    if (bmpHeader.color == 1)
        drawMonochromeBitmap(p, bmpHeader, x, y);
    if (bmpHeader.color == 24)
        drawGrayscaleBitmap24(p, bmpHeader, x, y);

    return 1;
}

int Inkplate::drawBitmapFromSD(char *fileName, int x, int y)
{
    if (sdCardOk == 0)
        return 0;
    SdFile dat;
    if (dat.open(fileName, O_RDONLY))
    {
        return drawBitmapFromSD(&dat, x, y);
    }
    else
    {
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
    if (getPanelState() == 0)
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
    if (getPanelState() == 0)
    {
        PWRUP_CLEAR;
        WAKEUP_CLEAR;
        delay(5);
    }
    return temp;
}

double Inkplate::readBattery()
{
    pinMode(PE2, OUTPUT);
    digitalWrite(PE2, HIGH);
    delay(5);
    int _rawADC = analogRead(PF10);
    digitalWrite(PE2, LOW);

    return (double)(_rawADC / 1023.0 * 3.3 * 2);
}

//--------------------------LOW LEVEL STUFF--------------------------------------------


void Inkplate::rowSkip(uint16_t _n)
{
    for (int i = 0; i < _n; i++)
    {
        CKV_CLEAR;
        delayUS(1);
        CKV_SET;
        delayUS(1);
    }
}

// Clears content from epaper diplay as fast as ESP32 can.
void Inkplate::cleanFast(uint8_t c, uint8_t rep)
{
    einkOn();
    uint8_t data;
    if (c == 0)
    {
        data = B10101010; // White
    }
    else if (c == 1)
    {
        data = B01010101; // Black
    }
    else if (c == 2)
    {
        data = B00000000; // Discharge
    }
    else if (c == 3)
    {
        data = B11111111; // Skip
    }

    // memset(oneRow, data, 200);

    // uint32_t _send = ((data & B00000011) << 4) | (((data & B00001100) >> 2) << 18) | (((data & B00010000) >> 4) <<
    // 23) | (((data & B11100000) >> 5) << 25);;
    for (int k = 0; k < rep; k++)
    {
        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; i++)
        {
            hscan_start(data, data);
            for (int j = 0; j < (E_INK_WIDTH / 8) - 1; j++)
            {
                //  Do not use HAL. it's way slower. Use direct writing data to memory
                *(__IO uint8_t *)(FMC_ADDRESS) = data;
                *(__IO uint8_t *)(FMC_ADDRESS) = data;
            }

            vscan_end();
        }

        delayMicroseconds(230);
    }
}

void Inkplate::pinsZstate()
{
    // CONTROL PINS
    // pinMode(PD11, INPUT);
    // PAXY Not Defined in Arduino IDE because of USB OTG and Ethernet on Nucleo Board! Switched to PB & PG port.

    pinMode(PA0, INPUT);  // EPD_CKV
    pinMode(PA1, INPUT);  // EPD_SPV
    pinMode(PA2, INPUT);  // EPD_SPH
    pinMode(PA8, INPUT);  // EPD_OE
    pinMode(PB6, INPUT);  // EPD_GMODE
    pinMode(PB15, INPUT); // EPD_LE
}

void Inkplate::pinsAsOutputs()
{
    // CONTROL PINS
    // pinMode(PD11, OUTPUT);
    // PAXY Not Defined in Arduino IDE because of USB OTG and Ethernet on Nucleo Board! Switched to PB & PG port.

    pinMode(PA0, OUTPUT);  // EPD_CKV
    pinMode(PA1, OUTPUT);  // EPD_SPV
    pinMode(PA2, OUTPUT);  // EPD_SPH
    pinMode(PA8, OUTPUT);  // EPD_OE
    pinMode(PB6, OUTPUT);  // EPD_GMODE
    pinMode(PB15, OUTPUT); // EPD_LE
}

//--------------------------PRIVATE FUNCTIONS--------------------------------------------
// Display content from RAM to display (1 bit per pixel,. monochrome picture).
void Inkplate::display1b(uint8_t _leaveOn)
{
    __IO uint8_t *_pos;
    uint8_t data;
    einkOn();
    cleanFast(0, 5);

    for (int k = 0; k < 10; k++)
    {
        _pos = partialBuffer;
        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; i++)
        {
            hscan_start(LUT2[(~(*_pos) >> 4) & 0x0F], LUT2[~(*_pos) & 0x0F]);
            _pos++;
            for (int j = 0; j < ((E_INK_WIDTH / 8) - 1); j++)
            {
                *(__IO uint8_t *)(FMC_ADDRESS) = LUT2[(~(*_pos) >> 4) & 0x0F];
                *(__IO uint8_t *)(FMC_ADDRESS) = LUT2[~(*_pos) & 0x0F];
                _pos++;
            }
            vscan_end();
        }
        delayMicroseconds(230);
    }
    cleanFast(1, 10);
    cleanFast(2, 1);

    // Full update? Copy everything in screen buffer before refresh!
    for (int i = 0; i < (E_INK_HEIGHT * E_INK_WIDTH) / 8; i++)
    {
        imageBuffer[i] &= partialBuffer[i];
        imageBuffer[i] |= partialBuffer[i];
    }

    for (int k = 0; k < 30; k++)
    {
        _pos = imageBuffer;
        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; i++)
        {
            hscan_start(LUT2[(~(*_pos) >> 4) & 0x0F], LUT2[~(*_pos) & 0x0F]);
            _pos++;
            for (int j = 0; j < ((E_INK_WIDTH / 8) - 1); j++)
            {
                *(__IO uint8_t *)(FMC_ADDRESS) = LUT2[(~(*_pos) >> 4) & 0x0F];
                *(__IO uint8_t *)(FMC_ADDRESS) = LUT2[~(*_pos) & 0x0F];
                _pos++;
            }
            vscan_end();
        }
        delayMicroseconds(230);
    }

    cleanFast(2, 1);
    for (int k = 0; k < 30; k++)
    {
        _pos = imageBuffer;
        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; i++)
        {
            hscan_start(LUT2[((*_pos) >> 4) & 0x0F], LUT2[(*_pos) & 0x0F]);
            _pos++;
            for (int j = 0; j < ((E_INK_WIDTH / 8) - 1); j++)
            {
                *(__IO uint8_t *)(FMC_ADDRESS) = LUT2[((*_pos) >> 4) & 0x0F];
                *(__IO uint8_t *)(FMC_ADDRESS) = LUT2[(*_pos) & 0x0F];
                _pos++;
            }
            vscan_end();
        }
        delayMicroseconds(230);
    }

    cleanFast(2, 1);
    vscan_start();
    if (!_leaveOn)
        einkOff();
    _blockPartial = 0;
}

// Display content from RAM to display (3 bit per pixel,. 8 level of grayscale, STILL IN PROGRESSS, we need correct
// wavefrom to get good picture, use it only for pictures not for GFX).
void Inkplate::display3b(uint8_t _leaveOn)
{
    DEBUG_MSG("display3 bit start");

    // Full update? Copy everything in screen buffer before refresh!
    for (int i = 0; i < (E_INK_HEIGHT * E_INK_WIDTH) / 2; i++)
    {
        imageBuffer[i] = partialBuffer[i];
    }

    einkOn();

    cleanFast(0, 10);
    cleanFast(1, 30);
    cleanFast(0, 30);
    cleanFast(1, 30);
    cleanFast(0, 30);
    cleanFast(2, 1);

    uint8_t _oneLine[800];



    for (int k = 0; k < 15; k++)
    {
        __IO uint8_t *dp = partialBuffer;
        vscan_start();
        for (int i = 0; i < E_INK_HEIGHT; i++)
        {
            hscan_start((GLUT2[k * 256 + (*(dp++))] | GLUT[k * 256 + (*(dp++))]),
                        (GLUT2[k * 256 + (*(dp++))] | GLUT[k * 256 + (*(dp++))]));
            for (int j = 0; j < ((E_INK_WIDTH / 8)) - 1; j++)
            {
                *(__IO uint8_t *)(FMC_ADDRESS) = GLUT2[k * 256 + (*(dp++))] | GLUT[k * 256 + (*(dp++))];
                *(__IO uint8_t *)(FMC_ADDRESS) = GLUT2[k * 256 + (*(dp++))] | GLUT[k * 256 + (*(dp++))];
            }
            vscan_end();
        }
        delayMicroseconds(230);
    }
    cleanFast(2, 2);
    cleanFast(3, 1);
    // vscan_start();
    if (!_leaveOn)
        einkOff();
    
    DEBUG_MSG("display3 bit done");
}

uint32_t Inkplate::read32(uint8_t *c)
{
    return (*(c) | (*(c + 1) << 8) | (*(c + 2) << 16) | (*(c + 3) << 24));
}

uint16_t Inkplate::read16(uint8_t *c)
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
    for (j = 0; j < h; j++)
    {
        for (i = 0; i < w; i++)
        {
            uint32_t pixelRow = f->read() << 24 | f->read() << 16 | f->read() << 8 | f->read();
            for (int n = 0; n < 32; n++)
            {
                drawPixel((i * 32) + n + x, h - j + y, !(pixelRow & (1ULL << (31 - n))));
            }
        }
        if (paddingBits)
        {
            uint32_t pixelRow = f->read() << 24 | f->read() << 16 | f->read() << 8 | f->read();
            for (int n = 0; n < paddingBits; n++)
            {
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
    for (j = 0; j < h; j++)
    {
        for (i = 0; i < w; i++)
        {
            // This is the proper way of converting True Color (24 Bit RGB) bitmap file into grayscale, but it takes
            // waaay too much time (full size picture takes about 17s to decode!) float px = (0.2126 *
            // (readByteFromSD(&file) / 255.0)) + (0.7152 * (readByteFromSD(&file) / 255.0)) + (0.0722 *
            // (readByteFromSD(&file) / 255.0)); px = pow(px, 1.5); display.drawPixel(i + x, h - j + y,
            // (uint8_t)(px*7));

            // So then, we are convertng it to grayscale. With this metod, it is still slow (full size image takes 4
            // seconds), but much beter than prev mentioned method.
            uint8_t px = (f->read() * 2126 / 10000) + (f->read() * 7152 / 10000) + (f->read() * 722 / 10000);
            drawPixel(i + x, h - j + y, px >> 5);
        }
        if (padding)
        {
            for (int p = 0; p < padding; p++)
            {
                f->read();
            }
        }
    }
    f->close();
    return 1;
}

// Really low level STM32 related stuff. Do not change anything unless you really know what you are doing!

/* FMC initialization function */
static void MX_FMC_Init(void)
{
    /* USER CODE BEGIN FMC_Init 0 */

    /* USER CODE END FMC_Init 0 */

    FMC_NORSRAM_TimingTypeDef Timing = {0};
    FMC_NORSRAM_TimingTypeDef ExtTiming = {0};

    /* USER CODE BEGIN FMC_Init 1 */

    /* USER CODE END FMC_Init 1 */

    /** Perform the SRAM1 memory initialization sequence
     */
    hsram1.Instance = FMC_NORSRAM_DEVICE;
    hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
    /* hsram1.Init */
    hsram1.Init.NSBank = FMC_NORSRAM_BANK3;
    hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
    hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
    hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_8;
    hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
    hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
    hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
    hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
    hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
    hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
    hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
    hsram1.Init.WriteFifo = FMC_WRITE_FIFO_DISABLE;
    hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
    /* Timing */
    Timing.AddressSetupTime = 0;
    Timing.AddressHoldTime = 0;
    Timing.DataSetupTime = 3; // ED060KC1
    Timing.BusTurnAroundDuration = 0;
    Timing.CLKDivision = 16;
    Timing.DataLatency = 17;
    Timing.AccessMode = FMC_ACCESS_MODE_A;
    /* ExtTiming */

    if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
    {
        Error_Handler();
    }

    /** Perform the SRAM2 memory initialization sequence
     */
    hsram2.Instance = FMC_NORSRAM_DEVICE;
    hsram2.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
    /* hsram2.Init */
    hsram2.Init.NSBank = FMC_NORSRAM_BANK1;
    hsram2.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
    hsram2.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
    hsram2.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
    hsram2.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
    hsram2.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram2.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
    hsram2.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
    hsram2.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
    hsram2.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
    hsram2.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram2.Init.WriteBurst = FMC_WRITE_BURST_ENABLE;
    hsram2.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
    hsram2.Init.WriteFifo = FMC_WRITE_FIFO_DISABLE;
    hsram2.Init.PageSize = FMC_PAGE_SIZE_NONE;
    /* Timing */
    Timing.AddressSetupTime = 3;
    Timing.AddressHoldTime = 0;
    Timing.DataSetupTime = 2;
    Timing.BusTurnAroundDuration = 0;
    Timing.CLKDivision = 0;
    Timing.DataLatency = 0;
    Timing.AccessMode = FMC_ACCESS_MODE_D;
    /* ExtTiming - Write only */
    ExtTiming.AddressSetupTime = 3;
    ExtTiming.AddressHoldTime = 0;
    ExtTiming.DataSetupTime = 1;
    ExtTiming.BusTurnAroundDuration = 0;
    ExtTiming.CLKDivision = 0;
    ExtTiming.DataLatency = 0;
    ExtTiming.AccessMode = FMC_ACCESS_MODE_D;

    if (HAL_SRAM_Init(&hsram2, &Timing, NULL) != HAL_OK)
    {
        Error_Handler();
    }
}

static void HAL_FMC_MspInit(void)
{
    /* USER CODE BEGIN FMC_MspInit 0 */

    /* USER CODE END FMC_MspInit 0 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (FMC_Initialized)
    {
        return;
    }
    FMC_Initialized = 1;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FMC;
    PeriphClkInitStruct.PLL2.PLL2M = 8;
    PeriphClkInitStruct.PLL2.PLL2N = 80;
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

  /* Peripheral clock enable */
  __HAL_RCC_FMC_CLK_ENABLE();

    /** FMC GPIO Configuration
    PE3   ------> FMC_A19
    PF0   ------> FMC_A0
    PF1   ------> FMC_A1
    PF2   ------> FMC_A2
    PF3   ------> FMC_A3
    PF4   ------> FMC_A4
    PF5   ------> FMC_A5
    PF12   ------> FMC_A6
    PF13   ------> FMC_A7
    PF14   ------> FMC_A8
    PF15   ------> FMC_A9
    PG0   ------> FMC_A10
    PG1   ------> FMC_A11
    PE7   ------> FMC_D4
    PE8   ------> FMC_D5
    PE9   ------> FMC_D6
    PE10   ------> FMC_D7
    PE11   ------> FMC_D8
    PE12   ------> FMC_D9
    PE13   ------> FMC_D10
    PE14   ------> FMC_D11
    PE15   ------> FMC_D12
    PD8   ------> FMC_D13
    PD9   ------> FMC_D14
    PD10   ------> FMC_D15
    PD11   ------> FMC_A16
    PD12   ------> FMC_A17
    PD13   ------> FMC_A18
    PD14   ------> FMC_D0
    PD15   ------> FMC_D1
    PG2   ------> FMC_A12
    PG3   ------> FMC_A13
    PG4   ------> FMC_A14
    PG5   ------> FMC_A15
    PG6   ------> FMC_NE3
    PC7   ------> FMC_NE1
    PD0   ------> FMC_D2
    PD1   ------> FMC_D3
    PD4   ------> FMC_NOE
    PD5   ------> FMC_NWE
    PE0   ------> FMC_NBL0
    PE1   ------> FMC_NBL1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                          GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12 |
                          GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                          GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FMC;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN FMC_MspInit 1 */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    /* USER CODE END FMC_MspInit 1 */
}

extern "C" void HAL_SRAM_MspInit(SRAM_HandleTypeDef *hsram)
{
    HAL_FMC_MspInit();
}

static void HAL_FMC_MspDeInit(void)
{
    /* USER CODE BEGIN FMC_MspDeInit 0 */

    /* USER CODE END FMC_MspDeInit 0 */
    if (FMC_DeInitialized)
    {
        return;
    }
    FMC_DeInitialized = 1;
    /* Peripheral clock enable */
    __HAL_RCC_FMC_CLK_DISABLE();

    /** FMC GPIO Configuration
    PE3   ------> FMC_A19
    PF0   ------> FMC_A0
    PF1   ------> FMC_A1
    PF2   ------> FMC_A2
    PF3   ------> FMC_A3
    PF4   ------> FMC_A4
    PF5   ------> FMC_A5
    PF12   ------> FMC_A6
    PF13   ------> FMC_A7
    PF14   ------> FMC_A8
    PF15   ------> FMC_A9
    PG0   ------> FMC_A10
    PG1   ------> FMC_A11
    PE7   ------> FMC_D4
    PE8   ------> FMC_D5
    PE9   ------> FMC_D6
    PE10   ------> FMC_D7
    PE11   ------> FMC_D8
    PE12   ------> FMC_D9
    PE13   ------> FMC_D10
    PE14   ------> FMC_D11
    PE15   ------> FMC_D12
    PD8   ------> FMC_D13
    PD9   ------> FMC_D14
    PD10   ------> FMC_D15
    PD11   ------> FMC_A16
    PD12   ------> FMC_A17
    PD13   ------> FMC_A18
    PD14   ------> FMC_D0
    PD15   ------> FMC_D1
    PG2   ------> FMC_A12
    PG3   ------> FMC_A13
    PG4   ------> FMC_A14
    PG5   ------> FMC_A15
    PG6   ------> FMC_NE3
    PC7   ------> FMC_NE1
    PD0   ------> FMC_D2
    PD1   ------> FMC_D3
    PD4   ------> FMC_NOE
    PD5   ------> FMC_NWE
    PE0   ------> FMC_NBL0
    PE1   ------> FMC_NBL1
    PE4   ------> FMC_A20
    PE5   ------> FMC_A21
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                               GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 |
                               GPIO_PIN_1);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12 |
                               GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                               GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_7);

    /* USER CODE BEGIN FMC_MspDeInit 1 */

    /* USER CODE END FMC_MspDeInit 1 */
}

extern "C" void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef *hsram)
{
    HAL_FMC_MspDeInit();
}
void Inkplate::stm32FmcInit()
{
    // Enable clock to GPIOs
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    // Enable clock to FMC
    __HAL_RCC_FMC_CLK_ENABLE();

    // Init FMC
    MX_FMC_Init();

    /* L1 cache issue. On Atollic, STM32 was sending extra bytes (8 instead 1), on Arduino FMC did not work at all!
     * https://community.st.com/s/question/0D50X0000C9hD8D/stm32f746-fmc-configured-for-8bit-parallel-interface-extra-bytes-sent
     * https://community.st.com/s/question/0D50X00009XkWQE/stm32h743ii-fmc-8080-lcd-spurious-writes
     * https://stackoverflow.com/questions/59198934/l1-cache-behaviour-of-stm32h7
     *
     * It can be fixed by enabling HAL_SetFMCMemorySwappingConfig(FMC_SWAPBMAP_SDRAM_SRAM); but this will hurt SRAM R/W
     * performace! Real workaround is to disable cache on LCD memory allocation with MPU (Memory Protection Unit).
     */
    mpuFMCPatch();
}

/**
 * @brief       It disables cacheing on LCD FMC memory area, but not affecting caching on SRAM by using MPU.
 *
 */
void Inkplate::mpuFMCPatch()
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    HAL_MPU_Disable();
    // Disable only cache on LCD interface, NOT SRAM!
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x68000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_64MB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}