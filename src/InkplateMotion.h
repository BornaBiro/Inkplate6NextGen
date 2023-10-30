/***************************************************
Author: Borna Biro ( https://github.com/BornaBiro/ )
Organization: Soldered Electronics.com

This library uses Adafruit GFX library (https://github.com/adafruit/Adafruit-GFX-Library) made by Adafruit Industries.

NOTE: This library is still heavily in progress, so there is still some bugs. Use it on your own risk!
 ****************************************************/

#ifndef __INKPLATEMOTION_H__
#define __INKPLATEMOTION_H__

// Include main Arduino Header file.
#include "Arduino.h"

// Include files for Inkplate driver
#include "system/Inkplate_Boards.h"

// Include Adafruit GFX, I2C and SPI libary
#include "Adafruit_GFX.h"
#include "SPI.h"
#include "Wire.h"

// Include SDFat library
#include "SdFat.h"

// Include library defines
#include "system/defines.h"

// Clases for SPI driver for microSD.
extern SPIClass spi2;
extern SdFat sd;

class Inkplate : public Adafruit_GFX, public EPDDriver
{
  public:
    struct bitmapHeader
    {
        uint16_t signature;
        uint32_t fileSize;
        uint32_t startRAW;
        uint32_t dibHeaderSize;
        uint32_t width;
        uint32_t height;
        uint16_t color;
        uint32_t compression;
    };

    Inkplate();
    void begin(uint8_t _mode = INKPLATE_1BW);
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    void setRotation(uint8_t);
    int drawBitmapFromSD(SdFile *p, int x, int y);
    int drawBitmapFromSD(char *fileName, int x, int y);
    void drawBitmap4Bit(int16_t _x, int16_t _y, const unsigned char *_p, int16_t _w, int16_t _h);
    int sdCardInit();
    SdFat getSdFat();
    SPIClass getSPI();

  protected:


  private:
    uint8_t _rotation = 0;
    int sdCardOk = 0;
    uint8_t _beginDone = 0;

    uint32_t read32(uint8_t *c);
    uint16_t read16(uint8_t *c);
    void readBmpHeader(SdFile *_f, struct bitmapHeader *_h);
    int drawMonochromeBitmap(SdFile *f, struct bitmapHeader bmpHeader, int x, int y);
    int drawGrayscaleBitmap24(SdFile *f, struct bitmapHeader bmpHeader, int x, int y);
};

#endif
