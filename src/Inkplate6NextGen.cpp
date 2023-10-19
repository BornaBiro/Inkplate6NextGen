/***************************************************
This library is used for controling ED060SC7 epaper panel on e-radionica's Inkplate6NextGen dev board (we are still
working on it!). If you don't know what Inkplate is, check it out here: https://inkplate.io/

Author: Borna Biro ( https://github.com/BornaBiro/ )
Organization: e-radionica.com / TAVU

This library uses Adafruit GFX library (https://github.com/adafruit/Adafruit-GFX-Library) made by Adafruit Industries.

NOTE: This library is still heavily in progress, so there is still some bugs. Use it on your own risk!
 ****************************************************/

#include "Inkplate6NextGen.h"

    // __attribute__((section(".DTCMRAM_section"))) uint8_t LUT2[16] = {B10101010, B10101001, B10100110, B10100101, B10011010, B10011001, B10010110, B10010101,
    //                           B01101010, B01101001, B01100110, B01100101, B01011010, B01011001, B01010110, B01010101};
    // __attribute__((section(".DTCMRAM_section"))) uint8_t LUTW[16] = {B11111111, B11111110, B11111011, B11111010, B11101111, B11101110, B11101011, B11101010,
    //                           B10111111, B10111110, B10111011, B10111010, B10101111, B10101110, B10101011, B10101010};
    // __attribute__((section(".DTCMRAM_section"))) uint8_t LUTB[16] = {B11111111, B11111101, B11110111, B11110101, B11011111, B11011101, B11010111, B11010101,
    //                           B01111111, B01111101, B01110111, B01110101, B01011111, B01011101, B01010111, B01010101};



SdFat sd(&SPI);

//--------------------------USER FUNCTIONS--------------------------------------------
Inkplate::Inkplate() : Adafruit_GFX(SCREEN_WIDTH, SCREEN_HEIGHT)
{

}

void Inkplate::begin(uint8_t _mode)
{
    if (_beginDone == 1)
        return;
    // Init I2C (Arduino Wire Library).
    Wire.begin();

    // Start the library into 1 bit mode.
    selectDisplayMode(INKPLATE_1BW);

    // Init low level driver for EPD.
    initDriver();

    // Set 16bit ADC resolution for battery measurement.
    analogReadResolution(16);

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

    if (getDisplayMode() == INKPLATE_1BW)
    {
        int x = x0 / 8;
        int x_sub = x0 % 8;

        uint8_t temp = *(partialBuffer + (SCREEN_WIDTH / 8 * y0) + x);
        *(partialBuffer + (SCREEN_WIDTH / 8 * y0) + x) = ~pixelMaskLUT[x_sub] & temp | (color ? pixelMaskLUT[x_sub] : 0);
    }
    else
    {
        color &= 0x0f;
        int x = x0 / 2;
        int x_sub = x0 % 2;
        uint8_t temp;

        temp = *(partialBuffer + SCREEN_WIDTH / 2 * y0 + x);
        *(partialBuffer + SCREEN_WIDTH / 2 * y0 + x) = pixelMaskGLUT1[x_sub] & temp | (x_sub ? color << 4 : color);
    }
}

void Inkplate::drawBitmap3Bit(int16_t _x, int16_t _y, const unsigned char *_p, int16_t _w, int16_t _h)
{
    if (getDisplayMode() != INKPLATE_GL16)
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
        _width = SCREEN_WIDTH;
        _height = SCREEN_HEIGHT;
        break;
    case 1:
    case 3:
        _width = SCREEN_HEIGHT;
        _height = SCREEN_WIDTH;
        break;
    }
}

int Inkplate::drawBitmapFromSD(SdFile *p, int x, int y)
{
    if (sdCardOk == 0)
        return 0;
    struct bitmapHeader bmpHeader;
    readBmpHeader(p, &bmpHeader);
    if (bmpHeader.signature != 0x4D42 || bmpHeader.compression != 0 || !(bmpHeader.color == 1 || bmpHeader.color == 24))
        return 0;

    if ((bmpHeader.color == 24 || bmpHeader.color == 32) && getDisplayMode() != INKPLATE_GL16)
    {
        selectDisplayMode(INKPLATE_GL16);
    }

    if (bmpHeader.color == 1 && getDisplayMode() != INKPLATE_1BW)
    {
        selectDisplayMode(INKPLATE_1BW);
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