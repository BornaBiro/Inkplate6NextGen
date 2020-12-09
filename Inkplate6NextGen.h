/*************************************************** 
This library is used for controling ED060SC7 epaper panel on e-radionica's Inkplate6NextGen dev board (we are still working on it!).
If you don't know what Inkplate is, check it out here: https://inkplate.io/

Author: Borna Biro ( https://github.com/BornaBiro/ )
Organization: e-radionica.com / TAVU

This library uses Adafruit GFX library (https://github.com/adafruit/Adafruit-GFX-Library) made by Adafruit Industries.

NOTE: This library is still heavily in progress, so there is still some bugs. Use it on your own risk!
 ****************************************************/

#ifndef __INKPLATE6NEXTGEN_H__
#define __INKPLATE6NEXTGEN_H__
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Adafruit_GFX.h"
#include "Wire.h"
#include "SPI.h"
#include "SdFat.h"

#define E_INK_WIDTH 		800
#define E_INK_HEIGHT 		600
#define BLACK 				1
#define WHITE 				0
#define INKPLATE_1BIT 		0
#define INKPLATE_3BIT 		1
#define TPS_SDA             14
#define TPS_SCL             15

#define PWR_GOOD_OK         0b11111010

// Fast direct port manipulation defines for epaper panel
#define DATA    		0x000000FF   //PD0 - PD7

#define CL        		GPIO_PIN_11
#define CL_SET    		{GPIOD -> BSRR = CL;}
#define CL_CLEAR  		{GPIOD -> BSRR = CL << 16;}

#define LE        		GPIO_PIN_9
#define LE_SET    		{GPIOC -> BSRR = LE;}
#define LE_CLEAR  		{GPIOC -> BSRR = LE << 16;}

#define CKV       		GPIO_PIN_3
#define CKV_SET   		{GPIOE -> BSRR = CKV;}
#define CKV_CLEAR 		{GPIOE -> BSRR = CKV << 16;}

#define SPH         	GPIO_PIN_4
#define SPH_SET     	{GPIOE -> BSRR = SPH;}
#define SPH_CLEAR   	{GPIOE -> BSRR = SPH << 16;}

#define GMOD            GPIO_PIN_3
#define GMOD_SET    	{GPIOG -> BSRR = GMOD;}
#define GMOD_CLEAR  	{GPIOG -> BSRR = GMOD << 16;}

#define OE          	GPIO_PIN_10
#define OE_SET      	{GPIOC -> BSRR = OE;}
#define OE_CLEAR    	{GPIOC -> BSRR = OE << 16;}

#define SPV         	GPIO_PIN_12
#define SPV_SET     	{GPIOC -> BSRR = SPV;}
#define SPV_CLEAR   	{GPIOC -> BSRR = SPV << 16;}

#define WAKEUP         	GPIO_PIN_9
#define WAKEUP_SET     	{GPIOE -> BSRR = WAKEUP;}
#define WAKEUP_CLEAR   	{GPIOE -> BSRR = WAKEUP << 16;}

#define PWRUP         	GPIO_PIN_12
#define PWRUP_SET     	{GPIOG -> BSRR = PWRUP;}
#define PWRUP_CLEAR   	{GPIOG -> BSRR = PWRUP << 16;}

#define VCOM         	GPIO_PIN_3
#define VCOM_SET     	{GPIOF -> BSRR = VCOM;}
#define VCOM_CLEAR   	{GPIOF -> BSRR = VCOM << 16;}

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

extern SPIClass spi2;
extern SdFat sd;

class Inkplate : public Adafruit_GFX {
  public:
    //uint8_t* D_memory_new;
    //uint8_t* _partial;
    //uint8_t* D_memory4Bit;
    //uint8_t * _pBuffer;
    uint8_t *imageBuffer;
    uint8_t *partialBuffer;
    const uint8_t LUT2[16] = {B10101010, B10101001, B10100110, B10100101, B10011010, B10011001, B10010110, B10010101, B01101010, B01101001, B01100110, B01100101, B01011010, B01011001, B01010110, B01010101};
    const uint8_t LUTW[16] = {B11111111, B11111110, B11111011, B11111010, B11101111, B11101110, B11101011, B11101010, B10111111, B10111110, B10111011, B10111010, B10101111, B10101110, B10101011, B10101010};
    const uint8_t LUTB[16] = {B11111111, B11111101, B11110111, B11110101, B11011111, B11011101, B11010111, B11010101, B01111111, B01111101, B01110111, B01110101, B01011111, B01011101, B01010111, B01010101};
    const uint8_t pixelMaskLUT[8] = {B00000001, B00000010, B00000100, B00001000, B00010000, B00100000, B01000000, B10000000};
    const uint8_t pixelMaskGLUT[2] = {B00001111, B11110000};
    const uint8_t discharge[16] = {B11111111, B11111100, B11110011, B11110000, B11001111, B11001100, B11000011, B11000000, B00111111, B00111100, B00110011, B00110000, B00001111, B00001100, B00000011, B00000000};
    //BLACK->WHITE
    const uint8_t waveform3Bit[16][16] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2}, 
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2}, 
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2}, 
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2}, 
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2}, 
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2}, 
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2}, 
        {0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2}, 
        {0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 
        {0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 
        {0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 
        {0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 
        {0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 
        {0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 
        {0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 
    };
    uint8_t* GLUT;
    uint8_t* GLUT2;
	
	struct bitmapHeader {
		uint16_t signature;
		uint32_t fileSize;
		uint32_t startRAW;
		uint32_t dibHeaderSize;
		uint32_t width;
		uint32_t height;
		uint16_t color;
		uint32_t compression;
	};
  
    Inkplate(uint8_t _mode);
	void begin(void);
    void drawPixel(int16_t x0, int16_t y0, uint16_t color);
    void clearDisplay();
    void display();
    void partialUpdate(uint8_t leaveOn = 0);
	void drawBitmap3Bit(int16_t _x, int16_t _y, const unsigned char* _p, int16_t _w, int16_t _h);
	void setRotation(uint8_t);
    void einkOff(void);
    void einkOn(void);
    uint8_t readPowerGood();
    void selectDisplayMode(uint8_t _mode);
	uint8_t getDisplayMode();
	int drawBitmapFromSD(SdFile* p, int x, int y);
	int drawBitmapFromSD(char* fileName, int x, int y);
	int sdCardInit();
	SdFat getSdFat();
	SPIClass getSPI();
	uint8_t getPanelState();
    void setPanelState(uint8_t);
    uint8_t readTouchpad(uint8_t);
    int8_t readTemperature();
    double readBattery();
	void vscan_start();
	void hscan_start(uint8_t _d = 0);
	void vscan_end();
    void cleanFast(uint8_t c, uint8_t rep);
    void pinsZstate();
    void pinsAsOutputs();

  private:
    int8_t _temperature;
    uint8_t _panelOn = 0;
    uint8_t _rotation = 0;
    uint8_t _displayMode = 0; //By default, 1 bit mode is used
	int sdCardOk = 0;
	uint8_t _blockPartial = 1;
	uint8_t _beginDone = 0;
	
	void display1b();
    void display3b();
	uint32_t read32(uint8_t* c);
	uint16_t read16(uint8_t* c);
	void readBmpHeader(SdFile *_f, struct bitmapHeader *_h);
	int drawMonochromeBitmap(SdFile *f, struct bitmapHeader bmpHeader, int x, int y);
	int drawGrayscaleBitmap24(SdFile *f, struct bitmapHeader bmpHeader, int x, int y);
};

#endif
