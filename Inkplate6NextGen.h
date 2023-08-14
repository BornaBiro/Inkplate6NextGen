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
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_sram.h"

// ED060SC7
#define E_INK_WIDTH 		800ULL
#define E_INK_HEIGHT 		600ULL

// ED060KC1
//#define E_INK_WIDTH 		1448ULL
//#define E_INK_HEIGHT 		1072ULL

// ED060XH7
//#define E_INK_WIDTH 		1024ULL
//#define E_INK_HEIGHT 		758ULL

#define BLACK 				1
#define WHITE 				0
#define INKPLATE_1BIT 		0
#define INKPLATE_3BIT 		1
#define TPS_SDA             PB9
#define TPS_SCL             PB8
#define FMC_ADDRESS         (0x68000000)

#define PWR_GOOD_OK         0b11111010

// Fast direct port manipulation defines for epaper panel
//#define DATA    		0x000000FF   //PD0 - PD7

/* Clock pin is not used, we do not bit-bang data anymore. Now we are using FMC
#define CL        		GPIO_PIN_11
#define CL_SET    		{GPIOD -> BSRR = CL;}
#define CL_CLEAR  		{GPIOD -> BSRR = CL << 16;}
*/

#define LE        		GPIO_PIN_15
#define LE_SET    		{GPIOB -> BSRR = LE;}
#define LE_CLEAR  		{GPIOB -> BSRR = LE << 16;}

#define CKV       		GPIO_PIN_0
#define CKV_SET   		{GPIOA -> BSRR = CKV;}
#define CKV_CLEAR 		{GPIOA -> BSRR = CKV << 16;}

#define SPH         	GPIO_PIN_2
#define SPH_SET     	{GPIOA -> BSRR = SPH;}
#define SPH_CLEAR   	{GPIOA -> BSRR = SPH << 16;}

#define GMOD            GPIO_PIN_6
#define GMOD_SET    	{GPIOB -> BSRR = GMOD;}
#define GMOD_CLEAR  	{GPIOB -> BSRR = GMOD << 16;}

#define OE          	GPIO_PIN_8
#define OE_SET      	{GPIOA -> BSRR = OE;}
#define OE_CLEAR    	{GPIOA -> BSRR = OE << 16;}

#define SPV         	GPIO_PIN_1
#define SPV_SET     	{GPIOA -> BSRR = SPV;}
#define SPV_CLEAR   	{GPIOA -> BSRR = SPV << 16;}

#define WAKEUP         	GPIO_PIN_12
#define WAKEUP_SET     	{GPIOA -> BSRR = WAKEUP;}
#define WAKEUP_CLEAR   	{GPIOA -> BSRR = WAKEUP << 16;}

#define PWRUP         	GPIO_PIN_11
#define PWRUP_SET     	{GPIOA -> BSRR = PWRUP;}
#define PWRUP_CLEAR   	{GPIOA -> BSRR = PWRUP << 16;}

#define VCOM         	GPIO_PIN_3
#define VCOM_SET     	{GPIOB -> BSRR = VCOM;}
#define VCOM_CLEAR   	{GPIOB -> BSRR = VCOM << 16;}

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

extern SPIClass spi2;
extern SdFat sd;

//void MX_FMC_Init(void);
//uint32_t FMC_Initialized = 0;
//void HAL_FMC_MspInit(void);
extern "C" void HAL_SRAM_MspInit(SRAM_HandleTypeDef* hsram);
//uint32_t FMC_DeInitialized = 0;
//void HAL_FMC_MspDeInit(void);
extern "C" void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef* hsram);
static void SystemClock_Config2(void);
static void delayUS(float _t);

class Inkplate : public Adafruit_GFX {
  public:
    //uint8_t* D_memory_new;
    //uint8_t* _partial;
    //uint8_t* D_memory4Bit;
    //uint8_t * _pBuffer;
    //uint8_t *imageBuffer;
    //uint8_t *partialBuffer;
	__IO uint8_t* imageBuffer = (__IO uint8_t*)0x60000000;
	__IO uint8_t* partialBuffer = (__IO uint8_t*)0x6003A980;
	//uint8_t* imageBuffer;
	//uint8_t* partialBuffer;
    //const uint8_t LUT2[16] = {B01010101, B01010110, B01011001, B01011010, B01100101, B01100110, B01101001, B01101010, B10010101, B10010110, B10011001, B10011010, B10100101, B10100110, B10101001, B10101010};
    //const uint8_t LUTW[16] = {B10101010, B10101011, B10101110, B10101111, B10111010, B10111011, B10111110, B10111111, B11101010, B11101011, B11101110, B11101111, B11111010, B11111011, B11111110, B11111111};
    //const uint8_t LUTB[16] = {B01010101, B01010111, B01011101, B01011111, B01110101, B01110111, B01111101, B01111111, B11010101, B11010111, B11011101, B11011111, B11110101, B11110111, B11111101, B11111111};
    const uint8_t LUT2[16] = {B10101010, B10101001, B10100110, B10100101, B10011010, B10011001, B10010110, B10010101, B01101010, B01101001, B01100110, B01100101, B01011010, B01011001, B01010110, B01010101};
    const uint8_t LUTW[16] = {B11111111, B11111110, B11111011, B11111010, B11101111, B11101110, B11101011, B11101010, B10111111, B10111110, B10111011, B10111010, B10101111, B10101110, B10101011, B10101010};
    const uint8_t LUTB[16] = {B11111111, B11111101, B11110111, B11110101, B11011111, B11011101, B11010111, B11010101, B01111111, B01111101, B01110111, B01110101, B01011111, B01011101, B01010111, B01010101};

    const uint8_t pixelMaskLUT[8] = {B10000000, B01000000, B00100000, B00010000, B00001000, B00000100, B00000010, B00000001};
    const uint8_t pixelMaskGLUT[2] = {B11110000, B00001111};
    const uint8_t discharge[16] = {B11111111, B11111100, B11110011, B11110000, B11001111, B11001100, B11000011, B11000000, B00111111, B00111100, B00110011, B00110000, B00001111, B00001100, B00000011, B00000000};
    //BLACK->WHITE
    
    // Waveform for ED060KC1
    /*
    const uint8_t waveform3Bit[16][15] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 1, 2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 2, 1, 2},
        {0, 0, 0, 0, 0, 0, 0, 2, 1, 2, 1, 1, 1, 2, 2},
		{0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 1, 1, 1, 2, 2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 1, 2},
        {0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2},
		
		{0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2},
		{0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2},
		{0, 0, 0, 0, 0, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2},
        {0, 0, 0, 0, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2},
        {0, 0, 0, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2},
        {0, 0, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2},
        {0, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2},
        {2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2},
    };
    */
    // Waveform for ED060SC7
    uint8_t waveform3Bit[16][17] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 1, 2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 1, 2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 1, 2},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 1, 2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 1, 2},
        {0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 1, 2},
		
		{0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2},
		{0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2},
		{0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2},
        {0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2},
        {0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2},
        {0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2},
        {0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2},
        {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2},
    };

    uint8_t waveform3Bit2[16][15] = {	
	    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1},

        {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2},
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
    void display(uint8_t _leaveOn = false);
    void partialUpdate(uint8_t _leaveOn = 0, uint16_t startRowPos = 0, uint16_t endRowPos = 600);
	void partialUpdate4Bit(uint8_t _leaveOn = 0);
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
	void hscan_start(uint8_t _d1 = 0, uint8_t _d2 = 0);
	void vscan_end();
    void rowSkip(uint16_t _n);
    void cleanFast(uint8_t c, uint8_t rep);
    void pinsZstate();
    void pinsAsOutputs();
    void stm32FmcInit();

  private:
    int8_t _temperature;
    uint8_t _panelOn = 0;
    uint8_t _rotation = 0;
    uint8_t _displayMode = 0; //By default, 1 bit mode is used
	int sdCardOk = 0;
	uint8_t _blockPartial = 1;
	uint8_t _beginDone = 0;
	
	void display1b(uint8_t _leaveOn);
    void display3b(uint8_t _leaveOn);
	uint32_t read32(uint8_t* c);
	uint16_t read16(uint8_t* c);
	void readBmpHeader(SdFile *_f, struct bitmapHeader *_h);
	int drawMonochromeBitmap(SdFile *f, struct bitmapHeader bmpHeader, int x, int y);
	int drawGrayscaleBitmap24(SdFile *f, struct bitmapHeader bmpHeader, int x, int y);
};

#endif
