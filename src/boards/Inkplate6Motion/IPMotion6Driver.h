#ifndef __IPMOTION6DRIVER_H__
#define __IPMOTION6DRIVER_H__

// Header guard for the Arduino include
#ifdef BOARD_INKPLATE6_MOTION

// Include main header file for the Arduino.
#include "Arduino.h"

// Include GPIO Pins definitions.
#include "pins.h"

// Include driver for the EPD PMIC.
#include "../../system/epdPmic.h"

// Include waveforms for EPD
#include "waveforms.h"

// Include library for the STM32 FMC
#include "../../stm32System/stm32FMC.h"

// Include Master DMA library for STM32.
#include "../../stm32System/stm32MDMA.h"

// Include library defines
#include "../../system/defines.h"

// Include library for PCAL6416A GPIO expander.
#include "../../system/PCAL_IO.h"

#include "arm_math.h"

// Defines for EPD GPIOs
#define EPD_DRIVER_PINS_OUTPUT  0
#define EPD_DRIVER_PINS_H_ZI    1

// FMC address for sending data to the EPD.
#define EPD_FMC_ADDR    0x68000000

// Fast LUT table for conversion from 2 * 4 bit grayscale pixel to EPD Wavefrom.
static uint8_t _fastGLUT[65536];

// --- Functions declared static inline here for less calling overhead. ---
// TODO: Try to store this function into the the internal RAM for faster execution.

// Start writing the frame on the epaper display.
static inline void vScanStart()
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
}

// Compiler be nice, please do not optimise this function.
__attribute__((optimize("O0"))) static inline void cycleDelay(uint32_t _cycles)
{
    while(_cycles--);
}

// Start writing the first line into epaper display.
__attribute__((always_inline)) static inline void hScanStart(uint8_t _d1, uint8_t _d2)
{
    SPH_CLEAR;
    CKV_SET;
    *(__IO uint8_t *)(EPD_FMC_ADDR) = _d1;
    cycleDelay(10ULL);
    SPH_SET;
    *(__IO uint8_t *)(EPD_FMC_ADDR) = _d2;
}

// End writing the line into epaper display.
__attribute__((always_inline)) static inline void vScanEnd()
{
    CKV_CLEAR;
    cycleDelay(10ULL);
    LE_SET;
    *(__IO uint8_t *)(EPD_FMC_ADDR) = 0;
    LE_CLEAR;
    cycleDelay(10ULL);
}
// --- End of static inline declared functions. ---

class EPDDriver
{
    public:
        EPDDriver();
        int initDriver();
        void cleanFast(uint8_t c, uint8_t rep);
        void clearDisplay();
        void partialUpdate(uint8_t _leaveOn);
        void partialUpdate4Bit(uint8_t _leaveOn);
        void display(uint8_t _leaveOn = 0);
        void display1b(uint8_t _leaveOn);
        void display3b(uint8_t _leaveOn);
        int epdPSU(uint8_t _state);
        double readBattery();
        void selectDisplayMode(uint8_t _mode);
        uint8_t getDisplayMode();

        // Object for ePaper power managment IC.
        EpdPmic pmic;

        // Object for GPIO expander.
        IOExpander internalIO;
    
    protected:
        // Function initializes all GPIO pins used on Inkplate for driving EPD.
        void gpioInit();

        // External SRAM frame buffers astart addresses. Statically allocated due speed.
        // Frame buffer for current image on the screen. 1MB in size (1048576 bytes).
        __IO uint8_t *_currentScreenFB = (__IO uint8_t *)0xD0000000;

        // Frame buffer for the image that will be written to the screen on update. 1MB in size (1048576 bytes).
        __IO uint8_t *_pendingScreenFB = (__IO uint8_t *)0xD0100000;

        // "Scratchpad memory" used for calculations (partial update for example). 6MB in size (6291456 bytes).
        __IO uint8_t *_scratchpadMemory = (__IO uint8_t *)0xD0200000;

    private:
        // LUT for wavefrom calculation.
        uint8_t *GLUT1 = NULL;
        uint8_t *GLUT2 = NULL;

        // Default EPD PSU state is off.
        uint8_t _epdPSUState = 0;

        // Sets EPD control GPIO pins to the output or High-Z state.
        void epdGpioState(uint8_t _state);

        // Fuctions calculates the LUT table for the current wavefrom
        void calculateGLUT(uint8_t *_waveform, uint8_t **_lut1, uint8_t **_lut2, int _phases);

        // Function calculates 4 pixels at once from 4 bit per pixel buffer on the fly (before start writing new frame).
        // With this we can crate LUT that will take all 4 pixels from the frame buffer and convert them into waveform.
        void calculateGLUTOnTheFly(uint8_t *_lut, uint8_t *_waveform);

        // Function calculates the difference between tfo framebuffers (usually between current image on the screen and pending in the MCU memory).
        // Also returns the number of pixel that will be changed.
        uint32_t differenceMask(uint8_t *_currentScreenFB, uint8_t *_pendingScreenFB, uint8_t *_helperArray, uint8_t *_differenceMask);

        // Current display mode. By defaul, set it to 1 bit B&W mode.
        uint8_t _displayMode = INKPLATE_1BW;
        
        // Block partial update at startup, use full update.
        uint8_t _blockPartial = 1;

        // Number of total waveform phases in the current EPD waveform. 
        int _wfPhases = 0;
};

#endif

#endif