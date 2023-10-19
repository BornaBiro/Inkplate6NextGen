#include "IPMotion6Driver.h"

// Header guard for the Arduino include
#ifdef BOARD_INKPLATE6_MOTION

EPDDriver::EPDDriver()
{
    // Empty constructor.
}

int EPDDriver::initDriver()
{
    // Configure GPIO pins.
    gpioInit();

    // Enable TPS65186 and keep it on.
    WAKEUP_SET;

    // Init PMIC. Return error if failed.
    if (!pmic.begin()) return 0;

    // Init STM32 FMC (Flexible memory controller) for faster pushing data to panel using Hardware (similar to ESP32 I2S
    // Parallel, but much faster and better).
    stm32FmcInit();

    // Send proper power up sequence for EPD.
    pmic.setPowerOffSeq(0b00000000, 0b00000000);

    // Calculate the GLUT for fast pixel to waveform calculation.
    calculateGLUT(GLUT1, GLUT2, 17);

    // Everything went ok? Return 1 for success.
    return 1;
}

void EPDDriver::cleanFast(uint8_t c, uint8_t rep)
{
    // Enable EPD PSU.
    epdPSU(1);

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

    for (int k = 0; k < rep; k++)
    {
        vScanStart();
        for (int i = 0; i < SCREEN_HEIGHT; i++)
        {
            hScanStart(data, data);
            for (int j = 0; j < (SCREEN_WIDTH / 8) - 1; j++)
            {
                //  Do not use HAL. it's way slower. Use direct writing data to memory
                *(__IO uint8_t *)(EPD_FMC_ADDR) = data;
                *(__IO uint8_t *)(EPD_FMC_ADDR) = data;
            }

            vScanEnd();
        }

        delayMicroseconds(230);
    }
}

void EPDDriver::clearDisplay()
{
    if (getDisplayMode() == INKPLATE_1BW)
    {
        for (int i = 0; i < (SCREEN_HEIGHT * SCREEN_WIDTH / 8); i++)
        {
            partialBuffer[i] = 0;
        }
    }

    if (getDisplayMode() == INKPLATE_GL16)
    {
        for (int i = 0; i < (SCREEN_HEIGHT * SCREEN_WIDTH / 2); i++)
        {
            partialBuffer[i] = 255;
        }
    }
}

void EPDDriver::partialUpdate(uint8_t _leaveOn)
{
    if (getDisplayMode() == INKPLATE_1BW)
        return;

    if (_blockPartial == 1)
    {
        display1b(_leaveOn);
        return;
    }

    // Enable EPD PSU.
    epdPSU(1);
    
    __IO uint8_t *_pos;
    __IO uint8_t *_partialPos;
    uint8_t diffw;
    uint8_t diffb;
    for (int k = 0; k < 7; k++)
    {
        vScanStart();
        _pos = imageBuffer;
        _partialPos = partialBuffer;

        for (int i = 0; i < SCREEN_HEIGHT; i++)
        {
            diffw = *(_pos) & ~*(_partialPos); // Calculate differences in white pixels
            diffb = ~*(_pos) & *(_partialPos); // Calculate differences in black pixels
            _pos++;                            // Move address pointers
            _partialPos++;
            hScanStart(LUTW[diffw >> 4] & (LUTB[diffb >> 4]),
                        LUTW[diffw & 0x0F] &
                            (LUTB[diffb & 0x0F]));          // Start sending first pixel byte to panel //Clock it!
            for (int j = 0; j < (SCREEN_WIDTH / 8) - 1; j++) // Now do all that for whole row
            {
                diffw = *(_pos) & ~*(_partialPos);
                diffb = ~*(_pos) & *(_partialPos);
                _pos++;
                _partialPos++;
                *(__IO uint8_t *)(EPD_FMC_ADDR) = LUTW[diffw >> 4] & (LUTB[diffb >> 4]);
                *(__IO uint8_t *)(EPD_FMC_ADDR) = LUTW[diffw & 0x0F] & (LUTB[diffb & 0x0F]);
            }
            vScanEnd();
        }
    }

    cleanFast(2, 1);
    
    // Disable EPD PSU if needed.
    if (!_leaveOn)
        epdPSU(0);

    // After update, copy differences to screen buffer
    for (int i = 0; i < (SCREEN_HEIGHT * SCREEN_WIDTH / 8); i++)
    {
        *(imageBuffer + i) = *(partialBuffer + i);
    }
}

void EPDDriver::partialUpdate4Bit(uint8_t _leaveOn)
{
    // TO-DO!!! Do not alloce whole user RAM just to find the difference mask. Do it line by line!
    // And also check why image is darker on partial update compared to the full grayscale update!
    // Anyhow, this must be optimised!
    const uint8_t _cleanWaveform[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2,
                                      2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2};
    uint8_t _panelMask[SCREEN_HEIGHT * SCREEN_WIDTH / 4];
    uint8_t *_pPanelMask = _panelMask;
    memset(_panelMask, 0, sizeof(_panelMask));

    // We present you the ugliest code ever...needs A LOT of clean up and optimisation!
    // Create the mask for pixel difference
    __IO uint8_t *_pPartial = partialBuffer;
    __IO uint8_t *_pImage = imageBuffer;
    for (int i = 0; i < SCREEN_HEIGHT; i++)
    {
        for (int j = 0; j < (SCREEN_WIDTH / 4); j++) // Now do all that for whole row
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
    
    // Enable EPD PSU.
    epdPSU(1);

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

        vScanStart();
        for (int i = 0; i < SCREEN_HEIGHT; i++)
        {
            hScanStart(_color & *_pPanelMask++, _color & *_pPanelMask++); // Start sending first pixel byte to panel
            for (int j = 0; j < (SCREEN_WIDTH / 8) - 1; j++)                // Now do all that for whole row
            {
                *(__IO uint8_t *)(EPD_FMC_ADDR) = _color & *_pPanelMask++;
                *(__IO uint8_t *)(EPD_FMC_ADDR) = _color & *_pPanelMask++;
            }
            vScanEnd(); // Write one row to panel
        }
        delayMicroseconds(230); // Wait 230uS before new frame
    }

    for (int k = 0; k < 15; k++)
    {
        __IO uint8_t *dp = partialBuffer;
        _pPanelMask = _panelMask;
        vScanStart();
        for (int i = 0; i < SCREEN_HEIGHT; i++)
        {
            hScanStart((GLUT2[k * 256 + (*(dp++))] | GLUT1[k * 256 + (*(dp++))]),
                        (GLUT2[k * 256 + (*(dp++))] | GLUT1[k * 256 + (*(dp++))]));
            for (int j = 0; j < ((SCREEN_WIDTH / 8)) - 1; j++)
            {

                *(__IO uint8_t *)(EPD_FMC_ADDR) = (GLUT2[k * 256 + (*(dp++))] | GLUT1[k * 256 + (*(dp++))]);
                *(__IO uint8_t *)(EPD_FMC_ADDR) = (GLUT2[k * 256 + (*(dp++))] | GLUT1[k * 256 + (*(dp++))]);
            }
            vScanEnd();
        }
        delayMicroseconds(230);
    }
    cleanFast(2, 2);
    cleanFast(3, 1);

    // After update, copy differences to screen buffer
    for (int i = 0; i < (SCREEN_WIDTH * SCREEN_HEIGHT / 2); i++)
    {
        *(imageBuffer + i) &= *(partialBuffer + i);
        *(imageBuffer + i) |= *(partialBuffer + i);
    }

    // Disable EPD PSU if needed.
    if (!_leaveOn)
        epdPSU(0);
}

void EPDDriver::display(uint8_t _leaveOn)
{
    // Depending on the mode, use on or the other function.
    if (getDisplayMode() == INKPLATE_1BW)
    {
        display1b(_leaveOn);
    }
    else
    {
        display3b(_leaveOn);
    }
}

// Display content from RAM to display (1 bit per pixel,. monochrome picture).
void EPDDriver::display1b(uint8_t _leaveOn)
{
    __IO uint8_t *_pos;
    uint8_t data;

    // Enable EPD PSU.
    epdPSU(1);
    
    cleanFast(0, 5);

    for (int k = 0; k < 10; k++)
    {
        _pos = partialBuffer;
        vScanStart();
        for (int i = 0; i < SCREEN_HEIGHT; i++)
        {
            hScanStart(LUT2[(~(*_pos) >> 4) & 0x0F], LUT2[~(*_pos) & 0x0F]);
            _pos++;
            for (int j = 0; j < ((SCREEN_WIDTH / 8) - 1); j++)
            {
                *(__IO uint8_t *)(EPD_FMC_ADDR) = LUT2[(~(*_pos) >> 4) & 0x0F];
                *(__IO uint8_t *)(EPD_FMC_ADDR) = LUT2[~(*_pos) & 0x0F];
                _pos++;
            }
            vScanEnd();
        }
        delayMicroseconds(230);
    }
    cleanFast(1, 10);
    cleanFast(2, 1);

    // Full update? Copy everything in screen buffer before refresh!
    for (int i = 0; i < (SCREEN_HEIGHT * SCREEN_WIDTH) / 8; i++)
    {
        imageBuffer[i] &= partialBuffer[i];
        imageBuffer[i] |= partialBuffer[i];
    }

    for (int k = 0; k < 30; k++)
    {
        _pos = imageBuffer;
        vScanStart();
        for (int i = 0; i < SCREEN_HEIGHT; i++)
        {
            hScanStart(LUT2[(~(*_pos) >> 4) & 0x0F], LUT2[~(*_pos) & 0x0F]);
            _pos++;
            for (int j = 0; j < ((SCREEN_WIDTH / 8) - 1); j++)
            {
                *(__IO uint8_t *)(EPD_FMC_ADDR) = LUT2[(~(*_pos) >> 4) & 0x0F];
                *(__IO uint8_t *)(EPD_FMC_ADDR) = LUT2[~(*_pos) & 0x0F];
                _pos++;
            }
            vScanEnd();
        }
        delayMicroseconds(230);
    }

    cleanFast(2, 1);
    for (int k = 0; k < 30; k++)
    {
        _pos = imageBuffer;
        vScanStart();
        for (int i = 0; i < SCREEN_HEIGHT; i++)
        {
            hScanStart(LUT2[((*_pos) >> 4) & 0x0F], LUT2[(*_pos) & 0x0F]);
            _pos++;
            for (int j = 0; j < ((SCREEN_WIDTH / 8) - 1); j++)
            {
                *(__IO uint8_t *)(EPD_FMC_ADDR) = LUT2[((*_pos) >> 4) & 0x0F];
                *(__IO uint8_t *)(EPD_FMC_ADDR) = LUT2[(*_pos) & 0x0F];
                _pos++;
            }
            vScanEnd();
        }
        delayMicroseconds(230);
    }

    cleanFast(2, 1);

    // Disable EPD PSU if needed.
    if (!_leaveOn)
        epdPSU(0);

    _blockPartial = 0;
}

void EPDDriver::display3b(uint8_t _leaveOn)
{
    // Full update? Copy everything in screen buffer before refresh!
    for (int i = 0; i < (SCREEN_HEIGHT * SCREEN_WIDTH) / 2; i++)
    {
        imageBuffer[i] = partialBuffer[i];
    }

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
        vScanStart();
        for (int i = 0; i < SCREEN_HEIGHT; i++)
        {
            hScanStart((GLUT2[k * 256 + (*(dp++))] | GLUT1[k * 256 + (*(dp++))]),
                        (GLUT2[k * 256 + (*(dp++))] | GLUT1[k * 256 + (*(dp++))]));
            for (int j = 0; j < ((SCREEN_WIDTH / 8)) - 1; j++)
            {
                *(__IO uint8_t *)(EPD_FMC_ADDR) = GLUT2[k * 256 + (*(dp++))] | GLUT1[k * 256 + (*(dp++))];
                *(__IO uint8_t *)(EPD_FMC_ADDR) = GLUT2[k * 256 + (*(dp++))] | GLUT1[k * 256 + (*(dp++))];
            }
            vScanEnd();
        }
        delayMicroseconds(230);
    }
    cleanFast(2, 2);
    cleanFast(3, 1);

    // Disable EPD PSU if needed.
    if (!_leaveOn)
        epdPSU(0);
}

int EPDDriver::epdPSU(uint8_t _state)
{
    // Check if the atate is already set.
    if (_state == _epdPSUState) return 1;

    // Enable the EPD power supply
    if (_state)
    {
        WAKEUP_SET;
        delay(1);
        PWRUP_SET;
        delay(1);

        // Enable all rails
        pmic.setRails(0b00111111);

        // Configure GPIO pins for defaullt state after power up.
        epdGpioState(EPD_DRIVER_PINS_OUTPUT);
        LE_CLEAR;
        SPH_SET;
        GMODE_SET;
        SPV_SET;
        CKV_CLEAR;
        OE_SET;
        VCOM_SET;

        // Wait until EPD PMIC has all needed voltages at it's outputs.
        // 250 ms should be long enough.
        unsigned long timer = millis();
        do
        {
            delay(1);
        } while ((pmic.getPwrgoodFlag() != TPS_PWR_GOOD_OK) && (millis() - timer) < 250);

        // Not ready even after 250ms? Something is wrong, shut down TPS!
        if (pmic.getPwrgoodFlag() != TPS_PWR_GOOD_OK)
        {
            VCOM_CLEAR;
            PWRUP_CLEAR;
            return 0;
        }

        // Set new PMIC state.
        _epdPSUState = 1;
    }
    else
    {
        // Shut down req.

        // Set all GPIO pins to lov (to avoid epaper display damage).
        OE_CLEAR;
        GMODE_CLEAR;
        CKV_CLEAR;
        SPH_CLEAR;
        SPV_CLEAR;
        LE_CLEAR;
        VCOM_CLEAR;
        PWRUP_CLEAR;

        // 250ms should be long enough to shut down all EPD PMICs voltage rails.
        unsigned long timer = millis();
        do
        {
            delay(1);
        } while ((pmic.getPwrgoodFlag() != 0) && (millis() - timer) < 250);

        // There is still voltages at the EPD PMIC? Something does not seems right...
        if (pmic.getPwrgoodFlag() != 0) return 0;

        epdGpioState(EPD_DRIVER_PINS_H_ZI);

        // Set new PMIC state.
        _epdPSUState = 1;      
    }

    // Everything went ok? Return 1 as success.
    return 1;
}

void EPDDriver::gpioInit()
{
    // For some reason, HAL doesn't initalize GPIOF properly, so it has to be done with Arduino pinMode() function.
    pinMode(PF0, OUTPUT);

    // Enable the external RAM (inverse logic due P-MOS) and enable it by default.
    pinMode(RAM_EN_GPIO, OUTPUT);
    digitalWrite(RAM_EN_GPIO, LOW);

    // Disable battery measurement pin
    pinMode(BATTERY_MEASUREMENT_EN, OUTPUT);
    digitalWrite(BATTERY_MEASUREMENT_EN, LOW);

    // Set battery measurement pin as analog input.
    pinMode(ANALOG_BATTERY_MEASUREMENT, INPUT_ANALOG);

    // Set TPS control pins to outputs.
    pinMode(TPS_PWR_GPIO, OUTPUT);
    pinMode(TPS_WAKE_GPIO, OUTPUT);
    pinMode(TPS_VCOMCTRL_GPIO, OUTPUT);

    // Set the type of the EPD control pins. 
    pinMode(EPD_CKV_GPIO, OUTPUT);
    pinMode(EPD_SPV_GPIO, OUTPUT);
    pinMode(EPD_SPH_GPIO, OUTPUT);
    pinMode(EPD_OE_GPIO, OUTPUT);
    pinMode(EPD_GMODE_GPIO, OUTPUT);
    pinMode(EPD_LE_GPIO, OUTPUT);
}

double EPDDriver::readBattery()
{
    // Enable MOSFET for viltage divider.
    digitalWrite(BATTERY_MEASUREMENT_EN, HIGH);

    // Wait a little bit.
    delay(1);

    // Get an voltage measurement from ADC.
    uint16_t _adcRaw = analogRead(ANALOG_BATTERY_MEASUREMENT);

    // Disable MOSFET for voltage divider (to save power).
    digitalWrite(BATTERY_MEASUREMENT_EN, LOW);

    // Calculate the voltage from ADC measurement. Divide by 82^16) - 1 to get
    // measurement voltage in the form of the percentage of the ADC voltage,
    // multiply it by analog reference voltage and multiply by two (voltage divider).
    double _voltage = (double)(_adcRaw) / 65535.0 * 3.3 * 2;

    // Return the measured voltage.
    return _voltage;
}

void EPDDriver::epdGpioState(uint8_t _state)
{
    if (_state)
    {
        // Set all pins to input (Hi-Z state).
        pinMode(EPD_GMODE_GPIO, INPUT);
        pinMode(EPD_CKV_GPIO, INPUT);
        pinMode(EPD_SPV_GPIO, INPUT);
        pinMode(EPD_SPH_GPIO, INPUT);
        pinMode(EPD_OE_GPIO, INPUT);
        pinMode(EPD_LE_GPIO, INPUT);
    }
    else
    {
        // Set all pins to the output.
        pinMode(EPD_GMODE_GPIO, OUTPUT);
        pinMode(EPD_CKV_GPIO, OUTPUT);
        pinMode(EPD_SPV_GPIO, OUTPUT);
        pinMode(EPD_SPH_GPIO, OUTPUT);
        pinMode(EPD_OE_GPIO, OUTPUT);
        pinMode(EPD_LE_GPIO, OUTPUT);
    }
}

void EPDDriver::calculateGLUT(uint8_t *_lut1, uint8_t *_lut2, int _phases)
{
    // Clear previous memory allocation
    if (_lut1 != NULL) free(_lut1);
    if (_lut2 != NULL) free(_lut2);

    // Allocate new memory allocaiton
    // Make fast Grayscale LUT to convert 4 pixels into proper waveform data for EPD.
    _lut1 = (uint8_t *)malloc(256 * _phases * sizeof(uint8_t));
    _lut1 = (uint8_t *)malloc(256 * _phases * sizeof(uint8_t));

    for (int j = 0; j < _phases; ++j)
    {
        for (uint32_t i = 0; i < 256; ++i)
        {
            _lut1[j * 256 + i] = (waveform3Bit2[i & 0x0f][j] << 2) | (waveform3Bit2[(i >> 4) & 0x0f][j]);
            _lut1[j * 256 + i] = ((waveform3Bit2[i & 0x0f][j] << 2) | (waveform3Bit2[(i >> 4) & 0x0f][j])) << 4;
        }
    }
}

void EPDDriver::selectDisplayMode(uint8_t _mode)
{
    _displayMode = _mode & 1;
    _blockPartial = 1;
}

uint8_t EPDDriver::getDisplayMode()
{
    return _displayMode;
}

#endif