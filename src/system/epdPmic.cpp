#include "epdPmic.h"

EpdPmic::EpdPmic()
{
    // Empty constructor.
}

bool EpdPmic::begin()
{
    // Try to ping PMIC. Return false if failed.
    Wire.beginTransmission(TPS_PMIC_ADDR);
    int _ret = Wire.endTransmission();

    // If Wire.endTransmission returns anything else than 0 - Success, return false.
    return (_ret != 0?false:true);
}

void EpdPmic::setRails(uint8_t _rails)
{
    // Disable or enable rails on the PMIC.
    
    // Remove upper two bits.
    _rails &= 0b00111111;

    // Send the data to the PMIC.
    writeRegister(TPS65186_ENABLE, &_rails, 1);
}

void EpdPmic::setVCOM(double _vcom)
{
    // Array for VCOM registers.
    uint8_t _vcomRegs[2] = {0, 0};

    // First divide VCOM voltage by 100 and remove the "-" sign.
    _vcom = abs(_vcom) * 100;

    // Get the VCOM2 register
    readRegister(TPS65186_VCOM2, &_vcomRegs[1], 1);

    // Save lower 8 bits into VCOM1
    _vcomRegs[0] = (int)(_vcom);

    // Save upper 9th bit of the VCOM into bitst bit of the VCOM2 register.
    _vcomRegs[1] &= 0b11111110;
    _vcomRegs[1] |= ((int)_vcom >> 8) & 1;

    // Write data to the PMIC.
    writeRegister(TPS65186_VCOM1, _vcomRegs, 2);
}

double EpdPmic::getVCOM()
{
    // Array to store the content of the VCOM registers.
    uint8_t _vcomRegs[2] = {0, 0};

    // Variable to store calculated VCOM voltage in volts.
    double _vcomVolts = 0;

    // Get the register values.
    readRegister(TPS65186_VCOM1, _vcomRegs, 2);

    // Convert integer value into volts.
    _vcomVolts = (_vcomRegs[0] | ((_vcomRegs[1] & 1) << 8)) / 100.0;
    
    // Return the value and add "-" sign.
    return (_vcomVolts * (-1));
}

void EpdPmic::setPowerOnSeq(uint8_t _upSeq, uint8_t _upSeqDelay)
{
    // Array to store register values.
    uint8_t _upSeqRegs[2];

    // Store power up sequence into first byte.
    _upSeqRegs[0] = _upSeq;

    // Store power up delays into second register.
    _upSeqRegs[1] = _upSeqDelay;

    // Send the regs to the TPS PMIC.
    writeRegister(TPS65186_UPSEQ0, _upSeqRegs, 2);
}

void EpdPmic::setPowerOffSeq(uint8_t _dwnSeq, uint8_t _dwnSeqDelay)
{
    // Array to store register values.
    uint8_t _dwnSeqRegs[2];

    // Store power down sequence into first byte.
    _dwnSeqRegs[0] = _dwnSeq;

    // Store power down delays into second register.
    _dwnSeqRegs[1] = _dwnSeqDelay;

    // Send the regs to the TPS PMIC.
    writeRegister(TPS65186_DWNSEQ0, _dwnSeqRegs, 2);
}

void EpdPmic::programVCOM(double _vcom)
{
    
}

int EpdPmic::getTemperature()
{
    // Temp. variable for storing temperature data.
    uint8_t _temp = 0;

    // Get the register value from the TPS PMIC.
    readRegister(TPS65186_TMST_VALUE, &_temp, 1);

    // Return the value.
    return (int8_t)(_temp);
}

uint8_t EpdPmic::getPwrgoodFlag()
{
    // Temp. variable for storing power good flag.
    uint8_t _pwrGood = 0;

    // Get the register value from TPS PMIC.
    readRegister(TPS65186_PG, &_pwrGood, 1);

    // Return the value.
    return _pwrGood;
}

void EpdPmic::readRegister(uint8_t _reg, uint8_t *_data, uint8_t _n)
{
    // Set the register address.
    Wire.beginTransmission(TPS_PMIC_ADDR);
    Wire.write(_reg);
    Wire.endTransmission();

    // Read the register content.
    Wire.requestFrom(TPS_PMIC_ADDR, _n);
    while (_n--)
    {
        _data[_n] = Wire.read();
    }
}

void EpdPmic::writeRegister(uint8_t _reg, uint8_t *_data, uint8_t _n)
{
    // Set the register address.
    Wire.beginTransmission(TPS_PMIC_ADDR);
    Wire.write(_reg);

    // Write the data to the register.
    Wire.write(_data, _n);

    // Finish I2C transmission.
    Wire.endTransmission();
}