# Inkplate6NextGen
Arduino Library for driving bare ED060SC7 epaper panel with STM32H743ZIT6 MCU.

Note: In order to use it with STM32 Arduino Core, use STM32 Arduino Core v2.0.0 and newer and modify stm32h7xx_hal_conf_default.h in
```
C:\Users\[USERNAME]\AppData\Local\Arduino15\packages\STMicroelectronics\hardware\stm32\2.0.0\system\STM32H7xx
```
and change 
```
#define HSE_VALUE    (25000000UL)
```
to
```
#define HSE_VALUE    (16000000UL)
```
This modification sets external clock frequency of 16MHz (Inkplate Next Gen Board uses 16 MHz XTAL oscillator as a main clock source used by PLL to create 480MHz main clock).
Also open stm32yyxx_hal_conf.h in
```
C:\Users\[USERNAME]\AppData\Local\Arduino15\packages\STMicroelectronics\hardware\stm32\2.0.0\cores\arduino\stm32
```
add
```
#if !defined(HAL_SRAM_MODULE_ENABLED)
  #define HAL_SRAM_MODULE_ENABLED
#else
  #undef HAL_SRAM_MODULE_ENABLED
#endif
```
between "Defined by default" and "Not defined by default" HAL modules. This module is used by FMC.
