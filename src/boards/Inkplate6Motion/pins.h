#ifndef __INKPLATE6MOTIONPINS_H__
#define __INKPLATE6MOTIONPINS_H__

// Header guard for the Arduino include
#ifdef BOARD_INKPLATE6_MOTION

// Define EPD Pins. Using direct port manipulation for higher speed.
// EPD Latch pin <-> PE6
#define LE                  GPIO_PIN_6
#define LE_SET              GPIOE->BSRR = LE;
#define LE_CLEAR            GPIOE->BSRR = (LE << 16);
#define EPD_LE_GPIO         PE6

// EPD Start Pulse Vertical pin <-> PB1
#define CKV                 GPIO_PIN_1
#define CKV_SET             GPIOB->BSRR = CKV;
#define CKV_CLEAR           GPIOB->BSRR = (CKV << 16);
#define EPD_CKV_GPIO        PB1

// EPD Start Pulse Horizontal pin <-> PD6
#define SPH                 GPIO_PIN_6
#define SPH_SET             GPIOD->BSRR = SPH;
#define SPH_CLEAR           GPIOD->BSRR = (SPH << 16);
#define EPD_SPH_GPIO        PD6

// EPD GMODE Pin <-> PG7
#define GMODE               GPIO_PIN_7
#define GMODE_SET           GPIOG->BSRR = GMODE;
#define GMODE_CLEAR         GPIOG->BSRR = (GMODE << 16);
#define EPD_GMODE_GPIO      PG7 

// EPD Output Enable Pin <-> PB7
#define OE                  GPIO_PIN_7
#define OE_SET              GPIOB->BSRR = OE;
#define OE_CLEAR            GPIOB->BSRR = (OE << 16);
#define EPD_OE_GPIO         PB7

// EPD Start Pulse Horizontal Pin <-> PB7
#define SPV                 GPIO_PIN_12
#define SPV_SET             GPIOG->BSRR = SPV;
#define SPV_CLEAR           GPIOG->BSRR = (SPV << 16);
#define EPD_SPV_GPIO        PG12

// Define pins for TPS65185/TPS65186 EPD PMIC.
// EPD PMIC WakeUp pin <-> PG6 (for now, in future it will be connected to the I/O expander!).
#define WAKEUP              GPIO_PIN_6
#define WAKEUP_SET          GPIOG->BSRR = WAKEUP;
#define WAKEUP_CLEAR        GPIOG->BSRR = WAKEUP << 16;
#define TPS_WAKE_GPIO       PG6

// EPD PMIC PowerUp pin <-> PC13 (for now, in future it will be connected to the I/O expander!).
#define PWRUP               GPIO_PIN_13
#define PWRUP_SET           GPIOC->BSRR = PWRUP;
#define PWRUP_CLEAR         GPIOC->BSRR = PWRUP << 16;
#define TPS_PWR_GPIO        PC13

// EPD PMIC VCOM drive pin <-> PA0 (for now, in future it will be connected to the I/O expander!).
#define VCOM                GPIO_PIN_0
#define VCOM_SET            GPIOA->BSRR = VCOM;
#define VCOM_CLEAR          GPIOA->BSRR = VCOM << 16;
#define TPS_VCOMCTRL_GPIO   PA0

// GPIO pins for TPS65185/TPS65186 EPD PMIC.
#define TPS_SDA_GPIO        PB9
#define TPS_SCL_GPIO        PB8

// External Static RAM power Supply (to save power in deep sleep).
#define RAM_EN_GPIO         PB0

// Battery measurement MOSFET enable GPIO pin
#define BATTERY_MEASUREMENT_EN  PE2

// Analog battery measurement GPIO pin
#define ANALOG_BATTERY_MEASUREMENT   PF10

#endif

#endif