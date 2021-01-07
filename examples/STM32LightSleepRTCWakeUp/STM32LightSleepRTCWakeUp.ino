/*
   This example shows how to use STM32H743 (Inkplate NextGen) internal RTC to wake STM32 up from sleep.

   In order to use this library and example, rtc.c must be removed from C:\Users\[USER]\AppData\Local\Arduino15\packages\STM32\hardware\stm32\1.9.0\libraries\SrcWrapper\src\stm32
   Otherwise this example will not compile!
*/

#include "STM32H7RTC.h"     // Add STM32H7 RTC library made by e-radionica.com
STM32RTC rtc;               // RTC Constructor

// Setting the clock: 14:45:00;
uint8_t hours = 14;
uint8_t minutes = 45;
uint8_t seconds = 0;
uint32_t subSeconds = 0;

// Setting the date: 4/1/2021, Monday
uint8_t day = 4;
uint8_t month = 1;
uint8_t year = 21;

// Recommended way of def. day of week
uint8_t weekday = RTC_WEEKDAY_MONDAY;

// Not recommended way of def. day of week, but it works
//uint8_t weekday = 1;

// Set alarm to active 14:45:10 same day
uint8_t alarmDay = 4;
uint8_t alarmHour = 14;
uint8_t alarmMinute = 45;
uint8_t alarmSeconds = 10;

// Set alarm to be once every day
//uint32_t alarmMask = RTC_ALARMMASK_DATEWEEKDAY;

// Set alarm to be once every hour
//uint32_t alarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS;

// Set alarm to be once every minute
uint32_t alarmMask =  RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;

void setup()
{
  // Setup Serial Communication @ 115200 bauds
  Serial.begin(115200);
  Serial.println("Alarm is set, STM32 is going in sleep mode...");
  Serial.flush();
  // Init RTC library and set it to 24 hour format and reset RTC
  rtc.begin(RTC_HOURFORMAT_24, true);

  // To use 12 hour format use this Init
  //rtc.begin(RTC_HOURFORMAT_12, true);

  // Set time & date on RTC
  rtc.setTime(hours, minutes, seconds, subSeconds);
  rtc.setDate(day, month, year, weekday);

  // Enable alarm on RTC Alarm A (Alarm B is still not fully supported yet!)
  rtc.enableAlarm(alarmDay, alarmHour, alarmMinute, alarmSeconds, RTC_ALARM_A, alarmMask);

  // Enable interrupt on alarm event (also waking up from sleep)
  // We don't want to call any function on wakeup, so we send NULL as argument.
  rtc.enableAlarmInterrupt(NULL);

  // Disable USB voltage detection on USB OTG (This causes high current consumption in sleep!)
  HAL_PWREx_DisableUSBVoltageDetector();

  // Use different voltage scale for internal voltage regulator (lower current consumption in sleep mode)
  HAL_PWREx_ControlStopModeVoltageScaling(PWR_REGULATOR_SVOS_SCALE5);

  // Finally, enter in sleep mode (light sleep, higher current consumption than deep sleep, but all data is retained)
  lightSleep();

  // Print wake up message
  Serial.println("STM32 woken up!");
}

void loop()
{
  // Nothing...
}

void lightSleep()
{
  HAL_PWREx_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI, PWR_D3_DOMAIN);
  HAL_PWREx_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI, PWR_D2_DOMAIN);
  HAL_PWREx_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI, PWR_D1_DOMAIN);
  SystemClock_ConfigFromStop();
}
