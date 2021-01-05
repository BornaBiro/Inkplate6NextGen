/*
   This example shows how to use STM32H743 (Inkplate NextGen) internal RTC to get Date & Time data.

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

// Checks for change in seconds in order to update print on screen
uint8_t oldSeconds = seconds;

void setup()
{
  // Setup Serial Communication @ 115200 bauds
  Serial.begin(115200);

  // Init RTC library and set it to 24 hour format
  rtc.begin(RTC_HOURFORMAT_24);

  // To use 12 hour format use this Init
  //rtc.begin(RTC_HOURFORMAT_12);

  // To reset RTC with every reset of MCU, set second argument in begin function to true
  // NOTE: This also works for 12 hour format! :)
  //rtc.begin(RTC_HOURFORMAT_24, true);

  // Check if the time is already set. If is not, set it!
  if (!rtc.isTimeSet())
  {
    rtc.setTime(hours, minutes, seconds, subSeconds);
    rtc.setDate(day, month, year, weekday);
  }
}

void loop()
{
  // Variables for time and date
  uint8_t h, m, s, d, mn, y, wk;
  uint32_t ss;

  // Get time and date data from STM32 internal RTC using pointers
  // First NULL is for PM/AM indicator (not used in 24 hour mode) and second is for Daylight Saving (not used in this example).
  rtc.getTime(&h, &m, &s, &ss, NULL, NULL);
  rtc.getDate(&d, &mn, &y, &wk);

  // If seconds has passed, print out new time and date
  if (s != oldSeconds)
  {
    oldSeconds = s;
    Serial.print("Time:");
    // Print out hours
    Serial.print(h, DEC);
    Serial.print(':');

    // Print out first and second digit of minutes
    Serial.print(m / 10, DEC);
    Serial.print(m % 10, DEC);
    Serial.print(':');

    // Print out first and second digit of seconds
    Serial.print(s / 10, DEC);
    Serial.print(s % 10, DEC);
    Serial.print(';');
    Serial.println(ss, DEC);

    // Print out date (by European date format)
    Serial.print("Date:");
    Serial.print(d / 10, DEC);
    Serial.print(d % 10, DEC);
    Serial.print('.');
    Serial.print(mn / 10, DEC);
    Serial.print(mn % 10, DEC);
    Serial.print('.');
    Serial.print(y + 2000, DEC);
    Serial.println('.');
  }
}
