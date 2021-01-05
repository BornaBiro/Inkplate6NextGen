#include <STM32RTC.h>
STM32RTC& rtc = STM32RTC::getInstance();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting...");
  Serial.flush();
  HAL_Init();
  HAL_Delay(8000);
  rtc.begin(); // initialize RTC 24H format
  rtc.setTime(12, 0, 0);
  rtc.setDate(4, 1, 21);

  rtc.attachInterrupt(alarmMatch);
  rtc.setAlarmDay(4);
  rtc.setAlarmTime(12, 0, 15);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  Serial.println("RTC Set, wait for wakeup...");
  Serial.flush();
  HAL_PWREx_DisableUSBVoltageDetector();  //This causes high current consumption in standby mode!
  HAL_PWREx_ControlStopModeVoltageScaling(PWR_REGULATOR_SVOS_SCALE5);
  deepSleep();
  //SystemClock_ConfigFromStop();
}

void loop() {

}

void lightSleep()
{
  HAL_PWREx_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI, PWR_D3_DOMAIN);
  HAL_PWREx_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI, PWR_D2_DOMAIN);
  HAL_PWREx_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI, PWR_D1_DOMAIN);
}

void deepSleep()
{
  HAL_PWREx_EnterSTANDBYMode(PWR_D3_DOMAIN);
  HAL_PWREx_EnterSTANDBYMode(PWR_D2_DOMAIN);
  HAL_PWREx_EnterSTANDBYMode(PWR_D1_DOMAIN);
}

void alarmMatch(void *data)
{
  UNUSED(data);
}
