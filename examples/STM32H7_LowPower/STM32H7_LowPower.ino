void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting...");
  Serial.flush();
  HAL_Init();
  HAL_Delay(8000);
  //__HAL_RCC_GPIOC_CLK_DISABLE();
  //__HAL_RCC_GPIOH_CLK_DISABLE();
  //__HAL_RCC_GPIOA_CLK_DISABLE();
  //__HAL_RCC_GPIOB_CLK_DISABLE();
  //__HAL_RCC_GPIOE_CLK_DISABLE();
  //__HAL_RCC_GPIOD_CLK_DISABLE();
  //__HAL_RCC_GPIOG_CLK_DISABLE();
  //HAL_PWREx_DisableUSBReg();
  HAL_PWREx_DisableUSBVoltageDetector();  //This causes high current consumption in standby mode!
  //HAL_PWREx_DisableBatteryCharging();
  //HAL_PWREx_DisableMonitoring();
  //HAL_PWR_DisablePVD();
  //HAL_PWREx_DisableAVD();
  //HAL_SuspendTick();
  HAL_PWREx_ControlStopModeVoltageScaling(PWR_REGULATOR_SVOS_SCALE5);
  //HAL_PWREx_DisableBkUpReg();
  //HAL_PWREx_EnableFlashPowerDown();
  //HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  //SCB_CleanDCache();
  //setWakeUpFromStop();
  setWakeUpFromStandby();
  deepSleep();
  SystemClock_ConfigFromStop();
}

void loop() {
  // put your main code here, to run repeatedly:

}

// To wake up STM32H743, pull PC13 High (or on Nucleo-H743 board push blue user button)
void setWakeUpFromStop()
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gpioinitstruct;
  gpioinitstruct.Pin = GPIO_PIN_13;
  gpioinitstruct.Pull = GPIO_PULLDOWN;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  /* Configure Button pin as input with External interrupt */
  gpioinitstruct.Mode   = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init(GPIOC, &gpioinitstruct);
  /* Enable and set Button EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(EXTI15_10_IRQn), 0x0F, 0);
  HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI15_10_IRQn));
  //__HAL_RCC_PWR_CLK_ENABLE();
  //SystemClock_ConfigFromStop();
}

void setWakeUpFromStandby()
{
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4);
  HAL_PWREx_ClearWakeupFlag(PWR_WAKEUP_FLAG4);
  PWREx_WakeupPinTypeDef wkup = {0};
  wkup.WakeUpPin = PWR_WAKEUP_PIN4;             //@ref PWREx_WakeUp_Pins
  wkup.PinPolarity = PWR_PIN_POLARITY_HIGH;     //@ref PWREx_PIN_Polarity
  wkup.PinPull = PWR_PIN_PULL_DOWN;             //@ref PWREx_PIN_Pull
  HAL_PWREx_EnableWakeUpPin(&wkup);
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
