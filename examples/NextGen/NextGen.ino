#include <Inkplate6NextGen.h>
#include "image4.h"

Inkplate display(INKPLATE_3BIT);
HardwareSerial mySerial(USART1);

extern SRAM_HandleTypeDef hsram1;
extern SRAM_HandleTypeDef hsram2;

void setup() {
  mpuInit();
  mySerial.begin(115200);
  mySerial.println("Test");
  //  pinMode(PB2, OUTPUT);
  //  digitalWrite(PB2, HIGH);

  // For some reason Arduino overrides some of Address FMC pins, so they have to be set as outputs with pinMode...
  //pinMode(PE3, OUTPUT);
  //pinMode(PF0, OUTPUT);
  //pinMode(PF1, OUTPUT);
  //pinMode(PF2, OUTPUT);
  //pinMode(PF3, OUTPUT);
  //pinMode(PF4, OUTPUT);
  //pinMode(PF5, OUTPUT);
  //pinMode(PF12, OUTPUT);
  //pinMode(PF13, OUTPUT);
  //pinMode(PF14, OUTPUT);
  //pinMode(PF15, OUTPUT);

  //  pinMode(PG0, OUTPUT);
  //  pinMode(PG1, OUTPUT);
  //  pinMode(PE7, OUTPUT);
  //  pinMode(PE8, OUTPUT);
  //  pinMode(PE9, OUTPUT);
  //  pinMode(PE10, OUTPUT);
  //  pinMode(PE11, OUTPUT);
  //  pinMode(PE12, OUTPUT);
  //  pinMode(PE13, OUTPUT);
  //  pinMode(PE14, OUTPUT);
  //  pinMode(PE15, OUTPUT);
  //  pinMode(PD8, OUTPUT);
  //  pinMode(PD9, OUTPUT);
  //  pinMode(PD10, OUTPUT);
  //  pinMode(PD11, OUTPUT);
  //  pinMode(PD12, OUTPUT);
  //  pinMode(PD13, OUTPUT);
  //  pinMode(PD14, OUTPUT);
  //  pinMode(PD15, OUTPUT);
  //  pinMode(PG2, OUTPUT);
  //  pinMode(PG3, OUTPUT);
  //  pinMode(PG4, OUTPUT);
  //  pinMode(PG5, OUTPUT);
  //  pinMode(PG6, OUTPUT);
  //  pinMode(PC7, OUTPUT);
  //  pinMode(PD0, OUTPUT);
  //  pinMode(PD1, OUTPUT);
  //  pinMode(PD4, OUTPUT);
  //  pinMode(PD5, OUTPUT);
  //  pinMode(PE0, OUTPUT);
  //  pinMode(PE1, OUTPUT);

  display.begin();
  display.clearDisplay();
  display.drawRect(0, 0, 800 , 600, 0);
  display.drawRect(0, 0, 7, 7, 0);
  display.drawRect(795, 0, 5, 5, 0);
  display.drawRect(0, 593, 7, 7, 0);
  display.drawRect(795, 595, 5, 5, 0);
  display.setCursor(100, 100);
  display.setTextColor(0);
  display.print("Hello!");

  for (int i = 0; i < display.width(); i += 3)
  {
    display.drawPixel(i, 550, 0);
  }

  display.display();

  delay(2500);

  for (int i = 0; i < 15; i++)
  {
    display.fillRect(i * 40 + 20, 20, 40, 400, i);
  }

  display.display();

  delay(2500);
  //
  display.drawBitmap3Bit(0, 0, img4, img4_w, img4_h);
  display.display();


  //  delay(1500);
  //
  //  Serial.println("Did I crash?");
  //  display.flipWaveform();
  //  Serial.println("Maybe??");
  //  display.display(false);
  //  Serial.println("Nope...");

  //  unsigned long t1, t2 = 0;
  //  uint8_t dummy;
  //
  //  t1 = micros();
  //  for (uint32_t i = 0; i < 1024 * 1024; i++)
  //  {
  //    *(__IO uint8_t*)(0x60000000 + i) = dummy;
  //  }
  //  t2 = micros();
  //  mySerial.print("Write time: ");
  //  mySerial.print(1.0 / ((float)(t2 - t1) * 1E-6), 4);
  //  mySerial.println("MB/s");
  //
  //  t1 = micros();
  //  for (uint32_t i = 0; i < 1024 * 1024; i++)
  //  {
  //    dummy = *(__IO uint8_t*)(0x60000000 + i);
  //  }
  //  t2 = micros();
  //  mySerial.print("Read time: ");
  //  mySerial.print(1.0 / ((float)(t2 - t1) * 1E-6), 4);
  //  mySerial.println("MB/s");
    // Disable USB voltage detection on USB OTG (This causes high current consumption in sleep!)
  HAL_PWREx_DisableUSBVoltageDetector();

  // Use different voltage scale for internal voltage regulator (lower current consumption in sleep mode)
  HAL_PWREx_ControlStopModeVoltageScaling(PWR_REGULATOR_SVOS_SCALE5);

  HAL_SRAM_DeInit(&hsram1);
  HAL_SRAM_DeInit(&hsram2);
//  
  HAL_PWREx_EnterSTANDBYMode(PWR_D3_DOMAIN);
  HAL_PWREx_EnterSTANDBYMode(PWR_D2_DOMAIN);
  HAL_PWREx_EnterSTANDBYMode(PWR_D1_DOMAIN);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void mpuInit()
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  // Disable only cache on LCD interface, NOT SRAM!
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x68000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FMC;
  PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}
