#include "Inkplate6NextGen.h"

Inkplate display(INKPLATE_1BIT);
HardwareSerial mySerial(USART1);
#define SRAM_BANK_ADDR                 ((uint32_t)0x60000000)
#define WRITE_READ_ADDR     ((uint32_t)0x0800)
DMA_HandleTypeDef hdma_memtomem_dma1_stream0;

void setup()
{
  mpuInit();
  mySerial.begin(115200);
  mySerial.println("Test");
  pinMode(PB2, OUTPUT);
  digitalWrite(PB2, HIGH);
  display.begin();
  display.cleanFast(1, 40);
  display.cleanFast(0, 40);
  display.cleanFast(1, 40);
  display.cleanFast(0, 40);
  display.cleanFast(2, 7);
  display.setTextSize(5);
  display.setTextColor(BLACK, WHITE);

  extern SRAM_HandleTypeDef hsram1;
  extern SRAM_HandleTypeDef hsram2;
  mySerial.println(hsram1.State, DEC);
  mySerial.println(hsram2.State, DEC);

  //
  //  uint8_t i = 0;
  //  while (true)
  //  {
  //    uint8_t myVar;
  //    //uint32_t *addr = (uint32_t*)(0xC0000000);
  //    //HAL_SRAM_Write_8b(&hsram2, addr, &i, 1);
  //    //addr = (uint32_t*)(0xC0000000);
  //    //HAL_SRAM_Read_8b(&hsram2, addr, &myVar, 1);
  //
  //    *(__IO uint8_t*)(0xC0000000) = i;
  //    myVar = *(__IO uint8_t*)(0xC0000000);
  //
  //    mySerial.println(myVar, DEC);
  //    display.setCursor(50, 50);
  //    display.print(myVar);
  //    display.partialUpdate(true);
  //    i++;
  //  }
  unsigned long time1, time2;
  uint8_t var, dummyVar;

  uint8_t array1[100], array2[100];
  for (int i = 0; i < 100; i++)
  {
    array1[i] = i;
  }

  //HAL_SRAM_Write_8b(&hsram2, (uint32_t*)0x60000000, array1, 100);
  //HAL_SRAM_Read_8b(&hsram2, (uint32_t*)0x60000000, array2, 100);

  for (int i = 0; i < 100; i++)
  {
    *(__IO uint8_t*)(0x60000000 + i) = array1[i];
  }

  for (int i = 0; i < 100; i++)
  {
    array2[i] = *(__IO uint8_t*)(0x60000000 + i);
  }

  for (int i = 0; i < 100; i++)
  {
    mySerial.print(array1[i], DEC);
    mySerial.print(' ');
    mySerial.println(array2[i], DEC);
  }
  //    for (int i = 0; i < 128000; i++)
  //    {
  //      var = random(255);
  //      *(__IO uint8_t*)(0x60000000 + i) = var;
  //      dummyVar = *(__IO uint8_t*)(0x60000000 + i);
  //      if (var != dummyVar)
  //      {
  //        Serial.println("RAM failed");
  //        while (true);
  //      }
  //    }

  mySerial.println("RAM Write Speed: ");
  time1 = micros();

  for (int i = 0; i < 128000; i++)
  {
    *(__IO uint8_t*)(0x60000000 + i) = dummyVar;
  }
  time2 = micros();
  mySerial.print((1.0 / (double(time2 - time1) * 1E-6) * 128000) / 1024 / 1024, 2);
  mySerial.println("MB/s");

  mySerial.println("RAM Read Speed: ");
  time1 = micros();

  for (int i = 0; i < 128000; i++)
  {
    dummyVar = *(__IO uint8_t*)(0x60000000 + i);
  }
  time2 = micros();
  mySerial.print((1.0 / (double(time2 - time1) * 1E-6) * 128000) / 1024 / 1024, 2);
  mySerial.println("MB/s");
}

void loop()
{
  //  mySerial.println("LED is HIGH");
  //  delay(100);
  //  mySerial.println("LED is LOW");
  //  delay(100);
}


void mpuInit()
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

//  /* Configure the MPU as Strongly ordered for not defined regions */
//  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
//  MPU_InitStruct.BaseAddress = 0x00;
//  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
//  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
//  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
//  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
//  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
//  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
//  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
//  MPU_InitStruct.SubRegionDisable = 0x87;
//  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
//
//  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  // Cache only LCD interface, NOT SRAM!
  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x68000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
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
