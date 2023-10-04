#include "Inkplate6NextGen.h"
//#include "image1.h"
//#include "image2.h"
#include "image3.h"
Inkplate display(INKPLATE_3BIT);

__IO uint8_t *ramBuffer = (__IO uint8_t *)0x60000000;
__attribute__((section(".myItcRam"))) volatile uint8_t array[1024];
MDMA_HandleTypeDef hmdma_mdma_channel40_sw_0;
extern SRAM_HandleTypeDef hsram2;

void setup()
{
    Serial.begin(115200);
    Serial.println("Code has started!");
    MX_MDMA_Init();
    display.begin();
    Serial.printf("array Addr: 0X%08X\r\n", (uint32_t)(array));
    //display.drawBitmap3Bit(0, 0, img3, img3_w, img3_h);
    //display.display();

    for (int i = 0; i < 1024; i++)
    {
        ramBuffer[i] = i;
    }
    delay(2000);

    //for (int i = 0; i < 30; i++)
    //{
    //    Serial.print(ramBuffer[i], DEC);
    //    Serial.print(", ");
    //}
    //delay(2000);
    Serial.println("Reading data from RAM using DMA");
    Serial.flush();
    memset((uint8_t*)array, 0, sizeof(array));
    unsigned long time1 = micros();
    hsram2.hmdma = &hmdma_mdma_channel40_sw_0;
    if (HAL_SRAM_Read_DMA(&hsram2, (uint32_t*)ramBuffer, (uint32_t*)array, 100 / 4) != HAL_OK)
    {
        Serial.println("DMA Failed!");
        while(1);
    }
    HAL_MDMA_PollForTransfer(&hmdma_mdma_channel40_sw_0, HAL_MDMA_FULL_TRANSFER, HAL_MAX_DELAY);
    unsigned long time2 = micros();
    Serial.printf("Done! Time: %lu\r\n", time2 - time1);
    for (int i = 0; i < 1024; i++)
    {
        Serial.print(array[i], DEC);
        Serial.print(", ");
    }
}

void loop() {
  // put your main code here, to run repeatedly:
}

static void MX_MDMA_Init(void)
{
  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* Configure MDMA channel MDMA_Channel0 */
  /* Configure MDMA request hmdma_mdma_channel40_sw_0 on MDMA_Channel0 */
  hmdma_mdma_channel40_sw_0.Instance = MDMA_Channel0;
  hmdma_mdma_channel40_sw_0.Init.Request = MDMA_REQUEST_SW;
  hmdma_mdma_channel40_sw_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel40_sw_0.Init.Priority = MDMA_PRIORITY_VERY_HIGH;
  hmdma_mdma_channel40_sw_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel40_sw_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel40_sw_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel40_sw_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD;
  hmdma_mdma_channel40_sw_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel40_sw_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel40_sw_0.Init.BufferTransferLength = 1;
  hmdma_mdma_channel40_sw_0.Init.SourceBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel40_sw_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel40_sw_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel40_sw_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel40_sw_0) != HAL_OK)
  {
    Error_Handler();
  }
}