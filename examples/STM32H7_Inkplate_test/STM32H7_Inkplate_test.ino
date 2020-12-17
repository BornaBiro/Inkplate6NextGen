#include "Inkplate6NextGen.h"
#include "img1.h"
#include "img2.h"
Inkplate display(INKPLATE_3BIT);
void setup()
{
  pinMode(USER_BTN, INPUT);
  //HAL_Init();
  //SystemInit();
  display.begin();
  Serial.begin(115200);
  display.setTextSize(3);
  display.setTextColor(BLACK, WHITE);
  //display.setCursor(100, 0);
  //display.setTextColor(0, 7);
  //display.setTextSize(4);
  //display.print("This is test");
  //grad();
  //display.drawBitmap3Bit(0, 0, gray, gray_w, gray_h);
  //display.display();
  //delay(5000);
  //display.clearDisplay();
  grad();
  //screenCenter();
  //screenCenterBW();
  //display.drawBitmap3Bit(0, 0, pic2, 800, 592);
  display.display();
}

int n = 0;

void loop() {
  if (digitalRead(USER_BTN)) {
    clearIt();
    delay(1000);
    //display.clearDisplay();
    //display.setCursor(0,0);
    //display.print("Var: ");
    //display.print(n, DEC);
    //unsigned long t1, t2;
    //t1 = micros();
    //display.partialUpdate(true);
    //t2 = micros();
    //Serial.println(t2 - t1, DEC);
    //n++;
  }
}

void grad()
{
  for (int i = 0; i < 16; i++)
  {
    display.fillRect(50 * i, 0, 50, 600, i);
    display.drawFastVLine(50 * i, 0, 600, 0);
  }
}

void screenCenter()
{
  display.drawRect(0, 0, 800, 600, 0);
  display.drawRect(0, 0, 10, 10, 0);
  display.drawRect(0, 589, 10, 10, 0);
  display.drawRect(789, 0, 10, 10, 0);
  display.drawRect(789, 589, 10, 10, 0);
}

void screenCenterBW()
{
  display.drawRect(0, 0, 800, 600, BLACK);
  display.drawRect(0, 0, 10, 10, BLACK);
  display.drawRect(0, 589, 10, 10, BLACK);
  display.drawRect(789, 0, 10, 10, BLACK);
  display.drawRect(789, 589, 10, 10, BLACK);
}

void clearIt()
{
  display.cleanFast(1, 28);
  display.cleanFast(2, 7);
  display.cleanFast(0, 28);
  display.cleanFast(2, 7);
  display.cleanFast(1, 28);
  display.cleanFast(2, 7);
  display.cleanFast(0, 28);
  display.cleanFast(2, 3);
  display.cleanFast(3, 2);
  display.einkOff();
}