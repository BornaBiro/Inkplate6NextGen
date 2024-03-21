#include "InkplateMotion.h"

#include "image1.h"
#include "image2.h"

Inkplate display;

void setup()
{
    Serial.begin(115200);
    Serial.println("STM32 code started...");

    display.begin(INKPLATE_GL16);

    // pinMode(PG9, OUTPUT);
    // digitalWrite(PG9, LOW);

    drawGrad(&display, 0, 300, display.width(), 200);
    display.display();
    delay(2500);

    display.clearDisplay();
    display.drawRect(0, 0, display.width(), display.height(), 0);
    display.drawRect(0, 0, 5, 5, 0);
    display.drawRect(display.width() - 5, 0, 5, 5, 0);
    display.drawRect(0, display.height() - 5, 5, 5, 0);
    display.drawRect(display.width() - 5, display.height() - 5, 5, 5, 0);
    display.display();
    delay(2500);

    display.clearDisplay();
    display.drawBitmap4Bit(0, 0, img1, img1_w, img1_h);
    display.display();
    delay(2500);

    display.clearDisplay();
    display.drawBitmap4Bit(0, 0, img2, img2_w, img2_h);
    display.display();
    delay(2500);
}

void loop()
{

}

void drawGrad(Inkplate *_display, int _x, int _y, int _w, int _h)
{
    int _xStep = _w / 16;

    for (int i = 0; i < 16; i++)
    {
        _display->fillRect(_x + (_xStep * i), _y, _xStep, _h, i);
    }
}