//*************************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 2.2
  RGB2812
  http://www.keyestudio.com
*/
#include <Adafruit_NeoPixel.h>

//Create a class called rgb_2812 to control rgb, there are four LEDs, and pins are connected to D10
Adafruit_NeoPixel rgb_2812 = Adafruit_NeoPixel(4, 10, NEO_GRB + NEO_KHZ800);

void setup() {
  rgb_2812.begin();   //start rgb2818
  rgb_2812.setBrightness(150);  //Initialize the brightness to (0~255)
  rgb_2812.clear();   //Initialize all the NeoPixels to “close”state
}
void loop() {
  uint8_t r = random(0, 255);
  uint8_t g = random(0, 255);
  uint8_t b = random(0, 255);
  for (uint8_t i = 0; i < 4; i++)
  {
    rgb_2812.setPixelColor(i, r, g, b);//The color of the i +1 LED is random(r,g,b)
    rgb_2812.show();    //Refresh display
    delay(100); //Wait for a while
  }
}
//*************************************************************************
