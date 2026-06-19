//*************************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 2.1
  RGB2812
  http://www.keyestudio.com
*/
#include <Adafruit_NeoPixel.h>

//Create a class called rgb_2812 to control rgb, there are four LEDs, and pins are connected to D10
Adafruit_NeoPixel rgb_2812 = Adafruit_NeoPixel(4, 10, NEO_GRB + NEO_KHZ800);

void setup() {
  rgb_2812.begin();   //Start rgb2818
  rgb_2812.setBrightness(100);  //Initialize the brightness to (0~255)
  rgb_2812.clear();   //Initialize all the NeoPixels to “close”state


  rgb_2812.setPixelColor(0, 255, 0, 0);//The first LED is red
  rgb_2812.setPixelColor(1, 0, 255, 0);//The second LED is green
  rgb_2812.setPixelColor(2, 0, 0, 255);//The third LED is blue
  rgb_2812.setPixelColor(3, 255, 255, 255);//The fourth LED is white
  rgb_2812.show();    //Refresh display
}

void loop() {
}
//*************************************************************************
