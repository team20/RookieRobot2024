#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip(19, 6);

int pos = 0;

void setup() {
  // put your setup code here, to run once:
  strip.begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  strip.setBrightness(255);

  strip.setPixelColor(pos, strip.gamma32(strip.ColorHSV(pos++*(65536/strip.numPixels()), 255, 200)));
  strip.show();

  delay(15);

  strip.setBrightness(0);

  if (pos > 18) {
    pos = 0;
  }

  /*strip.setPixelColor(pos++, 255, 0, 255);
  strip.show();
  delay(20);
  strip.setBrightness(0);
  strip.show();

  if (pos > 18) {
    pos = 0;
  }*/
}
