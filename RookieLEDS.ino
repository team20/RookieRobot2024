  #include <Adafruit_NeoPixel.h>
  #include <Wire.h>
  #ifdef __AVR__
  #include <avr/power.h>  // Required for 16 MHz Adafruit Trinket
  #endif

  #define ITERATION_DELAY_MS 10
  #define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT 0x32
  #define NUM_BYTES_TO_READ 1
  //I2C Master Code for Arduino Nano

  int ledState = 0;

  // Which pin on the Arduino is connected to the NeoPixels?
  // On a Trinket or Gemma we suggest changing this to 1:
  #define LED_PIN 6

  // How many NeoPixels are attached to the Arduino?
  #define LED_COUNT 96

  // Declare our NeoPixel strip object:
  Adafruit_NeoPixel ledStrip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

  // Argument 1 = Number of pixels in NeoPixel strip
  // Argument 2 = Arduino pin number (most are valid)
  // Argument 3 = Pixel type flags, add together as needed, refer to Adafruit_NeoPixel.h
  // for additional flags(in the event the LEDs don't display the color you want, you
  // probably need a different bitstream):
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
  //   NEO_BRG     Pixels are wired for BRG bitstream (whatever LED Strip 2023 uses)
  //   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
  //   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
  //   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)  
  int colorIndex = 0;  //frame variable, chnages from loop
  int pattern = -1;    //pattern led strips are on, read in from master/robot
  long startTime = -1;
  const long endTime = 30000;                   //timer variables for endgame
  int DHpin = 8;                                //idk, I copied this blindly, I don't think this does anything
  byte dat[5];                                  //same as above
  uint32_t teamColor = ledStrip.Color(0,255,0);    //color of team, default to green, can be set by master/robot to alliance color

  uint32_t TheaterLights(int c, int i, uint32_t color1, uint32_t color2) {  //pixel on color 1 or 2 depending on frame
    if (i % 2 == c % 2) { return (color1); }
    return (color2);
  }
  uint32_t BlinkingLights(int c, int i, uint32_t color3, uint32_t color4) {
    if (c % 2 == 0) { return (color3); }
    return (color4);
  }

  // uint32_t HSV2RGB(uint32_t h, double s, double v) {  //converts HSV colors to RGB integer, used for rainbows
  //   //note: idea is copied from google, math is created from thin air, can be trusted, cannot be explained
  //   h %= 360;
  //   double c = v * s;                                                                             //vs proportion
  //   double x = c * (1 - abs((h / 60) % 2 - 1));                                                   //assigning x
  //   double m = v - c;                                                                             //remaining c-v
  //   uint8_t r = (uint8_t)(((abs(180 - h) > 120) * c + (abs(180 - h) > 60) * (x - c) + m) * 255);  //red
  //   uint8_t g = (uint8_t)(((abs(120 - h) < 60) * c + (abs(120 - h) < 120) * (x - c) + m) * 255);  //green
  //   uint8_t b = (uint8_t)(((abs(240 - h) < 60) * c + (abs(240 - h) < 120) * (x - c) + m) * 255);  //blue
  //   return (strip.Color(r, g, b));                                                                //convert to uint32_t
  // }
  uint32_t RainbowColor(int c, int i) {  //solid rainbow, change with c
    return ((c * 18) % 360, 1, 1);
    //return(strip.Color(0,255,0));
  }
  uint32_t UpperRainbowPartyFunTime(int c, int i) {  //Rainbow but blinking
    if (c % 2 == 0) return (RainbowColor(c / 2, i));
    return (ledStrip.Color(0, 0, 0));
  }
  uint32_t MovingRainbow(int c, int i) {  //Moving rainbow
    return (RainbowColor(c + i, i));
  }
  uint32_t Timer(int c, int i, uint32_t color) {  //user timer proportion to light up specific pixel
    if (c % 2 == 0 && 2 > (i + c) % int(LED_COUNT * (1 - (millis() - startTime) / endTime)) && i * endTime > LED_COUNT * (millis() - startTime)) { return (color); }
    return (ledStrip.Color(0, 0, 0));
  }
  // setup() function -- runs once at startup --------------------------------
  void setup() {
    // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
    // Any other board, you can remove this part (but no harm leaving it):
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
  #endif
    // END of Trinket-specific code.
    ledStrip.begin();
    
    ledStrip.setBrightness(50);  // Set BRIGHTNESS to about 1/5 (max = 255)

    Wire.begin(0x18);         //begin I2C
    Serial.begin(9600);
    // Call receiveEvent when data comes in over I2C
    Wire.onReceive(receiveEvent);
  }
  void loop() {
    //  pattern=(int)((millis()-testStart)/testInc)-1;if(pattern>7){pattern=7;} // used for timed demo
    byte x = Wire.read();
    if (Serial.available()) {
      //If Input exists
      // Transmit I2C data request
      Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);
      Wire.write(NUM_BYTES_TO_READ);
      Wire.endTransmission();
      // Recieve the echoed value back
      Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);               // Begin transmitting to navX-Sensor
      Wire.requestFrom(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT, NUM_BYTES_TO_READ);  // Send number of bytes to read     delay(1);
      Wire.endTransmission();                                                    // Stop transmitting
    }
    switch (pattern) {  // sets up lights to patterns
    // note: every function returns a color based on colorIndex, the pixel index, and optional color parameters.
    // the for loops set the pixels to have their corrseponding colors based on the pattern function on the colorIndex frame
      case 8:           // reset code
        startTime = -1;
        colorIndex = 0;
        pattern = -1;
      case 9: // blinking yellow
        for (int i = 0; i < LED_COUNT; i++) {
          ledStrip.setPixelColor(i, BlinkingLights(colorIndex, i, ledStrip.Color(245, 149, 24), ledStrip.Color(0, 0, 0)));
         }
        delay(150);
        break;    
      case 10: // blinking purple
        for (int i = 0; i < LED_COUNT; i++) {
          ledStrip.setPixelColor(i, BlinkingLights(colorIndex, i, ledStrip.Color(230, 0, 255), ledStrip.Color(0, 0, 0)));
         }
        delay(150);
        break;
      case 16:  //moving green and red gradient
        for (int i = 0; i < LED_COUNT; i++) {
          ledStrip.setPixelColor(i, RainbowPartyFunTime(colorIndex, i));
         }
        delay(150);
        break;
      default:  //display team/alliance color
       for (int i = 0; i < LED_COUNT; i++) {
          ledStrip.setPixelColor(i, teamColor);
         }
        delay(150);
        break;
    }
    ledStrip.show();  //show
    colorIndex++;  //next frame
  }
  void receiveEvent(int howMany) {
    byte x = Wire.read();
    pattern = x;
    //first code must be non-zero multiple of 10
    if (pattern == 8 && Wire.available()) {  //for code 90 reset, check if new alliance color being set
      x = Wire.read();                       //read in alliance code
      if (x == 11) {                          //red alliance
        teamColor = ledStrip.Color(255, 0, 0);
      } else if (x == 12) {  //blue alliance
        teamColor = ledStrip.Color(0, 255, 0);
      }
    }
  }