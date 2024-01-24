#include <Arduino.h>
#include <SmoothStepper.h>
#include <FastLED.h>
#define NUM_LEDS 64

int totalsteps = 200;

// dir = 21
// step = 22

SmoothStepper stepper(totalsteps,22,19,23,18);

CRGB rawleds[NUM_LEDS];
CRGBSet leds(rawleds, NUM_LEDS);

CRGBSet row1(leds(0,7));
CRGBSet row2(leds(8,15));
CRGBSet row3(leds(16,23));
CRGBSet row4(leds(24,31));
CRGBSet row5(leds(32,39));
CRGBSet row6(leds(40,47));
CRGBSet row7(leds(48,55));
CRGBSet row8(leds(56,63));

int bright = 255;
int hue = 0;

struct CRGB * rows[] = { row1,row2,row3,row4,row5,row6,row7,row8 };

void setup() {

  Serial.begin(115200);
  
  disableCore0WDT();

  FastLED.addLeds<NEOPIXEL, 21>(leds, NUM_LEDS);

  stepper.accelerationEnable(10, 265, 2000);
  stepper.begin();

}

int dir = 1;

CHSV getRotationHue(int offset=0) {

  int stepholder = stepper.whatStepNumber() % totalsteps;
  hue = map(stepholder,0,199,0,255);
  CHSV hsvval(hue+offset,255,bright);
  return hsvval;

}

void loop() {

  fill_solid(rows[0]+0,1,getRotationHue(0));
  fill_solid(rows[1]+1,1,getRotationHue(0));
  fill_solid(rows[2]+2,1,getRotationHue(0));
  fill_solid(rows[3]+3,1,getRotationHue(0));
  FastLED.show();

  fill_solid(rows[0]+7,1,getRotationHue(64));
  fill_solid(rows[1]+6,1,getRotationHue(64));
  fill_solid(rows[2]+5,1,getRotationHue(64));
  fill_solid(rows[3]+4,1,getRotationHue(64));
  FastLED.show();

  fill_solid(rows[4]+4,1,getRotationHue(128));
  fill_solid(rows[5]+5,1,getRotationHue(128));
  fill_solid(rows[6]+6,1,getRotationHue(128));
  fill_solid(rows[7]+7,1,getRotationHue(128));
  FastLED.show();

  fill_solid(rows[7]+0,1,getRotationHue(192));
  fill_solid(rows[6]+1,1,getRotationHue(192));
  fill_solid(rows[5]+2,1,getRotationHue(192));
  fill_solid(rows[4]+3,1,getRotationHue(192));
  FastLED.show();

  if (stepper.isArrived()) {
    stepper.step(totalsteps*dir*60);
    dir *= -1;
  }

}

