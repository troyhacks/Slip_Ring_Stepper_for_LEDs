#include <Arduino.h>
#include <TMC2209.h>
#include <ESP_FlexyStepper.h>
// #include <FastLED.h>
#define NUM_LEDS 128

int totalsteps = 200*8;

ESP_FlexyStepper stepper;

TMC2209 stepper_driver_0;
const TMC2209::SerialAddress SERIAL_ADDRESS_0 = TMC2209::SERIAL_ADDRESS_0;
TMC2209 stepper_driver_1;
const TMC2209::SerialAddress SERIAL_ADDRESS_1 = TMC2209::SERIAL_ADDRESS_1;
TMC2209 stepper_driver_2;
const TMC2209::SerialAddress SERIAL_ADDRESS_2 = TMC2209::SERIAL_ADDRESS_2;
TMC2209 stepper_driver_3;
const TMC2209::SerialAddress SERIAL_ADDRESS_3 = TMC2209::SERIAL_ADDRESS_3;

const uint8_t REPLY_DELAY = 2;

const long SERIAL_BAUD_RATE = 115200;
const int RX_PIN = 26;
const int TX_PIN = 25;
const int HW_DISABLE_PIN = 19;

void setup() {

  Serial2.begin(SERIAL_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  stepper_driver_0.setup(Serial2, SERIAL_BAUD_RATE, SERIAL_ADDRESS_0);
  stepper_driver_0.setReplyDelay(REPLY_DELAY);

  delay(100);

  stepper_driver_1.setup(Serial2, SERIAL_BAUD_RATE, SERIAL_ADDRESS_1);
  stepper_driver_1.setReplyDelay(REPLY_DELAY);

  delay(100);

  stepper_driver_2.setup(Serial2, SERIAL_BAUD_RATE, SERIAL_ADDRESS_2);
  stepper_driver_2.setReplyDelay(REPLY_DELAY);

  delay(100);

  stepper_driver_3.setup(Serial2, SERIAL_BAUD_RATE, SERIAL_ADDRESS_3);
  stepper_driver_3.setReplyDelay(REPLY_DELAY);

  Serial.begin(115200);

  delay(100);

  if(stepper_driver_0.isSetupAndCommunicating()) {
    Serial.println("Motor 0 = OK");
  } else {
    Serial.println("Motor 0 = BAD");
  }

  if(stepper_driver_1.isSetupAndCommunicating()) {
    Serial.println("Motor 1 = OK");
  } else {
    Serial.println("Motor 1 = BAD");
  }

  if(stepper_driver_2.isSetupAndCommunicating()) {
    Serial.println("Motor 2 = OK");
  } else {
    Serial.println("Motor 2 = BAD");
  }

  if(stepper_driver_3.isSetupAndCommunicating()) {
    Serial.println("Motor 3 = OK");
  } else {
    Serial.println("Motor 3 = BAD");
  }
  
  Serial.println();

  stepper_driver_0.setHardwareEnablePin(HW_DISABLE_PIN);                  
  stepper_driver_0.setRunCurrent(5); // percent %
  stepper_driver_0.setHoldCurrent(5); // percent %
  stepper_driver_0.disableCoolStep();    
  stepper_driver_0.disableStealthChop();
  stepper_driver_0.disableAutomaticGradientAdaptation();
  stepper_driver_0.disableAutomaticCurrentScaling(); 
  stepper_driver_0.setMicrostepsPerStep(8);
  // stepper_driver_0.enableInverseMotorDirection();
  stepper_driver_0.enable();

  stepper_driver_1.setHardwareEnablePin(HW_DISABLE_PIN);                                                  
  stepper_driver_1.setRunCurrent(5); // percent %
  stepper_driver_1.setHoldCurrent(5); // percent %
  stepper_driver_1.disableCoolStep();    
  stepper_driver_1.disableStealthChop();
  stepper_driver_1.disableAutomaticGradientAdaptation();
  stepper_driver_1.disableAutomaticCurrentScaling(); 
  stepper_driver_1.setMicrostepsPerStep(8);
  // stepper_driver_1.enableInverseMotorDirection();
  stepper_driver_1.enable();

  stepper_driver_2.setHardwareEnablePin(HW_DISABLE_PIN);                                                  
  stepper_driver_2.setRunCurrent(5); // percent %
  stepper_driver_2.setHoldCurrent(5); // percent %
  stepper_driver_2.disableCoolStep();    
  stepper_driver_2.disableStealthChop();
  stepper_driver_2.disableAutomaticGradientAdaptation();
  stepper_driver_2.disableAutomaticCurrentScaling(); 
  stepper_driver_2.setMicrostepsPerStep(8);
  // stepper_driver_2.enableInverseMotorDirection();
  stepper_driver_2.enable();

  stepper_driver_3.setHardwareEnablePin(HW_DISABLE_PIN);                                                  
  stepper_driver_3.setRunCurrent(5); // percent %
  stepper_driver_3.setHoldCurrent(5); // percent %
  stepper_driver_3.disableCoolStep();    
  stepper_driver_3.disableStealthChop();
  stepper_driver_3.disableAutomaticGradientAdaptation();
  stepper_driver_3.disableAutomaticCurrentScaling(); 
  stepper_driver_3.setMicrostepsPerStep(8);
  // stepper_driver_3.enableInverseMotorDirection();
  stepper_driver_3.enable();

  stepper.connectToPins(22,21); // step, dir
  stepper.setStepsPerRevolution(totalsteps);
  stepper.setSpeedInStepsPerSecond(200*8*10);
  stepper.setAccelerationInStepsPerSecondPerSecond(6*totalsteps);
  stepper.setDecelerationInStepsPerSecondPerSecond(6*totalsteps);
  stepper.startAsService(0);

  delay(500);

}

int timedelay = 0;
int dir = 1;
int run = 1;
float currentrev = 0;
int speed = 0;
int rotation = 0;
int steps = 0;

long motor0steps = 0;
long motor1steps = 0;
long motor2steps = 0;
long motor3steps = 0;

void loop() {

  if (stepper.motionComplete()) {

    delay(random(500,2000));

    speed = totalsteps*random(1,8);
    run = random(1,5);
    rotation = random(1,3);
    rotation = totalsteps/4*rotation;
    steps = rotation*run;
    stepper.setSpeedInStepsPerSecond(speed);
    int spin = random(0,4);

    if (spin == 0) {
      stepper_driver_0.enableInverseMotorDirection();
      stepper_driver_1.enableInverseMotorDirection();
      stepper_driver_2.enableInverseMotorDirection();
      stepper_driver_3.enableInverseMotorDirection();
      motor0steps -= steps;
      motor1steps -= steps;
      motor2steps -= steps;
      motor3steps -= steps;
    } else if (spin == 1) {
      stepper_driver_0.disableInverseMotorDirection();
      stepper_driver_1.disableInverseMotorDirection();
      stepper_driver_2.disableInverseMotorDirection();
      stepper_driver_3.disableInverseMotorDirection();
      motor0steps += steps;
      motor1steps += steps;
      motor2steps += steps;
      motor3steps += steps;
    } else if (spin == 2) {
      stepper_driver_0.enableInverseMotorDirection();
      stepper_driver_1.disableInverseMotorDirection();
      stepper_driver_2.disableInverseMotorDirection();
      stepper_driver_3.enableInverseMotorDirection();
      motor0steps -= steps;
      motor1steps += steps;
      motor2steps += steps;
      motor3steps -= steps;
    } else if (spin == 3) {
      stepper_driver_0.disableInverseMotorDirection();
      stepper_driver_1.enableInverseMotorDirection();
      stepper_driver_2.enableInverseMotorDirection();
      stepper_driver_3.disableInverseMotorDirection();
      motor0steps += steps;
      motor1steps -= steps;
      motor2steps -= steps;
      motor3steps += steps;
    }
    
    // } else if (spin == 4) {

    //   stepper_driver_0.disableInverseMotorDirection();
    //   stepper_driver_1.disableInverseMotorDirection();
    //   stepper_driver_2.disableInverseMotorDirection();
    //   stepper_driver_3.disableInverseMotorDirection();

    //   stepper_driver_0.disable();
    //   stepper_driver_1.disable();
    //   stepper_driver_2.disable();
    //   stepper_driver_3.disable();

    //   stepper_driver_0.enable();
    //   stepper.setTargetPositionRelativeInSteps(motor0steps*-1);
    //   if (!stepper.motionComplete()) { }
    //   stepper_driver_0.disable();

    //   stepper_driver_1.enable();
    //   stepper.setTargetPositionRelativeInSteps(motor1steps*-1);
    //   if (!stepper.motionComplete()) { }
    //   stepper_driver_1.disable();

    //   stepper_driver_2.enable();
    //   stepper.setTargetPositionRelativeInSteps(motor2steps*-1);
    //   if (!stepper.motionComplete()) { }
    //   stepper_driver_2.disable();

    //   stepper_driver_3.enable();
    //   stepper.setTargetPositionRelativeInSteps(motor3steps*-1);
    //   if (!stepper.motionComplete()) { }
    //   stepper_driver_3.disable();

    //   stepper_driver_0.enable();
    //   stepper_driver_1.enable();
    //   stepper_driver_2.enable();
    //   stepper_driver_3.enable();

    //   motor0steps = 0;
    //   motor1steps = 0;
    //   motor2steps = 0;
    //   motor3steps = 0;

    // }

    motor0steps = motor0steps % totalsteps;
    motor1steps = motor1steps % totalsteps;
    motor2steps = motor2steps % totalsteps;
    motor3steps = motor3steps % totalsteps;

    Serial.printf("Speed = %d, Rotation = %d, Run = %d, Steps = %d Spin Style = %d\n", speed, rotation, run, steps, spin);
    Serial.printf("m0 = %d, m1 = %d, m2 = %d, m3 = %d\n", motor0steps, motor1steps, motor2steps, motor3steps);
    
    stepper.setTargetPositionRelativeInSteps(steps*dir);

  }

}

