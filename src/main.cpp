#include <Arduino.h>
#include <TMC2209.h>
#include <ESP_FlexyStepper.h>

const int microsteps = 8;
const int totalsteps = 200*microsteps; // 200 revolutions and we set 8 microsteps later.

ESP_FlexyStepper stepper;

// The "address" is 2-bit and set with the MS1 and MS2 pins set to 3.3v.
// SERIAL_ADDRESS_0 = MC1=NC, MC2=NC
// SERIAL_ADDRESS_1 = MC1=3.3v, MC2=NC
// SERIAL_ADDRESS_2 = MC1=NC, MC2=3.3v
// SERIAL_ADDRESS_3 = MC1=3.3v, MC2=3.3v

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

  delay(100); // Delay important. Doesn't init multiple motors via serial otherwise.

  stepper_driver_1.setup(Serial2, SERIAL_BAUD_RATE, SERIAL_ADDRESS_1);
  stepper_driver_1.setReplyDelay(REPLY_DELAY);

  delay(100); // Delay important. Doesn't init multiple motors via serial otherwise.

  stepper_driver_2.setup(Serial2, SERIAL_BAUD_RATE, SERIAL_ADDRESS_2);
  stepper_driver_2.setReplyDelay(REPLY_DELAY);

  delay(100); // Delay important. Doesn't init multiple motors via serial otherwise.

  stepper_driver_3.setup(Serial2, SERIAL_BAUD_RATE, SERIAL_ADDRESS_3);
  stepper_driver_3.setReplyDelay(REPLY_DELAY);

  delay(100); // Just in case.

  Serial.begin(115200);

  delay(100); // Just in case.

  // Check the TMC2209 serial control. 
  // The driver boards need both 3.3v and motor power to return OK.
  // The "address" is 2-bit and set with the MS1 and MS2 pins set to 3.3v.

  if (stepper_driver_0.isSetupAndCommunicating()) {
    Serial.println("Motor 0 = OK");
  } else {
    Serial.println("Motor 0 = BAD");
  }

  if (stepper_driver_1.isSetupAndCommunicating()) {
    Serial.println("Motor 1 = OK");
  } else {
    Serial.println("Motor 1 = BAD");
  }

  if (stepper_driver_2.isSetupAndCommunicating()) {
    Serial.println("Motor 2 = OK");
  } else {
    Serial.println("Motor 2 = BAD");
  }

  if (stepper_driver_3.isSetupAndCommunicating()) {
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
  stepper_driver_0.setMicrostepsPerStep(microsteps);
  stepper_driver_0.enable();

  stepper_driver_1.setHardwareEnablePin(HW_DISABLE_PIN);                                                  
  stepper_driver_1.setRunCurrent(5); // percent %
  stepper_driver_1.setHoldCurrent(5); // percent %
  stepper_driver_1.disableCoolStep();    
  stepper_driver_1.disableStealthChop();
  stepper_driver_1.disableAutomaticGradientAdaptation();
  stepper_driver_1.disableAutomaticCurrentScaling(); 
  stepper_driver_1.setMicrostepsPerStep(microsteps);
  stepper_driver_1.enable();

  stepper_driver_2.setHardwareEnablePin(HW_DISABLE_PIN);                                                  
  stepper_driver_2.setRunCurrent(5); // percent %
  stepper_driver_2.setHoldCurrent(5); // percent %
  stepper_driver_2.disableCoolStep();    
  stepper_driver_2.disableStealthChop();
  stepper_driver_2.disableAutomaticGradientAdaptation();
  stepper_driver_2.disableAutomaticCurrentScaling(); 
  stepper_driver_2.setMicrostepsPerStep(microsteps);
  stepper_driver_2.enable();

  stepper_driver_3.setHardwareEnablePin(HW_DISABLE_PIN);                                                  
  stepper_driver_3.setRunCurrent(5); // percent %
  stepper_driver_3.setHoldCurrent(5); // percent %
  stepper_driver_3.disableCoolStep();    
  stepper_driver_3.disableStealthChop();
  stepper_driver_3.disableAutomaticGradientAdaptation();
  stepper_driver_3.disableAutomaticCurrentScaling(); 
  stepper_driver_3.setMicrostepsPerStep(microsteps);
  stepper_driver_3.enable();

  stepper.connectToPins(22,21); // step, dir - we're not using dir in this code though.
  stepper.setStepsPerRevolution(totalsteps);
  stepper.setAccelerationInStepsPerSecondPerSecond(6*totalsteps);
  stepper.setDecelerationInStepsPerSecondPerSecond(6*totalsteps);
  stepper.startAsService(0);

  delay(500);

}

int dir = 1;
int run = 1;
int speed = 0;
int rotation = 0;
int steps = 0;

long motor0steps = 0;
long motor1steps = 0;
long motor2steps = 0;
long motor3steps = 0;

bool stopatzero = false;

void loop() {

  if (digitalRead(0) == LOW) {
    Serial.println("Will stop at next all-zero state.");
    stopatzero = true;
  }

  if (stepper.motionComplete()) {

    Serial.printf("m0 = %d, m1 = %d, m2 = %d, m3 = %d\n", motor0steps, motor1steps, motor2steps, motor3steps);
    
    if (stopatzero) {
      if (motor0steps == 0 && motor0steps == motor1steps && motor1steps == motor2steps && motor2steps == motor3steps) {
        Serial.println("Stopping at zero!");
        while (digitalRead(0) == HIGH) {
          // noop
        }
        stopatzero = false;
      }
    } 

    delay(random(500,2000)); // how long to pause once we get to the position.

    speed = totalsteps*random(1,8); // max speed is also influenced by accelleration config!
    run = random(1,5); // 1 to 4 "moves"
    rotation = random(1,3); // 1 or 2  
    rotation = totalsteps/4*rotation; // 90° or 180° moves, make it /8 for 45° increment moves
    steps = rotation*run; // 90° or 180° done 1..4 times.
    stepper.setSpeedInStepsPerSecond(speed);

    int spin = random(0,4); // pick a random spin pattern

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

    }// else if (spin == 4) {

    //   // "spin == 4" doesn't work as expected. Intended as a "go to home" function.
    //   // Could likey by enabling all the motors by driving the hardware pin to ground
    //   // but that comes with some other consequences.

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

    // Keep track of the steps back to 0, rather than the total steps which may just increase forever.
    // This isn't really used anywhere at the moment.
    //
    motor0steps = motor0steps % totalsteps;
    motor1steps = motor1steps % totalsteps;
    motor2steps = motor2steps % totalsteps;
    motor3steps = motor3steps % totalsteps;

    Serial.printf("Speed = %d, Rotation = %d, Run = %d, Steps = %d, Spin Style = %d\n", speed, rotation, run, steps, spin);
    
    // "dir" is used here mostly for FlexyStepper to keep track of steps
    // as the rotation is handled by the driver board, not the direction pin.
    //
    stepper.setTargetPositionRelativeInSteps(steps*dir);

  }

}

