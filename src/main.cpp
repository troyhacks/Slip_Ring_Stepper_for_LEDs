#include <Arduino.h>
#include <TMC2209.h>
#include <ESP_FlexyStepper.h>

const int microsteps = 8;
const int totalsteps = 200*microsteps; // 200 revolutions and we set 8 microsteps later.

ESP_FlexyStepper stepper;

// The "address" is 2-bit and set with the MS1 and MS2 pins set to 3.3v.
// SERIAL_ADDRESS_0 = MC1=NC   MC2=NC
// SERIAL_ADDRESS_1 = MC1=3.3v MC2=NC
// SERIAL_ADDRESS_2 = MC1=NC   MC2=3.3v
// SERIAL_ADDRESS_3 = MC1=3.3v MC2=3.3v

enum{MOTOR_COUNT=4};

TMC2209 stepper_drivers[MOTOR_COUNT];

TMC2209::SerialAddress SERIAL_ADDRESS_0 = TMC2209::SERIAL_ADDRESS_0;
TMC2209::SerialAddress SERIAL_ADDRESS_1 = TMC2209::SERIAL_ADDRESS_1;
TMC2209::SerialAddress SERIAL_ADDRESS_2 = TMC2209::SERIAL_ADDRESS_2;
TMC2209::SerialAddress SERIAL_ADDRESS_3 = TMC2209::SERIAL_ADDRESS_3;

TMC2209::SerialAddress * serial_addesses[MOTOR_COUNT] = {
  &SERIAL_ADDRESS_0,
  &SERIAL_ADDRESS_1,
  &SERIAL_ADDRESS_2,
  &SERIAL_ADDRESS_3
};

const uint8_t REPLY_DELAY = 2;

const long SERIAL_BAUD_RATE = 115200;
const int RX_PIN = 26;
const int TX_PIN = 25;
const int HW_DISABLE_PIN = 19;

void setup() {

  Serial2.begin(SERIAL_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  for (size_t motor_index=0; motor_index<MOTOR_COUNT; motor_index++) {

    stepper_drivers[motor_index].setup(Serial2, SERIAL_BAUD_RATE, *(serial_addesses[motor_index]));
    stepper_drivers[motor_index].setReplyDelay(REPLY_DELAY);
    stepper_drivers[motor_index].setHardwareEnablePin(HW_DISABLE_PIN);                  
    stepper_drivers[motor_index].setRunCurrent(5); // percent %
    stepper_drivers[motor_index].setHoldCurrent(5); // percent %
    stepper_drivers[motor_index].disableCoolStep();    
    stepper_drivers[motor_index].disableStealthChop();
    stepper_drivers[motor_index].disableAutomaticGradientAdaptation();
    stepper_drivers[motor_index].disableAutomaticCurrentScaling(); 
    stepper_drivers[motor_index].setMicrostepsPerStep(microsteps);
    stepper_drivers[motor_index].enable();

    delay(100); // important for first inits.

  }

  Serial.begin(115200);

  delay(100); // Just in case.

  // Check the TMC2209 serial control. 
  // The driver boards need both 3.3v and motor power to return OK.
  // The "address" is 2-bit and set with the MS1 and MS2 pins set to 3.3v.

  for (size_t motor_index=0; motor_index<MOTOR_COUNT; motor_index++) {
      Serial.print("Motor ");
      Serial.print(motor_index);
    if (stepper_drivers[motor_index].isSetupAndCommunicating()) {
      Serial.println(" = OK");
    } else {
      Serial.println(" = BAD");
    }
  }

  Serial.println();

  stepper.connectToPins(22,21); // step, dir - we're not using dir in this code though.
  stepper.setStepsPerRevolution(totalsteps);
  stepper.setAccelerationInStepsPerSecondPerSecond(6*totalsteps);
  stepper.setDecelerationInStepsPerSecondPerSecond(6*totalsteps);
  stepper.startAsService(0);

  delay(1500); // in case the board resets during programming.

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
bool homev1 = true;
int spin = 0;

void loop() {

  if (digitalRead(0) == LOW) {
    Serial.println("Will stop at next all-zero state.");
    stopatzero = true;
    delay(1000);
  }

  if (stepper.motionComplete()) {

    if (stopatzero) {

      if (motor0steps == 0 && motor0steps == motor1steps && motor1steps == motor2steps && motor2steps == motor3steps) {

        Serial.println("Stopping at zero! Press the button again to restart.");

        while (digitalRead(0) == HIGH) {
          // noop
        }

        Serial.println("Resuming stepping.");
        stopatzero = false;

      }

    } 

    delay(random(500,2000)); // how long to pause once we get to the position.

    speed = totalsteps*random(1,8); // max speed is also influenced by accelleration config!
    run = random(1,5); // 1 to 4 "moves"
    rotation = random(1,3); // 1 or 2  
    rotation = totalsteps/4*rotation; // 90° or 180° moves, make it /8 for 45° increment moves
    steps = rotation*run; // 90° or 180° done 1..4 times.
    homev1 = true;
    spin = random(0,4); // pick a random spin pattern

    stepper.setSpeedInStepsPerSecond(speed);

    if (stopatzero) {
      
      if (abs(motor0steps) == abs(motor1steps) || abs(motor0steps) + abs(motor1steps) == totalsteps) {
        
        delay(1000);

        Serial.println("Trying to step to home!");

        stepper_drivers[0].disableInverseMotorDirection();
        stepper_drivers[1].disableInverseMotorDirection();
        stepper_drivers[2].disableInverseMotorDirection();
        stepper_drivers[3].disableInverseMotorDirection();

        if (abs(motor0steps) == abs(motor1steps)) {
          
          homev1 = true;
          steps = totalsteps-abs(motor0steps);

        } else {
          
          homev1 = false;
          steps = totalsteps-abs(motor0steps);

        }

        if (motor0steps < 0) {

          stepper_drivers[0].enableInverseMotorDirection();

        }
        
        if (motor1steps < 0 && homev1 || motor1steps > 0 && !homev1) {

          stepper_drivers[1].enableInverseMotorDirection();

        }

        if (motor2steps < 0 && homev1 || motor2steps > 0 && !homev1) {

          stepper_drivers[2].enableInverseMotorDirection();

        }

        if (motor3steps < 0) {

          stepper_drivers[3].enableInverseMotorDirection();

        }

        motor0steps = 0;
        motor1steps = 0;
        motor2steps = 0;
        motor3steps = 0;

        spin = -1;

      } else {
        
        Serial.println("Can't calculate a 1-step solution to home.");

        // No 1-step solution for an equal step count.
        // Let's just kick everything 90° and hope it improves.
        //
        run = 1; // one run
        rotation = 1;
        rotation = totalsteps/4*rotation; // 90°
        steps = rotation*run; // move 90° once

      }
    
    }
    
    if (spin == 0) {

      stepper_drivers[0].enableInverseMotorDirection();
      stepper_drivers[1].enableInverseMotorDirection();
      stepper_drivers[2].enableInverseMotorDirection();
      stepper_drivers[3].enableInverseMotorDirection();

      motor0steps -= steps;
      motor1steps -= steps;
      motor2steps -= steps;
      motor3steps -= steps;

    } else if (spin == 1) {

      stepper_drivers[0].disableInverseMotorDirection();
      stepper_drivers[1].disableInverseMotorDirection();
      stepper_drivers[2].disableInverseMotorDirection();
      stepper_drivers[3].disableInverseMotorDirection();

      motor0steps += steps;
      motor1steps += steps;
      motor2steps += steps;
      motor3steps += steps;

    } else if (spin == 2) {

      stepper_drivers[0].enableInverseMotorDirection();
      stepper_drivers[1].disableInverseMotorDirection();
      stepper_drivers[2].disableInverseMotorDirection();
      stepper_drivers[3].enableInverseMotorDirection();

      motor0steps -= steps;
      motor1steps += steps;
      motor2steps += steps;
      motor3steps -= steps;

    } else if (spin == 3) {

      stepper_drivers[0].disableInverseMotorDirection();
      stepper_drivers[1].enableInverseMotorDirection();
      stepper_drivers[2].enableInverseMotorDirection();
      stepper_drivers[3].disableInverseMotorDirection();

      motor0steps += steps;
      motor1steps -= steps;
      motor2steps -= steps;
      motor3steps += steps;

    }

    Serial.printf("Moving to:  m0 = %d, m1 = %d, m2 = %d, m3 = %d\n", motor0steps, motor1steps, motor2steps, motor3steps);
    
    // Keep track of the minimal steps back to 0, rather than 
    // the total steps which may just increase forever.
    // This is used for homing.
    //
    motor0steps = motor0steps % totalsteps;
    motor1steps = motor1steps % totalsteps;
    motor2steps = motor2steps % totalsteps;
    motor3steps = motor3steps % totalsteps;

    Serial.printf("Speed = %d, Rotation = %d, Run = %d, Steps = %d, Spin Style = %d\n", speed, rotation, run, steps, spin);
    
    stepper.setTargetPositionRelativeInSteps(steps);

  }

}

