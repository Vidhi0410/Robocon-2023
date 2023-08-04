// Import Required Libraries
#include <Cytron_SmartDriveDuo.h>
#include "CytronMotorDriver.h"
#include <HardwareSerial.h>
#include <math.h>
#include <ODriveArduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <XBOXRECV.h>


// Controller ID and Wheels Max Speed
#define conid 0
#define maxSpeed 150
#define minSpeed 0

// MDDS30 for all 4 wheels
#define mdds_2_1 29
#define mdds_4_3 27

// Pnumatic Relay to push rings into place
#define pneumatic 35

// Loading Motor to push rings from pulley to platform
#define ldg_pwm 6
#define ldg_dir 31

// Pulley to pick up rings
#define ply_pwm 4
#define ply_dir 23

// Linear Actuator to change platform angle
#define lnr_pwm 5
#define lnr_dir 25

// ODrive Motor Number
#define motornum 0

// Initialize ODrive and Cytron Smart Drive Duo and XBOX Reciver
HardwareSerial& odrive_serial = Serial1;
ODriveArduino odrive(odrive_serial);

Cytron_SmartDriveDuo motor2motor1(SERIAL_SIMPLIFIED, mdds_2_1, 115200);
Cytron_SmartDriveDuo motor4motor3(SERIAL_SIMPLIFIED, mdds_4_3, 115200);

USB Usb;
XBOXRECV Xbox(&Usb);




// Variables
bool loadingWheelStatus = false, pneumaticState = false;
int linearActuatorStatus = 0, requested_state, plySpeed = 0, m1Speed = 0, m2Speed = 0, m3Speed = 0, m4Speed = 0;
float odrivePosition = 0, initialPosition = 0;




void updateMotors(int XSpeed, int YSpeed, int TSpeed) {

  // Check if input is above Minimum Speed
  if (XSpeed < minSpeed && XSpeed > -minSpeed) {
    XSpeed = 0;
  }
  if (YSpeed < minSpeed && YSpeed > -minSpeed) {
    YSpeed = 0;
  }
  if (TSpeed < minSpeed && TSpeed > -minSpeed) {
    TSpeed = 0;
  }

  // Calculate speed for each Motor
  m1Speed = constrain(-XSpeed - TSpeed, -maxSpeed, maxSpeed);
  m2Speed = constrain(YSpeed + TSpeed, -maxSpeed, maxSpeed);
  m3Speed = constrain(XSpeed - TSpeed, -maxSpeed, maxSpeed);
  m4Speed = constrain(-YSpeed + TSpeed, -maxSpeed, maxSpeed);

  // Map value to -100 to 100
  m1Speed = map(m1Speed, -255, 255, -100, 100);
  m2Speed = map(m2Speed, -255, 255, -100, 100);
  m3Speed = map(m3Speed, -255, 255, -100, 100);
  m4Speed = map(m4Speed, -255, 255, -100, 100);

  // Send values to Cytron
  motor2motor1.control(m2Speed, m1Speed);
  motor4motor3.control(m4Speed, m3Speed);
}

void updatePulley(int ltSpeed, int rtSpeed) {
  plySpeed = constrain(rtSpeed - ltSpeed, -maxSpeed, maxSpeed);

  digitalWrite(ply_dir, (plySpeed > 0) ? true : false);

  analogWrite(ply_pwm, abs(plySpeed));
}

void updateLinearActuator(bool ltButton, bool rtButton) {
  if (rtButton) {
    digitalWrite(lnr_dir, LOW);
    analogWrite(lnr_pwm, 255);
  } else if (ltButton) {
    digitalWrite(lnr_dir, HIGH);
    analogWrite(lnr_pwm, 255);
  } else {
    digitalWrite(lnr_dir, LOW);
    analogWrite(lnr_pwm, 0);
  }
}

void updateLoadingWheel() {
  if (!loadingWheelStatus) {
    loadingWheelStatus = true;
    analogWrite(ldg_pwm, 70);
    digitalWrite(ldg_dir, HIGH);
  } else {
    loadingWheelStatus = false;
    analogWrite(ldg_pwm, 0);
    digitalWrite(ldg_dir, LOW);
  }
}

void updatePneumaticPiston() {
  if (pneumaticState) {
    digitalWrite(pneumatic, HIGH);
    pneumaticState = false;
  } else {
    digitalWrite(pneumatic, LOW);
    pneumaticState = true;
  }
}

void setup() {
  // Start XBOX Receiver
  Wire.begin();
  Serial.begin(115200);
#if !defined(MIPSEL)
  while (!Serial)
    ;
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));

  // Start ODrive
  while (!Serial)
    ;
  odrive_serial.begin(115200);
  // Set pinMode for all output pins
  pinMode(pneumatic, INPUT_PULLUP);
  pinMode(pneumatic, OUTPUT);
  digitalWrite(pneumatic, HIGH);
  pinMode(ldg_pwm, OUTPUT);
  pinMode(ldg_dir, OUTPUT);
  digitalWrite(ldg_dir, LOW);
  pinMode(ply_pwm, OUTPUT);
  pinMode(ply_dir, OUTPUT);
  pinMode(lnr_pwm, OUTPUT);
  pinMode(lnr_dir, OUTPUT);
}





void loop() {
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    if (Xbox.Xbox360Connected[conid]) {

      // Code to update Motor Speed
      int rightHatX = Xbox.getAnalogHat(RightHatX, conid);
      int leftHatY = Xbox.getAnalogHat(LeftHatY, conid);
      int leftHatX = Xbox.getAnalogHat(LeftHatX, conid);
      int XSpeed = 0, YSpeed = 0, TSpeed = 0;

      if (leftHatX > 7500) {
        XSpeed = map(leftHatX, 7500, 32768, 0, 255);
      } else if (leftHatX < -7500) {
        XSpeed = map(leftHatX, -32768, -7500, -255, 0);
      }
      if (leftHatY > 7500) {
        YSpeed = map(leftHatY, 7500, 32768, 0, 255);
      } else if (leftHatY < -7500) {
        YSpeed = map(leftHatY, -32768, -7500, -255, 0);
      }
      if (rightHatX > 7500) {
        TSpeed = map(rightHatX, 7500, 32768, 0, 255);
      } else if (rightHatX < -7500) {
        TSpeed = map(rightHatX, -32768, -7500, -255, 0);
      }

      updateMotors(XSpeed, YSpeed, TSpeed);
      Serial.print("Motor 1: ");
      Serial.print(m1Speed);
      Serial.print("  Motor 2: ");
      Serial.print(m2Speed);
      Serial.print("  Motor 3: ");
      Serial.print(m3Speed);
      Serial.print("  Motor 4: ");
      Serial.print(m4Speed);


      // Code to update Pulley Speed
      int ltSpeed = Xbox.getButtonPress(LT, conid);
      int rtSpeed = Xbox.getButtonPress(RT, conid);
      if (ltSpeed > 20 || ltSpeed < 20 || rtSpeed > 20 || rtSpeed < 20) {
        updatePulley(ltSpeed, rtSpeed);
      } else {
        updatePulley(0, 0);
      }
      Serial.print("  Pulley: ");
      Serial.print(plySpeed);


      // Code to update Linear Actuator
      int ltButton = Xbox.getButtonPress(LB, conid);
      int rtButton = Xbox.getButtonPress(RB, conid);
      if (ltButton && !rtButton || rtButton && !ltButton) {
        updateLinearActuator(ltButton, rtButton);
      } else {
        updateLinearActuator(false, false);
      }
      Serial.print("  Linear: ");
      Serial.print((linearActuatorStatus > 0 ? "Up" : (linearActuatorStatus < 0 ? "Down" : "Off")));


      // Code to toggle Loading Wheel
      if (Xbox.getButtonClick(B, conid)) {
        updateLoadingWheel();
      }
      Serial.print("  Loading Wheel: ");
      Serial.print(loadingWheelStatus == 0 ? "Off" : "On");


      // Toggle Pneumatic
      if (Xbox.getButtonClick(Y, conid)) {
        updatePneumaticPiston();
      }


      // Save ODrive last position
      if (Xbox.getButtonClick(X, conid)) {
        initialPosition = odrive.GetPosition(motornum);
      }


      // ODrive Code
      if (!pneumaticState) {
        if (Xbox.getButtonPress(BACK, conid)) {
          odrivePosition = odrive.GetPosition(motornum) - 0.05;
          Serial.print(odrivePosition);
          odrive.SetPosition(motornum, odrivePosition);
        }

        if (Xbox.getButtonPress(START, conid)) {
          odrivePosition = odrive.GetPosition(motornum) + 0.05;
          Serial.print(odrivePosition);
          odrive.SetPosition(motornum, odrivePosition);
        }

        //Return to start
        if (Xbox.getButtonClick(RIGHT, conid)) {
          int currentPosition = odrive.GetPosition(motornum);
          initialPosition = round(currentPosition) + fmod(initialPosition, 1.0);
          odrive.SetPosition(motornum, initialPosition);
        }

        //Type 1
        if (Xbox.getButtonClick(UP, conid)) {
          odrivePosition = odrive.GetPosition(motornum) - 0.7;
          odrive.SetPosition(motornum, odrivePosition);
          delay(1500);
          odrivePosition = odrive.GetPosition(motornum) + 0.7;
          odrive.SetPosition(motornum, odrivePosition);
        }

        //Typhe 2
        if (Xbox.getButtonClick(LEFT, conid)) {
          odrivePosition = odrive.GetPosition(motornum) - 1.2;
          odrive.SetPosition(motornum, odrivePosition);
          delay(2000);
          odrivePosition = odrive.GetPosition(motornum) + 1.2;
          odrive.SetPosition(motornum, odrivePosition);
        }

        //Type 3
        if (Xbox.getButtonClick(DOWN, conid)) {
          odrivePosition = odrive.GetPosition(motornum) - 1.8;
          odrive.SetPosition(motornum, odrivePosition);
          delay(2500);
          odrivePosition = odrive.GetPosition(motornum) + 1.8;
          odrive.SetPosition(motornum, odrivePosition);
        }

        // Calibrate ODrive
        if (Xbox.getButtonClick(A, conid)) {
          requested_state = AXIS_STATE_MOTOR_CALIBRATION;
          if (!odrive.run_state(motornum, requested_state, true)) return;

          requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          if (!odrive.run_state(motornum, requested_state, true, 25.0f)) return;

          requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          if (!odrive.run_state(motornum, requested_state, false)) return;
        }

        if (Xbox.getButtonClick(L3, conid)) {
          int currentPosition = odrive.GetPosition(motornum);
          initialPosition = round(currentPosition) + fmod(initialPosition, 1.0);
          odrive.SetPosition(motornum, initialPosition - 0.5);
          for (int i = 0; i < 500; i++) {
            delay(1);
            updateMotors(0, 0, 0);
            updatePulley(0, 0);
            updateLinearActuator(false, false);
          }
          updatePneumaticPiston();
          for (int i = 0; i < 500; i++) {
            delay(1);
            updateMotors(0, 0, 0);
            updatePulley(0, 0);
            updateLinearActuator(false, false);
          }
          updatePneumaticPiston();
          for (int i = 0; i < 500; i++) {
            delay(1);
            updateMotors(0, 0, 0);
            updatePulley(0, 0);
            updateLinearActuator(false, false);
          }
          initialPosition = round(currentPosition) + fmod(initialPosition, 1.0);
          int _currentPosition = odrive.GetPosition(motornum);
          Serial.println(_currentPosition);
          odrive.SetPosition(motornum, initialPosition);
          Serial.print(" Initial Position");
          delay(1000);
          odrive.SetPosition(motornum, initialPosition + 0.05);
          delay(300);
          odrive.SetPosition(motornum, initialPosition + 0.1);
          delay(300);
          odrive.SetPosition(motornum, initialPosition + 0.15);
          delay(300);
          odrive.SetPosition(motornum, initialPosition + 0.2);
          delay(300);
          odrive.SetPosition(motornum, initialPosition + 0.25);
          delay(300);
          odrive.SetPosition(motornum, initialPosition + 0.3);
          delay(300);
          odrive.SetPosition(motornum, initialPosition + 0.33);
          delay(300);
          odrive.SetPosition(motornum, odrive.GetPosition(motornum) - 0.6);
        }
      }

    } else {
      updateMotors(0, 0, 0);
      updatePulley(0, 0);
      updateLinearActuator(false, false);
    }
  } else {
    updateMotors(0, 0, 0);
    updatePulley(0, 0);
    updateLinearActuator(false, false);
  }
}