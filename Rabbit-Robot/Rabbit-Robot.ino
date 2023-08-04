#include <Wire.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#include <math.h>
#include "CytronMotorDriver.h"

#define maxSpeed 150

#define p1 3
#define p2 4
#define p3 6
#define p4 7
#define pul_pwm 5

#define m1 22
#define m2 23
#define m3 27
#define m4 30
#define pul_dir 26
#define s1_p 8
#define s1_dir 31
#define s2 34

CytronMD motor1(PWM_DIR, p1, m1);
CytronMD motor2(PWM_DIR, p2, m2);
CytronMD motor3(PWM_DIR, p3, m3);
CytronMD motor4(PWM_DIR, p4, m4);
CytronMD pulley(PWM_DIR, pul_pwm, pul_dir);
CytronMD s1_cytron(PWM_DIR, s1_p, s1_dir);

double fmap(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void updateMotors(int m1Speed, int m2Speed, int m3Speed, int m4Speed, int pulley_speed) {

  m1Speed = map(m1Speed, 255, -255, maxSpeed, -maxSpeed);
  m2Speed = map(m2Speed, 255, -255, maxSpeed, -maxSpeed);
  m3Speed = map(m3Speed, 255, -255, maxSpeed, -maxSpeed);
  m4Speed = map(m4Speed, 255, -255, maxSpeed, -maxSpeed);

  Serial.print(-m1Speed);
  Serial.print(" ");
  Serial.print(-m2Speed);
  Serial.print(" ");
  Serial.print(m3Speed);
  Serial.print(" ");
  Serial.print(m4Speed);
  Serial.print(" ");
  Serial.print(pulley_speed);
  Serial.println(" ");
  motor1.setSpeed(-m1Speed);
  motor2.setSpeed(-m2Speed);
  motor3.setSpeed(m3Speed);
  motor4.setSpeed(m4Speed);
  pulley.setSpeed(pulley_speed);
}


void setup() {
  Serial.begin(115200);

  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
  pinMode(p1, OUTPUT);
  pinMode(p2, OUTPUT);
  pinMode(p3, OUTPUT);
  pinMode(p4, OUTPUT);
}

void loop() {
  int ch1 = pulseIn(43, HIGH);
  int ch2 = pulseIn(45, HIGH);
  int ch3 = pulseIn(47, HIGH);
  int ch4 = pulseIn(49, HIGH);
  int ch5 = pulseIn(51, HIGH);
  int ch6 = pulseIn(53, HIGH);

  Serial.print("Ch1: ");
  Serial.print(ch1);
  Serial.print(" Ch2: ");
  Serial.print(ch2);
  Serial.print(" Ch3: ");
  Serial.print(ch3);
  Serial.print(" Ch4: ");
  Serial.print(ch4);
  Serial.print(" Ch5: ");
  Serial.print(ch5);
  Serial.print(" Ch6: ");
  Serial.println(ch6);

  if ((ch1 > 900 && ch1 < 2000) || (ch2 > 900 && ch2 < 2000) || (ch4 > 900 && ch4 < 2000) || (ch5 > 900 && ch5 < 2000) || (ch3 > 900 && ch3 < 2000) || (ch6 > 900 && ch6 < 2000)) {
    int XSpeed = 0, YSpeed = 0, TSpeed = 0, PulSpeed = 0;
    if (ch1 >= 1558) {
      TSpeed = fmap(ch1, 1558, 1732, 0, 255);
    } else if (ch1 <= 1458) {
      TSpeed = fmap(ch1, 1458, 1183, 0, -255);
    } else {
      TSpeed = 0;
    }
    if (ch2 >= 1552) {
      YSpeed = -fmap(ch2, 1552, 1976, 0, -255);
    } else if (ch2 <= 1452) {
      YSpeed = -fmap(ch2, 1452, 987, 0, 255);
    } else {
      YSpeed = 0;
    }

    if (ch3 < 1100) {
      PulSpeed = fmap(ch3, 980, 1100, -255, 0);
    } else if (ch3 > 1800) {
      PulSpeed = fmap(ch3, 1800, 2000, 0, 255);
    } else {
      PulSpeed = 0;
    }

    if (ch5 > 1500) {
      s1_cytron.setSpeed(255);
      Serial.println("S1 High");
    } else {
      s1_cytron.setSpeed(0);
      Serial.println("S1 Low");
    }

    if (ch6 > 1500) {
      digitalWrite(s2, HIGH);
      Serial.println("S2 High");
    } else {
      digitalWrite(s2, LOW);
      Serial.println("S2 Low");
    }


    int m1Speed = 0, m2Speed = 0, m3Speed = 0, m4Speed = 0;

    m1Speed = constrain((XSpeed + YSpeed + TSpeed), -255, 255);
    m2Speed = constrain((YSpeed - XSpeed - TSpeed), -255, 255);
    m3Speed = constrain((-XSpeed + YSpeed + TSpeed), -255, 255);
    m4Speed = constrain((YSpeed + XSpeed - TSpeed), -255, 255);
    updateMotors(m1Speed, m2Speed, m3Speed, m4Speed, PulSpeed);
  } else {
    updateMotors(0, 0, 0, 0, 0);
  }
}