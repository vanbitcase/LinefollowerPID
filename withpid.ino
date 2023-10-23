#include <QTRSensors.h>

#define NUM_SENSORS 8
#define TIMEOUT 4
#define EMITTER_PIN 13

QTRSensors qtra;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int IndicatorLED = 13;
int pinAIN1 = 2;
int pinAIN2 = 4;
int pinPWMA = 5;
int pinBIN1 = 7;
int pinBIN2 = 8;
int pinPWMB = 6;
int pinSTBY = 9;

int position = 0;
int error = 0;
int m1Speed = 0;
int m2Speed = 0;
int motorSpeed = 0;
unsigned long previousMillis = 0;
int BlinkTime = 300;
int CycleTime = 1;
int BlinkCycle = 1;

int lastError = 0;
float KP = 0;
float KD = 0;
int M1 = 150;
int M2 = 150;
int M1max = 255;
int M2max = 255;
int M1min = 0;
int M2min = 0;

static boolean turnCW = 0;
static boolean turnCCW = 1;
static boolean motor1 = 0;
static boolean motor2 = 1;

void setup() {
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {A2, A4, A6, A7, A1, A0, A5, A3}, SensorCount);
  qtra.setEmitterPin(13);

  pinMode(IndicatorLED, OUTPUT);
  digitalWrite(IndicatorLED, HIGH);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinSTBY, OUTPUT);

  for (int i = 0; i < 400; i++) {
    TimeCheck();
    delay(5);
    qtra.calibrate();
  }
}

void loop() {
  TimeCheck();
  uint16_t position = qtra.readLineBlack(sensorValues);
  error = position - 3500;
  motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
  m1Speed = M1 - motorSpeed;
  m2Speed = M2 + motorSpeed;

  if (m1Speed < M1min)
    m1Speed = M1min;
  if (m2Speed < M2min)
    m2Speed = M2min;
  if (m1Speed > M1max)
    m1Speed = M1max;
  if (m2Speed > M2max)
    m2Speed = M2max;

  forward();
}

void forward() {
  TimeCheck();
  motorDrive(motor1, turnCW, m1Speed);
  motorDrive(motor2, turnCW, m2Speed);
}

void TimeCheck() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > BlinkTime) {
    previousMillis = currentMillis;
    BlinkIt();
  }
}

void motorDrive(boolean motorNumber, boolean motorDirection, int motorSpeed) {
  boolean pinIn1;

  if (motorDirection == turnCW)
    pinIn1 = HIGH;
  else
    pinIn1 = LOW;

  if (motorNumber == motor1) {
    digitalWrite(pinAIN1, pinIn1);
    digitalWrite(pinAIN2, !pinIn1);
    analogWrite(pinPWMA, motorSpeed);
  } else {
    digitalWrite(pinBIN1, pinIn1);
    digitalWrite(pinBIN2, !pinIn1);
    analogWrite(pinPWMB, motorSpeed);
  }
  digitalWrite(pinSTBY, HIGH);
}

void motorBrake(boolean motorNumber) {
  if (motorNumber == motor1)
    analogWrite(pinPWMA, 0);
  else
    analogWrite(pinPWMB, 0);
}

void motorStop(boolean motorNumber) {
  if (motorNumber == motor1) {
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, LOW);
  } else {
    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, LOW);
  }
}

void BlinkIt() {
  if (BlinkCycle == 1) {
    digitalWrite(IndicatorLED, HIGH);
  }
  if (BlinkCycle == 2) {
    digitalWrite(IndicatorLED, LOW);
    BlinkCycle = 0;
  }
  BlinkCycle++;
}
