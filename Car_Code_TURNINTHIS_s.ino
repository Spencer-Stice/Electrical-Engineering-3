//Self-Driving Car implementation. Created by Spencer Stice and Andrew Sonico Eugenio.
#include <ECE3.h>

//Declare variables
uint16_t sensorValues[8];
float minAdjusted[8];
float maxAdjusted[8];
float weighted[8];
float error;
float prevTime;
float prevError;
float nextTime;
float nextError;

int endRun = 0;
int turnFlag = 0;
int W0 = -15;
int W1 = -14;
int W2 = -12;
int W3 = -8;
int W4 = 8;
int W5 = 12;
int W6 = 14;
int W7 = 15;

int propW = 50;
int derivW = 500;
int integW = 0;

int PWML = 100;
int PWMR = 100;

//Define pins
#define LMotor 40
#define RMotor 39
#define LMotorD 29
#define RMotorD 30
#define nslpL 31
#define nslpR 11

void setup()
{
  //Setup pins and initialize values
  pinMode(LMotor, OUTPUT);
  pinMode(RMotor, OUTPUT);
  pinMode(LMotorD, OUTPUT);
  pinMode(RMotorD, OUTPUT);
  pinMode(nslpL, OUTPUT);
  pinMode(nslpR, OUTPUT);
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  digitalWrite(LMotorD, LOW);
  digitalWrite(RMotorD, LOW);
  digitalWrite(nslpL, HIGH);
  digitalWrite(nslpR, HIGH);
  float prevError = 0;
  delay(2000);
}

//Declare and implement function for reading values and obtaining error term

float readValuesandGetError()
{
  ECE3_read_IR(sensorValues);

  minAdjusted[0] = sensorValues[0] - 643;
  minAdjusted[1] = sensorValues[1] - 782;
  minAdjusted[2] = sensorValues[2] - 643;
  minAdjusted[3] = sensorValues[3] - 643;
  minAdjusted[4] = sensorValues[4] - 647;
  minAdjusted[5] = sensorValues[5] - 689;
  minAdjusted[6] = sensorValues[6] - 689;
  minAdjusted[7] = sensorValues[7] - 759;

  maxAdjusted[0] = minAdjusted[0] / 1.857;
  maxAdjusted[1] = minAdjusted[1] / 1.718;
  maxAdjusted[2] = minAdjusted[2] / 1.857;
  maxAdjusted[3] = minAdjusted[3] / 1.750;
  maxAdjusted[4] = minAdjusted[4] / 1.698;
  maxAdjusted[5] = minAdjusted[5] / 1.811;
  maxAdjusted[6] = minAdjusted[6] / 1.811;
  maxAdjusted[7] = minAdjusted[7] / 1.741;

  error = (maxAdjusted[0] * W0) + (maxAdjusted[1] * W1) + (maxAdjusted[2] * W2) +
          (maxAdjusted[3] * W3) + (maxAdjusted[4] * W4) + (maxAdjusted[5] * W5) +
          (maxAdjusted[6] * W6) + (maxAdjusted[7] * W7);

  error /= 8; // GET STANDARD ERROR VALUE

  error /= 2949; // NORMALIZE IT TO 1
  return error;
}


void loop()
{
  while (endRun != 2) {

    readValuesandGetError();

    //Adjust speed based on current error
    if (abs(error) < 0.4) {
      PWML = 185;
      PWMR = 185;
      propW = 122;
      derivW = 1050;
    }
    else{
      PWML = 135;
      PWMR = 135;
      propW = 62;
      derivW = 620;
    }

    float propTerm = propW * error;

    float derivTerm = ((error - prevError)) * (derivW);

    float totalPID = propTerm + derivTerm;
    PWML -=  totalPID;
    PWMR += totalPID;
    prevError = error;

  //If at end of track, turn or stop
    if (maxAdjusted[2] > 800)
      if (maxAdjusted[3] > 800)
        if (maxAdjusted[4] > 800)
          if (maxAdjusted[5] > 800)

            turnFlag++;
    if (maxAdjusted[2] < 800)
      turnFlag = 0;
    if (turnFlag == 2)
    {
      endRun++;
      if (endRun == 2)
      {
        analogWrite(LMotor, 0);
        analogWrite(RMotor, 0);
      }
      else {
        analogWrite(LMotor, 0);
        analogWrite(RMotor, 0);
        delay(100);
        digitalWrite(LMotorD, HIGH);
        analogWrite(LMotor, 200);
        analogWrite(RMotor, 200);
        delay(305);
        digitalWrite(LMotorD, LOW);
        turnFlag = 0;
        analogWrite(LMotor, 0);
        analogWrite(RMotor, 0);
        delay(100);
        analogWrite(LMotor, 100);
        analogWrite(RMotor, 100);
        delay(500);
      }
    }
    else {
      analogWrite(LMotor, PWML);
      analogWrite(RMotor, PWMR);
    }
  }
}
