#include <ECE3.h>

uint16_t sensorValues[8]; 

const int left_nslp_pin = 31; 
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11; 
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

int baseSpeed = 20;
float kP = 0.5;
float kD = 6.0;

int Error = 0;
int prevError = 0;
int PIDSUM = 0;
int diffSum = 0;

int FC = 0;
int TC = 0;
bool isRotating = false;
int phantomCount = 0; 

unsigned long rotationStartMillis = 0;
unsigned long previousMillis = 0;
const long interval = 4000;


// MinMax normalization
void normalizeSensorValues(uint16_t* values) {
  uint16_t maxVal = 0;
  uint16_t minVal = 2500; 

  for (int i = 0; i < 8; i++) {
    if (values[i] > maxVal) {
      maxVal = values[i];
    }
    if (values[i] < minVal) {
      minVal = values[i];
    }
  }

  for (int i = 0; i < 8; i++) {
    values[i] = (values[i] - minVal) * 1000 / (maxVal - minVal);
  }
}


//Pinsetup
void setup() {
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH); 

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH); 

  ECE3_Init();

  Serial.begin(9600);
  delay(2000);
}

void loop() {
  int leftSpd = baseSpeed;
  int rightSpd = baseSpeed;

  ECE3_read_IR(sensorValues);

  normalizeSensorValues(sensorValues);

  int weightedSum = (sensorValues[0] * -15 +
                     sensorValues[1] * -14 +
                     sensorValues[2] * -12 +
                     sensorValues[3] * -8 +
                     sensorValues[4] * 8 +
                     sensorValues[5] * 12 +
                     sensorValues[6] * 14 +
                     sensorValues[7] * 15) / 8;

  Error = weightedSum;

  diffSum = (Error - prevError);

  PIDSUM = (kP * Error) + (kD * diffSum);

  leftSpd -= PIDSUM;
  rightSpd += PIDSUM;

//  leftSpd = constrain(leftSpd, 0, 170 + baseSpeed);
//  rightSpd = constrain(rightSpd, 0, 170 + baseSpeed);
  leftSpd = constrain(leftSpd, 0, 170);
  rightSpd = constrain(rightSpd, 0, 170);

  analogWrite(left_pwm_pin, leftSpd);
  analogWrite(right_pwm_pin, rightSpd);

  prevError = Error;

//Set basetime
  unsigned long currentMillis = millis();


//Phantom crosspiece prevention
  if (weightedSum > 3500) {
    phantomCount++;
  } else {
    phantomCount = 0;
  }

//1. Rotation
  if (phantomCount >= 2 && (currentMillis - previousMillis >= interval) && FC == 0) {
    previousMillis = currentMillis;
    analogWrite(left_pwm_pin, 255);
    analogWrite(right_pwm_pin, 255);
    digitalWrite(left_dir_pin, HIGH);
    digitalWrite(right_dir_pin, LOW);
    delay(300);
    digitalWrite(left_dir_pin, LOW);
    digitalWrite(right_dir_pin, LOW);
    rotationStartMillis = currentMillis;
    isRotating = true;
  }

//Multi-triggering prevention
  if (isRotating && (currentMillis - rotationStartMillis >= 1000)) {
    FC += 1;
    isRotating = false;
  }


//2. Stop
  if (phantomCount >= 2 && FC >= 1) {
    previousMillis = currentMillis;
    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);
    digitalWrite(left_nslp_pin, LOW);
    digitalWrite(right_nslp_pin, LOW);
  }

}
