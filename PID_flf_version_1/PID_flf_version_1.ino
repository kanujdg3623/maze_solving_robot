#include <QTRSensors.h>

const uint8_t BUZZER=9;
const uint8_t BLUE_LED=12;

const uint8_t LEFT_MOTOR_A=5;
const uint8_t LEFT_MOTOR_B=6;

const uint8_t RIGHT_MOTOR_A=3;
const uint8_t RIGHT_MOTOR_B=11;

const uint8_t PUSH_BUTTON=4;
 
const uint8_t LEFT=0;
const uint8_t RIGHT=1;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
QTRSensors qtr;

void setup()
{
  pinMode(BLUE_LED, OUTPUT);
  pinMode(BUZZER,OUTPUT);
  pinMode(BLUE_LED,OUTPUT);
  pinMode(LEFT_MOTOR_A,OUTPUT);
  pinMode(LEFT_MOTOR_B,OUTPUT);
  pinMode(RIGHT_MOTOR_A,OUTPUT);
  pinMode(RIGHT_MOTOR_B,OUTPUT);
  pinMode(PUSH_BUTTON,INPUT_PULLUP);
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(10);

  delay(1000);
  
  digitalWrite(BLUE_LED, HIGH);
  for (uint16_t i = 0; i < 100; i++)
  {
    if(i<25 || i>=75)
      maneuver(LEFT,64);
    else
      maneuver(RIGHT,64);
    qtr.calibrate();
  }
  maneuver(0,0);
  digitalWrite(BLUE_LED, LOW);
  
  Serial.begin(57600);
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  Serial.println("PUSH BUTTON");
  while(!digitalRead(PUSH_BUTTON))
  delay(1);
}

void loop()
{
//  uint16_t position = qtr.readLineWhite(sensorValues);
  qtr.readCalibrated(sensorValues);
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.println(position);
  Serial.println("");
  delay(250);
}

void maneuver(uint8_t stat, double Speed)
{
  analogWrite(LEFT_MOTOR_A,stat*Speed);
  analogWrite(LEFT_MOTOR_B,(1-stat)*Speed);
  analogWrite(RIGHT_MOTOR_A,(1-stat)*Speed);
  analogWrite(RIGHT_MOTOR_B,stat*Speed);
}
