#include <QTRSensors.h>
#include <PID_v1.h>

const uint8_t BUZZER=9;
const uint8_t BLUE_LED=12;

const uint8_t LEFT_MOTOR_A=5;
const uint8_t LEFT_MOTOR_B=6;

const uint8_t RIGHT_MOTOR_A=3;
const uint8_t RIGHT_MOTOR_B=11;

const uint8_t PUSH_BUTTON=4;

const uint8_t BACKWARD=0;
const uint8_t FORWARD=1;

const uint8_t RIGHT=0;
const uint8_t LEFT=7;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint8_t flag;
double Input, Output;
double Setpoint;
double Kp=0.04, Kd=0.001, Ki=0.1;

QTRSensors qtr;
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

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
      maneuver(BACKWARD,FORWARD,64,64);
    else
      maneuver(FORWARD,BACKWARD,64,64);
    qtr.calibrate();
  }
  maneuver(0,0,0,0);
  digitalWrite(BLUE_LED, LOW);
  
  Serial.begin(57600);
  
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);  

  wait();
  
  Input =(double) qtr.readLineWhite(sensorValues);
  Setpoint=Input;
}

void loop()
{
  maneuver(FORWARD,FORWARD,225,225);
  qtr.readCalibrated(sensorValues);
  if(sensorValues[LEFT]<800 || sensorValues[RIGHT]<800)      //---EITHER left or right most is white
  {
    int i, delayTime=10;
    for(i=0;i<delayTime;i++)
    {
      qtr.readCalibrated(sensorValues);
      if(sensorValues[LEFT]<800)                             //if left most is white 
      {
        if(sensorValues[LEFT]<800 && sensorValues[RIGHT]<800)//if both left and right most white
        { flag=FORWARD; break; }
        else
        flag=LEFT;
      }
      else if(sensorValues[RIGHT]<800)                       //if right most is white
      {
        if(sensorValues[LEFT]<800 && sensorValues[RIGHT]<800)//if  both left and right most white
        { flag=FORWARD; break; }
        else
        flag=RIGHT;
      }
      delay(1);
    }
    delayTime=delayTime-i;
    if(flag==FORWARD)                                         //--IF  both left and right most white
    { 
      Serial.print("FORWARD ");
      while(delayTime--)
      delay(1);
      delay(100);
      qtr.readCalibrated(sensorValues);
      if(sensorValues[LEFT]<800 && sensorValues[RIGHT]<800)   //if  both left and right most white
      {
        Serial.println("STOP");
      }
      else if(sensorValues[(LEFT+RIGHT)/2]<800 || sensorValues[(LEFT+RIGHT)/2+1]<800)
      {                                                       //if either middle left or right white
        Serial.println("STRAIGHT");
      }
      else 
      {
        Serial.println("TURNS");
      }
      maneuver(0,0,0,0);
      wait();
    }
    else if(flag==RIGHT)                                      //--IF right most is white
    {
      Serial.print("RIGHT ");
      delay(100);
      qtr.readCalibrated(sensorValues);
      if(sensorValues[(LEFT+RIGHT)/2]<800 || sensorValues[(LEFT+RIGHT)/2+1]<800)
      {                                                       //if either middle left or right white
        Serial.println("STRAIGHT");
      }
      else 
      {
        Serial.println("TURNS");
      }
      maneuver(0,0,0,0);
      wait();
    }
    else if(flag==LEFT)                                       //--IF left most is white
    {
      Serial.print("LEFT ");
      delay(100);
      qtr.readCalibrated(sensorValues);
      if(sensorValues[(LEFT+RIGHT)/2]<800 || sensorValues[(LEFT+RIGHT)/2+1]<800)
      {                                                       //if either middle left or right white
        Serial.println("STRAIGHT");
      }
      else 
      {
        Serial.println("TURNS");
      }
      maneuver(0,0,0,0);
      wait();
    }
  }
  else if(sensorValues[(LEFT+RIGHT)/2]>800 && sensorValues[(LEFT+RIGHT)/2+1]>800)
  {                                                           //---IF both middle left and right sensor black
    Serial.println("DEAD END");
    maneuver(0,0,0,0);
    wait();
  }
}

void lineFollow()
{
  Input =(double) qtr.readLineWhite(sensorValues);
  pid.Compute();
  maneuver(FORWARD,FORWARD,225+Output,225-Output);
}

void wait()
{
  while(digitalRead(PUSH_BUTTON))
  delay(1);
  delay(500);
}

void maneuver(uint8_t leftStat, uint8_t rightStat, double leftSpeed, double rightSpeed)
{
  analogWrite(LEFT_MOTOR_A,constrain(leftStat*leftSpeed,0,255));
  analogWrite(LEFT_MOTOR_B,constrain((1-leftStat)*leftSpeed,0,255));
  analogWrite(RIGHT_MOTOR_A,constrain(rightStat*rightSpeed,0,255));
  analogWrite(RIGHT_MOTOR_B,constrain((1-rightStat)*rightSpeed,0,255));
}
