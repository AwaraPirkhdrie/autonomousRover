

#include <ros.h>
#include <std_msgs/String.h>

#define voltageSensorPin  3 
#define rightEncoderPinA  2
#define rightEncoderPinB  4
#define leftEncoderPinA  3  
#define leftEncoderPinB  5
#define oneRotation  6000.0 
#define CIRCUMFERENCE 15.4488
#define rightMotorMin 1 
#define rightMotorStop 64 
#define rightMotorMax 127 
#define leftMotorMin 128
#define leftMotorStop 192
#define leftMotorMax 255
#define enablesPWMSignalForMotorA 9
#define in1EnableMotorA 6 
#define in2EnableMotorA 7 
#define enablesPWMSignalForMotorB 8
#define in1EnableMotorB 10
#define in2EnableMotorB 11

int rotDirection = 0;
int pressed = false;
volatile long rightEncoderPos = 0;
volatile long leftEncoderPos = 0;
ros::NodeHandle_<ArduinoHardware, 3, 3, 125, 125> nh;
long lastMotorCtrlTime = 0;

void onMotorCtrlMsg( const std_msgs::String& msg) {
  String statusMsg = "OK";
  if (cStrLen(msg.data) < 2) {
    motorStop();
    statusMsg = "Too Small";
    publishSensorData(statusMsg);
    return;
  }

  byte leftR = msg.data[0];
  byte rightR = msg.data[1];

  if ((int)leftR < leftMotorMin || leftR > leftMotorMax) {
    motorStop();
    statusMsg = "Left Motor Error";
    publishSensorData(statusMsg);
    return;
  } else if ((int)R < rightMotorMin || R > rightMotorMax) {
    motorStop();
    statusMsg = "Right Motor Error";
    publishSensorData(statusMsg);
    return;
  }
  lastMotorCtrlTime = millis();
  unsigned char leftDir = (leftR < leftMotorStop) ? 'R' : 'F';
  unsigned char rightDir = (R < rightMotorStop) ? 'R' : 'F';
  publishSensorData(statusMsg);
  motorSpeed(leftR, R);
}

std_msgs::String sensorMsg;
ros::Publisher sensorTopic("/hercules/sensors", &sensorMsg);

void publishSensorData(String msg) {
  String sensorData = "";
  sensorData.concat("RE:");
  sensorData.concat(getRightEncoder());
  sensorData.concat(";LE:");
  sensorData.concat(getLeftEncoder());
  sensorData.concat(";VI:");
  char buffer[10];
  dtostrf(readVoltage(),3,1,buffer);
  sensorData.concat(buffer);
  sensorData.concat(";LM:");
  sensorData.concat(msg);
  sensorMsg.data = sensorData.c_str();
  sensorTopic.publish( &sensorMsg );
}

int getRightEncoder() {
  long localEncoder = rightEncoderPos;
  rightEncoderPos = 0;
  return ceil((localEncoder / oneRotation) * CIRCUMFERENCE);
}

int getLeftEncoder() {
  long localEncoder = leftEncoderPos;
  leftEncoderPos = 0;
  return ceil((localEncoder / oneRotation) * CIRCUMFERENCE);
}

ros::Subscriber<std_msgs::String> sub("/hercules/motorCtrl", onMotorCtrlMsg );
void setup() {
  delay(10000);
  pinMode(rightEncoderPinA, INPUT);
  digitalWrite(rightEncoderPinA, HIGH);
  pinMode(rightEncoderPinB, INPUT);
  digitalWrite(rightEncoderPinB, HIGH);
  attachInterrupt(0, dorightEncoder, CHANGE);
  pinMode(leftEncoderPinA, INPUT);
  digitalWrite(leftEncoderPinA, HIGH);
  pinMode(leftEncoderPinB, INPUT);
  digitalWrite(leftEncoderPinB, HIGH);
  pinMode(enablesPWMSignalForMotorA, OUTPUT);
  pinMode(in1EnableMotorA, OUTPUT);
  pinMode(in2EnableMotorA, OUTPUT);
  pinMode(button, INPUT);
  digitalWrite(in1EnableMotorA, LOW);
  digitalWrite(in2EnableMotorA, HIGH);
  attachInterrupt(1, doleftEncoder, CHANGE);
  Serial2.begin(9600);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(sensorTopic);
  nh.subscribe(sub);
  nh.logwarn("Starting...");
}

long lastPub = 0;

void loop() {
  if (millis() - lastMotorCtrlTime > 2000) {
    nh.logwarn("Motor stop initiated...");
    motorStop();
    lastMotorCtrlTime = millis();
  }

  if (millis() - lastPub > 100) {
    publishSensorData("OK");
    lastPub = millis();
  }
  nh.spinOnce();
  delay(2000);
}

void dorightEncoder() {
  if (digitalRead(rightEncoderPinA) == digitalRead(rightEncoderPinB)) {
    rightEncoderPos--; 
  } else {
    rightEncoderPos++; 
  }
}

void motorBehavior() {
  int pwmOutput = map(potValue, 0, 1023, 0 , 255); 
  analogWrite(enablesPWMSignalForMotorA, pwmOutput); 
  if (digitalRead(button) == true) {
    pressed = !pressed;
  }
  while (digitalRead(button) == true);
  delay(20);
  if (pressed == true  & rotDirection == 0) {
    digitalWrite(in1EnableMotorA, HIGH);
    digitalWrite(in2EnableMotorA, LOW);
    rotDirection = 1;
    delay(20);
  }
  if (pressed == false & rotDirection == 1) {
    digitalWrite(in1EnableMotorA, LOW);
    digitalWrite(in2EnableMotorA, HIGH);
    rotDirection = 0;
    delay(20);
  }
}

void doleftEncoder() {
  if (digitalRead(leftEncoderPinA) == digitalRead(leftEncoderPinB)) {
    leftEncoderPos++;
  } else {
    leftEncoderPos--;
  }
}

void motorStop() {
  Serial2.write((byte)0);
}

void motorSpeed(byte leftR, byte R) {
  Serial2.write(leftR);
  Serial2.write(R);
}

float readVoltage() {
  float temp;
  int val11 = analogRead(voltageSensorPin);
  temp = val11 / 4.092;
  return (temp/10) - 1.2;  
}

int cStrLen(const char* cstr) {
  int counter = 0;
  while (cstr[counter] != '\0') {
    counter++;
  }
  return counter;
}
