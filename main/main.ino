#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
// #include <std_msgs/Int32MultiArray.h>
//#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;

// Servo
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
const uint8_t PINS[6] = {0, 1, 2, 3, 4, 5};
const uint8_t MINS[6] = {122, 110, 115, 102, 102, 102}; // 102, 110
const int MAXS[6] = {490, 487, 485, 491, 491, 491}; // 491, 480
int angles[6] = {90, 90, 90, 90, 90, 90};

void servoSetup()
{ 
  pwm.begin();         // 初期設定
  pwm.setPWMFreq(50);  // PWM周期を50Hzに設定
}

void setAngle(int id, int angle)
{
  pwm.setPWM(PINS[id], 0, map(angle, 0, 180, MINS[id], MAXS[id]));
}

void servoCb(const std_msgs::Float32MultiArray& msg)
{
  //angles = msg;
  for (int i = 0; i < 6; ++i)
  {
    angles[i] = msg.data[i];
  }

  if (msg.data[2] == 0) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> servo_sub("servo/command", servoCb);

void servoLoop()
{
  for (int i = 0; i < 6; ++i)
  {
    setAngle(i, angles[i]);
  }
}

void setup() {
  nh.initNode();
  servoSetup();

  pinMode(13, OUTPUT);

  nh.subscribe(servo_sub);
  
  delay(1000);
}

void loop() {
  nh.spinOnce();

  servoLoop();
  delay(100);

}
