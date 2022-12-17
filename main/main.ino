#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/Imu.h>

ros::NodeHandle nh;

// Servo
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
const uint8_t PINS[6] = {0, 1, 2, 3, 4, 5};
const uint8_t MINS[6] = {122, 110, 115, 102, 102, 102}; // 102, 110
const int MAXS[6] = {490, 487, 485, 491, 491, 491}; // 491, 480
uint16_t angles[6] = {90, 90, 90, 90, 90, 90};

void setAngle(int id, int angle)
{
  pwm.setPWM(PINS[id], 0, map(angle, 0, 180, MINS[id], MAXS[id]));
}

void servoCb(const std_msgs::Int16MultiArray& msg)
{
  for (int i = 0; i < 6; ++i)
  {
    angles[i] = msg.data[i];
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> servo_sub("servo/command", servoCb);

void servoSetup()
{
  pwm.begin();
  pwm.setPWMFreq(50);
  nh.subscribe(servo_sub);
}

void servoLoop()
{
  for (int i = 0; i < 6; ++i)
  {
    setAngle(i, angles[i]);
  }
}

// Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
sensor_msgs::Imu imu_msg;
std_msgs::UInt8MultiArray calib_msg;

ros::Publisher imu_pub("sensor/imu", &imu_msg);
ros::Publisher calib_pub("sensor/calib", &calib_msg);

void sensorSetup()
{
  bno.begin();
  bno.setExtCrystalUse(true);

  nh.advertise(imu_pub);
  nh.advertise(calib_pub);

  calib_msg.data = (uint8_t*)malloc(sizeof(int8_t) * 4);
  calib_msg.data_length = 4;
}

void sensorLoop()
{
  imu::Quaternion quat = bno.getQuat();
  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();

  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu_msg.linear_acceleration.x = acc.x();
  imu_msg.linear_acceleration.y = acc.y();
  imu_msg.linear_acceleration.z = acc.z();
  
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  calib_msg.data[0] = system;
  calib_msg.data[1] = gyro;
  calib_msg.data[2] = accel;
  calib_msg.data[3] = mag;

  imu_pub.publish(&imu_msg);
  calib_pub.publish(&calib_msg);
}

// main
unsigned long servo_timer = 0;
unsigned long sensor_timer = 0;

void setup() {
  nh.initNode();

  servoSetup();
  sensorSetup();

  delay(1000);
}

void loop() {
  unsigned long now = millis();

  if ((now - servo_timer) > 50 ) {
    servoLoop();
    servo_timer = now;
  }

  if ((now - sensor_timer) > 100 ) {
    sensorLoop();
    sensor_timer = now;
  }
  
  nh.spinOnce();
}
