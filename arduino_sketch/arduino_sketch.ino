#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// --- ROS Node ---
ros::NodeHandle nh;

// --- Motor pins ---
const int leftMotorPWM = 5;   
const int rightMotorPWM = 6;
const int leftMotorDir = 7;
const int rightMotorDir = 8;

// --- Encoder pins ---
const int leftEncoderA = 2;
const int rightEncoderA = 3;

// --- Robot parameters ---
const float wheelRadius = 0.03; // meters
const float wheelBase = 0.16;   // distance between wheels in meters
volatile long leftTicks = 0;
volatile long rightTicks = 0;
const int ticksPerRevolution = 360;

// --- Odometry ---
float x = 0, y = 0, theta = 0;
unsigned long lastOdomTime = 0;
const float ODOM_PUBLISH_RATE = 50; // ms

nav_msgs::Odometry odomMsg;

// --- Callback for cmd_vel ---
void cmdVelCallback(const geometry_msgs::Twist &msg){
  float v = msg.linear.x;
  float w = msg.angular.z;

  float vLeft = v - w*wheelBase/2;
  float vRight = v + w*wheelBase/2;

  int leftPWM = (int)(vLeft / 1.0 * 255);
  int rightPWM = (int)(vRight / 1.0 * 255);

  digitalWrite(leftMotorDir, leftPWM >= 0 ? HIGH : LOW);
  digitalWrite(rightMotorDir, rightPWM >= 0 ? HIGH : LOW);

  analogWrite(leftMotorPWM, abs(leftPWM));
  analogWrite(rightMotorPWM, abs(rightPWM));
}

// --- Subscriber ---
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVelCallback);

// --- Encoder ISRs ---
void leftEncoderISR() { leftTicks++; }
void rightEncoderISR() { rightTicks++; }

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  pinMode(leftMotorPWM, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorDir, OUTPUT);

  pinMode(leftEncoderA, INPUT_PULLUP);
  pinMode(rightEncoderA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderA), rightEncoderISR, RISING);

  lastOdomTime = millis();
}

void loop() {
  nh.spinOnce();
  unsigned long now = millis();
  if(now - lastOdomTime >= ODOM_PUBLISH_RATE){
    lastOdomTime = now;

    float dLeft = (2*PI*wheelRadius) * (leftTicks / (float)ticksPerRevolution);
    float dRight = (2*PI*wheelRadius) * (rightTicks / (float)ticksPerRevolution);

    leftTicks = 0;
    rightTicks = 0;

    float dx = (dRight + dLeft) / 2.0;
    float dtheta = (dRight - dLeft) / wheelBase;

    theta += dtheta;
    x += dx * cos(theta);
    y += dx * sin(theta);

    odomMsg.header.stamp = nh.now();
    odomMsg.header.frame_id = "odom";
    odomMsg.child_frame_id = "base_link";

    odomMsg.pose.pose.position.x = x;
    odomMsg.pose.pose.position.y = y;
    odomMsg.pose.pose.position.z = 0;

    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    odomMsg.pose.pose.orientation.x = q.x();
    odomMsg.pose.pose.orientation.y = q.y();
    odomMsg.pose.pose.orientation.z = q.z();
    odomMsg.pose.pose.orientation.w = q.w();

    odomMsg.twist.twist.linear.x = dx / (ODOM_PUBLISH_RATE/1000.0);
    odomMsg.twist.twist.angular.z = dtheta / (ODOM_PUBLISH_RATE/1000.0);

    static ros::Publisher odomPub("odom", &odomMsg);
    static bool pubInit = false;
    if(!pubInit){
      nh.advertise(odomPub);
      pubInit = true;
    }
    odomPub.publish(&odomMsg);
  }
}
