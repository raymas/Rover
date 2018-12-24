/*
*
######  #### ######## ########     ###
##    ##  ##  ##       ##     ##   ## ##
##        ##  ##       ##     ##  ##   ##
######   ##  ######   ########  ##     ##
     ##  ##  ##       ##   ##   #########
##    ##  ##  ##       ##    ##  ##     ##
######  #### ######## ##     ## ##     ##

*
* SiERA 2017 - ReCover project.
*
* Name:        RecoverMotors
* Purpose:
*
*
* Author:      raymas
*
* Created:     25/01/2017
* Copyright:   (c) raymas 2017
* Licence:
*
* Revision 2.0 adding ROS compatibility
*
* Motor schematics
*
*  \\1-|---|-3//
*      |   |
*      |   |
*  //2-|___|-4\\
*/
#include <Adafruit_MotorShield.h>

//ros includes
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

//recover spec
#include "recover_specifications.h"

#include "Motor.h"


#define REFRESH_TIME 10 //10 ms
#define REFRESH_RATE 100 //10 ms correspond to 100hz


//Global Motor steepers pins : Arduino MEGA has 4 on 18, 19, 20, 21 the adafruit do not use thoses pin
const int pinMotor1=18;
const int pinMotor2=19;
const int pinMotor3=20;
const int pinMotor4=21;

//PID for 10 ms : already tested on DFRobot 12V low noise + encoder
const int Kp=100;
const int Ki=20;
const int Kd=40;

//Time variable
unsigned long lTimer;

//Adafruit main board
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
//Init adafruit motors
Adafruit_DCMotor *mMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *mMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *mMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *mMotor4 = AFMS.getMotor(4);
//Motor class
Motor motors[4];
enum MotorID {
  M1 = 0,
  M2 = 1,
  M3 = 2,
  M4 = 3
};


//Ros variables
ros::NodeHandle nh;
//communication
void handle_velocity_cmd(const geometry_msgs::Twist& cmd_msg);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_velocity_cmd);
ros::Publisher rpm_pub("rpm", &rpm_msg);
//rpm message
geometry_msgs::Vector3Stamped rpm_msg;

//ros time
ros::Time current_time;
ros::Time last_time;



/*
 *
 * Begining of setup
 *
 */
void setup() {

  //PID interrupt
  attachInterrupt(digitalPinToInterrupt(pinMotor1), incrementTickMotor1, RISING);
  attachInterrupt(digitalPinToInterrupt(pinMotor2), incrementTickMotor2, RISING);
  attachInterrupt(digitalPinToInterrupt(pinMotor3), incrementTickMotor3, RISING);
  attachInterrupt(digitalPinToInterrupt(pinMotor4), incrementTickMotor4, RISING);

  //init completed, starting the adafruit board !
  AFMS.begin();

  motors[M1] = Motor(mMotor1);
  motors[M2] = Motor(mMotor2);
  motors[M3] = Motor(mMotor3);
  motors[M4] = Motors(mMotor4);

  //ros init
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(sub);
  nh.advertise(rpm_pub);

  //Starting timer
  lTimer=millis();
}




/*
 *
 * Main loop
 *
 */
void loop() {
  nh.spinOnce()
  unsigned long lTime = millis();
  if (lTime - lTimer >= REFRESH_TIME) {
    for (int m=M1; m <= M4; m++) {
      float command = PID(motors[m]);
      int direction = (command >= 0) ? FORWARD : BACKWARD;
      motors[m].setState(direction);
      motors[m].setSpeed((int) abs(command));
    }
    publishRPM(lTime - lTimer);
    lTimer = millis();
  }
}


/*
 *
 * handle ros commands order
 *
 */
void handle_velocity_cmd(const geometry_msgs::Twist& cmd_msg) {
 double x = cmd_msg.linear.x;
 double z = cmd_msg.angular.z;
 if (z == 0) {
   // only translation
   // convert m/s command to rpm
   for (int m=M1; m <= M4; m++) {
     motor[m].setCommand(speedToRPM(x));
   }
 } else if (x == 0) {
   // convert rad/s to rpm
   // convert m/s rotation to rpm
   // if z > 0 left rotation, z < 0 right rotation
   int sign = 1;
   if (z < 0) {
     sign = -1;
   }
   for (int m=M1; m <= M4; m++) {

   }
   //TODO: RPM command
   rpmCommands[1] = z*track_width*60/(wheel_diameter*pi*2);
   rpmCommands[0] = -rpmCommands[1];
   rpmCommands[1] = z*track_width*60/(wheel_diameter*pi*2);
   rpmCommands[0] = -rpmCommands[1];
 } else {
   rpm_req1 = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
   rpm_req2 = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
 }
}

void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}


/*
 *
 * PID correctSpeed
 *
 */

float PID(Motor motor) {
  float request = motor.getCommand();
  float speed = motor.getRPM();
  float deltaError = (request - speed) - motor.getError();
  float error = request - speed;
  float sumError = motor.getSumError() + error;
  motor.setErrors(error, sumError, deltaError);

  float cmd = Kp * motor.getError() + Ki * motor.getSumError() + Kd * motor.getDeltaError();
  cmd = constrain(cmd, -MAX_RPM, MAX_RPM);
  return cmd;
}



/*
 *
 * attachInterrupt fonctions for PID
 *
 */
void incrementTickMotor1() {
  motors[M1].incrementTickMotor();
}

void incrementTickMotor2() {
  motors[M2].incrementTickMotor();
}

void incrementTickMotor3() {
  motors[M3].incrementTickMotor();
}

void incrementTickMotor4() {
  motors[M4].incrementTickMotor();
}


int speedToRPM(float spd)
{
  return (int) spd*60/(pi*wheel_diameter);
}
