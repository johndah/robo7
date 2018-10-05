/*********************************************************************
* Software License Agreement (BSD License)
*
*  ROS driver for Phidgets motor control HC
*  Copyright (c) 2010, Bob Mottram
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sstream>
#include <phidget21.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include "phidgets/motor_params.h"
#include "phidgets/motor_encoder.h"

// Motor::Motor()
// {
//   node_handle_ = ros::NodeHandle("~");
//   if(!node_handle_.getParam("serial", serial_number_))
//   {
//     ROS_ERROR("Motor requires a serial number to be initialized!");
//     exit(-1);
//   }
//
//   if(!node_handle_.getParam("name", motor_name_))
//   {
//     motor_name_ = "default_motor";
//   }
//
//   if(!node_handle_.getParam("default_topic_path", topic_path_))
//   {
//
//   }
//
//   node_handle_.getParam("motor_acceleration", acceleration_);
//   node_handle_.getParam("invert_motor", invert_motor_);
//   node_handle_.getParam("default_timeout", timeout_sec_);
//   node_handle_.getParam("default_frequency", frequency_);
//   motors_active_ = false;
//   initialised_ = false;
//
//   float timeout_sec = 0.5;
//   nh.getParam("timeout", timeout_sec);
//   int frequency = 30;
//   nh.getParam("frequency", frequency);
// }

// handle
CPhidgetMotorControlHandle phid;

// motor controller state publisher
ros::Publisher motor_pub;
ros::Publisher encoder_pub;

float acceleration = 20;
bool invert_motor = false;
ros::Time last_velocity_command;
bool motors_active = false;
bool initialised = false;
int encoder_seq = 0;

int AttachHandler(CPhidgetHandle phid, void *userptr)
{
  int serial_number;
  const char *name;

  CPhidget_getDeviceName (phid, &name);
  CPhidget_getSerialNumber(phid, &serial_number);
  ROS_INFO("%s Serial number %d attached!", name, serial_number);

  return 0;
}

int DetachHandler(CPhidgetHandle phid, void *userptr)
{
  int serial_number;
  const char *name;

  CPhidget_getDeviceName (phid, &name);
  CPhidget_getSerialNumber(phid, &serial_number);
  ROS_INFO("%s Serial number %d detached!", name, serial_number);

  return 0;
}

int ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *Description)
{
  ROS_ERROR("Error handled. %d - %s", ErrorCode, Description);
  return 0;
}

int InputChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State)
{
  phidgets::motor_params m;
  m.index = Index;
  m.value_type = 1;
  m.value = (float)State;
  if (initialised) motor_pub.publish(m);
  // ROS_INFO("Motor input %d Inputs %d", Index, State);
  return 0;
}

int VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
{
  phidgets::motor_params m;
  m.index = Index;
  m.value_type = 2;
  m.value = (float)Value;
  if (initialised) motor_pub.publish(m);
  // ROS_INFO("Motor %d Velocity %.2f", Index, (float)Value);
  return 0;
}

int CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
{
  phidgets::motor_params m;
  m.index = Index;
  m.value_type = 3;
  m.value = (float)Value;
  if (initialised) motor_pub.publish(m);
  // ROS_INFO("Motor %d Current %.2f", Index, (float)Value);
  return 0;
}

int EncoderPositionHandler(CPhidgetMotorControlHandle phid, void *usrptr, int index, int position_change)
{
  if(initialised)
  {
    phidgets::motor_encoder encoder_msg;
    encoder_msg.header.stamp = ros::Time::now();
    encoder_msg.header.seq = encoder_seq++;

    CPhidgetMotorControl_getEncoderPosition(phid, index, &(encoder_msg.count));
    encoder_msg.count_change = position_change;

    encoder_pub.publish(encoder_msg);
    // ROS_INFO("Encoder: %d", position_change);
  }
}

int display_properties(CPhidgetMotorControlHandle phid)
{
  int serial_number, version, num_motors, num_inputs;
  const char* ptr;

  CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
  CPhidget_getSerialNumber((CPhidgetHandle)phid, &serial_number);
  CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

  CPhidgetMotorControl_getInputCount(phid, &num_inputs);
  CPhidgetMotorControl_getMotorCount(phid, &num_motors);

  ROS_INFO("%s", ptr);
  ROS_INFO("Serial Number: %d", serial_number);
  ROS_INFO("Version: %d", version);
  ROS_INFO("Number of motors %d", num_motors);
  ROS_INFO("Number of inputs %d", num_inputs);

  return 0;
}

bool attach(CPhidgetMotorControlHandle &phid, int serial_number)
{
  // create the object
  CPhidgetMotorControl_create(&phid);

  // Set the handlers to be run when the device is
  // plugged in or opened from software, unplugged
  // or closed from software, or generates an error.
  CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid, AttachHandler, NULL);
  CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid, DetachHandler, NULL);
  CPhidget_set_OnError_Handler((CPhidgetHandle)phid, ErrorHandler, NULL);

  // Registers a callback that will run if an input changes.
  // Requires the handle for the Phidget, the function
  // that will be called, and a arbitrary pointer that
  // will be supplied to the callback function (may be NULL).
  CPhidgetMotorControl_set_OnInputChange_Handler (phid, InputChangeHandler, NULL);

  // Registers a callback that will run if a motor changes.
  // Requires the handle for the Phidget, the function
  // that will be called, and a arbitrary pointer that
  // will be supplied to the callback function (may be NULL).
  CPhidgetMotorControl_set_OnVelocityChange_Handler (phid, VelocityChangeHandler, NULL);

  // Registers a callback that will run if the current
  // draw changes.
  // Requires the handle for the Phidget, the function
  // that will be called, and a arbitrary pointer that
  // will be supplied to the callback function (may be NULL).
  CPhidgetMotorControl_set_OnCurrentChange_Handler (phid, CurrentChangeHandler, NULL);

  // Register the most recent encoder position
  CPhidgetMotorControl_set_OnEncoderPositionUpdate_Handler (phid, EncoderPositionHandler, NULL);

  //open the device for connections
  CPhidget_open((CPhidgetHandle)phid, serial_number);

  // get the program to wait for an motor control
  // device to be attached
  if (serial_number == -1) {
    ROS_INFO("Waiting for Motor Control HC Phidget " \
    "to be attached....");
  }
  else {
    ROS_INFO("Waiting for Motor Control HC Phidget " \
    "%d to be attached....", serial_number);
  }
  int result;
  if((result = CPhidget_waitForAttachment((CPhidgetHandle)phid, 10000)))
  {
    const char *err;
    CPhidget_getErrorDescription(result, &err);
    ROS_ERROR("Problem waiting for motor " \
    "attachment: %s", err);
    return false;
  }
  else return true;
}

/*!
* \brief disconnect the motor controller
*/
void disconnect(CPhidgetMotorControlHandle &phid)
{
  ROS_INFO("Closing...");
  CPhidget_close((CPhidgetHandle)phid);
  CPhidget_delete((CPhidgetHandle)phid);
}

/*!
* \brief callback when a velocity command is received
* \param ptr encoder parameters
*/
void velocityCommandCallback(const std_msgs::Float32::ConstPtr& ptr)
{
  if (initialised) {
    std_msgs::Float32 m = *ptr;
    float motor_speed = m.data;

    if(invert_motor) motor_speed = -motor_speed;

    ros::Time current_time = ros::Time::now();
    CPhidgetMotorControl_setVelocity (phid, 0, motor_speed);
    CPhidgetMotorControl_setAcceleration (phid, 0, acceleration);

    last_velocity_command = current_time;
    motors_active = true;
  }
}

void stop_motors()
{
  CPhidgetMotorControl_setVelocity (phid, 0, 0);
  motors_active = false;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "phidgets_motor_control_hc");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  int serial_number = -1;
  nh.getParam("serial", serial_number);
  std::string name = "motorcontrol";
  nh.getParam("name", name);
  nh.getParam("invert_motor", invert_motor);
  if (serial_number==-1) {
    nh.getParam("serial_number", serial_number);
  }

  std::string topic_path = "phidgets/";
  nh.getParam("topic_path", topic_path);
  float timeout_sec = 0.5;
  nh.getParam("timeout", timeout_sec);
  int frequency = 30;
  nh.getParam("frequency", frequency);


  if (attach(phid, serial_number)) {
    display_properties(phid);

    const int buffer_length = 100;
    std::string topic_name = topic_path + name;
    std::string service_name = name;
    if (serial_number > -1) {
      char ser[10];
      sprintf(ser, "%d", serial_number);
      topic_name += "/";
      topic_name += ser;
      service_name += "/";
      service_name += ser;
    }
    motor_pub =
    n.advertise<phidgets::motor_params>(topic_name, buffer_length);

    std::string cmd_vel_top = name + "/" + "cmd_vel";
    std::string encoder_topic = name + "/" + "encoder";
    encoder_pub = n.advertise<phidgets::motor_encoder>(encoder_topic, buffer_length);


    // receive velocity commands
    ros::Subscriber command_velocity_sub =
    n.subscribe(cmd_vel_top.data(), 1, velocityCommandCallback);


    initialised = true;
    ros::Rate loop_rate(frequency);
    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();

      // SAFETY FEATURE
      // if a velocity command has not been received
      // for a period of time then stop the motors
      double time_since_last_command_sec = (ros::Time::now() -
      last_velocity_command).toSec();

      if ((motors_active) &&
      (time_since_last_command_sec > timeout_sec)) {
        stop_motors();
        ROS_WARN("No velocity command received - " \
        "motors stopped");
      }
    }

    disconnect(phid);
  }
  return 0;
}
