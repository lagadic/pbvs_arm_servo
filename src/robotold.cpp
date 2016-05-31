#include <iostream>
#include <vector>
#include <ros/ros.h>
#include "visp_naoqi_robot.h"

static const unsigned int fake_computation = 100000;

VispNaoqiRobot::VispNaoqiRobot()
{
  std::vector< std::string > joint_names;
  joint_names.push_back( "left_wheel_joint" );
  joint_names.push_back( "right_wheel_joint" );

  // init to make a diff
  pos[0] = 1.5f;
  pos[1] = 2.5f;

  /* not available in indigo anymore */
  //hardware_interface::JointModeHandle mode_handle("joint_mode", &joint_mode);
  //jnt_mode_interface.registerHandle(mode_handle);

  // for (size_t i = 0; i < joint_names.size(); ++i)
  // {

  // }


}

void VispNaoqiRobot::readJoints()
{
  for (unsigned int i = 0; i < fake_computation; ++i)
  {
    1+1;
  }

  ROS_INFO_STREAM("*****read*******");
  ROS_INFO_STREAM("pos0 :" << pos[0] );
  ROS_INFO_STREAM("vel0 :" << vel[0] );
  ROS_DEBUG("I just computed read joints");
}

void VispNaoqiRobot::writeJoints()
{
  for (unsigned int i = 0; i< fake_computation; ++i)
  {
    2+2;
  }
  ROS_INFO_STREAM("*****write*******");
  ROS_INFO_STREAM("cmd0: " << cmd[0] );
  ROS_DEBUG("I just computed write joints");
}
