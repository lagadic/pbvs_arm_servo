#include <ros/ros.h>
#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp_naoqi/vpNaoqiConfig.h>
#include <sensor_msgs/JointState.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>
#include "geometry_msgs/PoseStamped.h"
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>

#include <vpServoArm.h>



class pbvs_arm_servo
{
public:

  pbvs_arm_servo(ros::NodeHandle &nh);
  ~pbvs_arm_servo();
  void publish();
  void spin();
  void getActualPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void getDesiredPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void displayImage(const sensor_msgs::Image::ConstPtr& image);
  void initDisplayVisp();
  void setupCameraParameters(const sensor_msgs::CameraInfoConstPtr &cam_rgb);
  void getRobotJoints();


protected:

  // Robot
  vpNaoqiRobot * romeo;
  int port;
  std::string ip;
  std::vector <std::string> jointBodyNames;
  // ROS
  ros::NodeHandle n;
  ros::Time veltime;
  std::string actualPoseTopicName;  // default to /cmd_vel
  std::string desiredPoseTopicName;
  std::string imageTopicName;
  std::string cameraRGBTopicName;
  std::string cmdVelTopicName;
  int freq;
  ros::Subscriber actualPoseSub;
  ros::Subscriber desiredPoseSub;
  ros::Subscriber imageSub;
  ros::Subscriber cam_rgb_info_sub;
  ros::Publisher cmdVelPub;
  // Messages
  sensor_msgs::JointState jointStateMsg;
  std::vector<float> posState;
  std::vector<float> velState;

  //Servo Arm
  vpServoArm m_servo_arm;

  double servo_time_init;

  vpHomogeneousMatrix m_actualPose;
  vpHomogeneousMatrix m_desiredPose;
  vpHomogeneousMatrix oMe_Arm;
  vpMatrix JacobienTeeeeest;

  vpImage<vpRGBa> m_img_;
  vpDisplay* m_disp;
  vpCameraParameters m_cam_rgb;


  //conditions
  bool m_actualPose_computed;
  bool m_desiredPose_computed;
  bool m_disp_is_initialized;
  bool m_cam_is_initialized;
  bool opt_right_arm;

};
