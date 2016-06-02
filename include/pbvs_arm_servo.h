#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>

#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp_naoqi/vpNaoqiConfig.h>

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
  void getStatusPoseHandCb(const std_msgs::Int8::ConstPtr &status);
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
  std::string statusPoseHandTopicName;
  std::string cmdVelTopicName;
  int freq;
  ros::Subscriber actualPoseSub;
  ros::Subscriber desiredPoseSub;
  ros::Subscriber imageSub;
  ros::Subscriber camRgbInfoSub;
  ros::Subscriber statusPoseHandSub;
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

  vpCameraParameters m_cam_rgb;


  //conditions
  bool m_cMh_isInitialized;
  bool m_cMdh_isInitialized;
  bool m_disp_is_initialized;
  bool m_cam_is_initialized;
  bool opt_right_arm;
  int m_statusPoseHand;

};
