#include <iostream>
#include <vector>
#include <algorithm>


#include <visp_naoqi/vpNaoqiGrabber.h>
#include <vpRomeoTkConfig.h>


#include "pbvs_arm_servo.h"



pbvs_arm_servo::pbvs_arm_servo(ros::NodeHandle &nh)
{
  // read in config options
  n = nh;

  m_img_.init(480,640);

  m_disp_is_initialized = false;
  m_cam_is_initialized = false;
  m_actualPose_computed = false;
  m_desiredPose_computed = false;
  servo_time_init = 0;

  n.param( "frequency", freq,100);
  n.param<std::string>("ActualPoseTopicName", actualPoseTopicName, "/visp_blobs_tracker/object_position");
  n.param<std::string>("DesiredPoseTopicName", desiredPoseTopicName, "/visp_blobs_tracker/object_des_position");
  n.param<std::string>("ImageTopicName", imageTopicName, "/camera/rgb/image_raw");
  n.param<std::string>("CameraRBGTopicName", cameraRGBTopicName, "/camera/rgb/camera_info");
  n.param<std::string>("cmdVelTopicName", cmdVelTopicName, "joint_state");
  n.param( "opt_right_arm", opt_right_arm, false );
//  n.param<std::string>("Ip", ip, "198.18.0.1");
//  n.param("Port", port, 9559);

  // Initialize subscriber and publisher
  cam_rgb_info_sub = n.subscribe( cameraRGBTopicName, 1, (boost::function < void(const sensor_msgs::CameraInfo::ConstPtr&)>) boost::bind( &pbvs_arm_servo::setupCameraParameters, this, _1 ));
  desiredPoseSub = n.subscribe( desiredPoseTopicName, 1, (boost::function < void(const geometry_msgs::PoseStampedConstPtr&)>) boost::bind( &pbvs_arm_servo::getDesiredPose , this, _1 ));
  actualPoseSub = n.subscribe( actualPoseTopicName, 1, (boost::function < void(const geometry_msgs::PoseStampedConstPtr &)>) boost::bind( &pbvs_arm_servo::getActualPose, this, _1 ));
  imageSub = n.subscribe( imageTopicName, 1, (boost::function < void(const sensor_msgs::Image::ConstPtr &)>) boost::bind( &pbvs_arm_servo::displayImage, this, _1 ));
  cmdVelPub = n.advertise<sensor_msgs::JointState >(cmdVelTopicName, 10);

//  ROS_INFO("DEBUGGG 1 ");
  std::string chain_name;
  std::string suffix;
  if (opt_right_arm)
  {
    suffix = "_r";
    chain_name = "RArm";
  }
  else
  {
    suffix = "_l";
    chain_name = "LArm";
  }

  vpMatrix JacobienTest((unsigned int)6, (unsigned int)7);
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 7; j++)
    {
      if ( i == j)
      {
        JacobienTest[i][i] = 1;
      }
      else
      {
        JacobienTest[i][j] = -(0.1 * i + j * 0.2)/ JacobienTest[i][j-1];
      }
    }
  }
  JacobienTeeeeest.stack(JacobienTest);
  std::cout << "Jacobien matrix :" << std::endl << JacobienTeeeeest << std::endl;

  std::string filename_transform = std::string(ROMEOTK_DATA_FOLDER) + "/transformation.xml";
  std::string name_transform = "qrcode_M_e_" + chain_name;
  vpXmlParserHomogeneousMatrix pm; // Create a XML parser

  if (pm.parse(oMe_Arm, filename_transform, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
    std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
//    return 0;
  }
  else
    std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << oMe_Arm << std::endl;


  ROS_INFO("Launch NaoqiRobotros node");
//  romeo = new vpNaoqiRobot;
//  romeo->setRobotIp(ip);
//  romeo->open();

  // Initialize names msg JointState
//  jointBodyNames = romeo->getProxy()->getBodyNames("Body");
//  jointStateMsg.name = jointBodyNames;

//  std::stringstream ss;
//  std::copy(jointBodyNames.begin(), jointBodyNames.end()-1, std::ostream_iterator<std::string>(ss,","));
//  std::copy(jointBodyNames.end()-1, jointBodyNames.end(), std::ostream_iterator<std::string>(ss));
//  ROS_INFO("Romeo joints found: %s",ss.str().c_str());

}

pbvs_arm_servo::~pbvs_arm_servo(){

//  if (romeo){
//    romeo->stop(jointBodyNames);
//    delete romeo;
//    romeo = NULL;
//  }
}


void pbvs_arm_servo::spin()
{
  ros::Rate loop_rate(10);
  while(ros::ok()){
    this->publish();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void pbvs_arm_servo::publish()
{
  vpHomogeneousMatrix cMh;
  vpHomogeneousMatrix cMo;
  vpHomogeneousMatrix oMh;
  geometry_msgs::PoseStamped dMa_msg;
  vpColVector q_dot_larm;
  sensor_msgs::JointState q_dot_msg;


  if ( m_actualPose_computed && m_desiredPose_computed)
  {
    static bool first_time = true;
    if (first_time) {
      std::cout << "-- Start visual servoing of the arm" << std::endl;
      servo_time_init = vpTime::measureTimeSecond();
      first_time = false;
    }

    cMo = m_actualPose;
    cMh = m_desiredPose;

    oMh = cMo.inverse() * cMh;

    // Create twist matrix from target Frame to Arm end-effector (WristPitch)
    vpVelocityTwistMatrix oVe_LArm(oMe_Arm);

    m_servo_arm.setLambda(0.1);
    m_servo_arm.set_eJe(JacobienTeeeeest);
    m_servo_arm.m_task.set_cVe(oVe_LArm);

    m_servo_arm.setCurrentFeature(oMh) ;

    q_dot_larm = - m_servo_arm.computeControlLaw(servo_time_init);

    for (int i = 0; i < q_dot_larm.size(); i++)
    {
      q_dot_msg.velocity.push_back(q_dot_larm[i]);
    }
    vpDisplay::display(m_img_);
    vpDisplay::displayFrame(m_img_, m_desiredPose, m_cam_rgb, 0.1);
    vpDisplay::displayFrame(m_img_, m_actualPose, m_cam_rgb, 0.1);
    vpDisplay::flush(m_img_);

    ros::Time now = ros::Time::now();
    dMa_msg.header.stamp = now;
    dMa_msg.header.frame_id = "camera/rgb";
    dMa_msg.pose = visp_bridge::toGeometryMsgsPose(oMh);

    cmdVelPub.publish(q_dot_msg);

    m_actualPose_computed = false;
    m_desiredPose_computed = false;
  }

}


void pbvs_arm_servo::getDesiredPose(const geometry_msgs::PoseStamped::ConstPtr &desiredPose)
{
  if ( !m_desiredPose_computed )
  {
    m_desiredPose = visp_bridge::toVispHomogeneousMatrix(desiredPose->pose);

    ROS_INFO("DesiredPose = true");
    m_desiredPose_computed = true;
  }

//  if (msg->velocity.size() != msg->name.size()) {
//    ROS_ERROR("The vector of the joint name and of the velocity have a different size.");
//    return;
//  }

//  romeo->setVelocity(msg->name, msg->velocity);
//  ROS_INFO("Applying vel at %f s:",veltime.toSec());


}


void pbvs_arm_servo::getActualPose(const geometry_msgs::PoseStamped::ConstPtr &actualPose)
{
  //ros::Time veltime = ros::Time::now();
  if ( !m_actualPose_computed )
  {
    m_actualPose = visp_bridge::toVispHomogeneousMatrix(actualPose->pose);

    ROS_INFO("ActualPose = true");
    m_actualPose_computed = true;
  }


//  if (msg->velocity.size() != msg->name.size()) {
//    ROS_ERROR("The vector of the joint name and of the velocity have a different size.");
//    return;
//  }

//  romeo->setVelocity(msg->name, msg->velocity);
//  ROS_INFO("Applying vel at %f s:",veltime.toSec());


}

void pbvs_arm_servo::initDisplayVisp()
{
  if (! m_disp_is_initialized) {
    m_disp = new vpDisplayX(m_img_, 750, 0, "Image Viewer");
//    m_disp->init(m_img_);
//    m_disp->setTitle("Image Mono viewer");
    vpDisplay::flush(m_img_);
    vpDisplay::display(m_img_);
    ROS_INFO("Initialisation done");
    vpDisplay::flush(m_img_);

    m_disp_is_initialized = true;
  }

}

void pbvs_arm_servo::displayImage(const sensor_msgs::Image::ConstPtr& image)
{
  initDisplayVisp();

  m_img_ = visp_bridge::toVispImageRGBa(*image);

}

void pbvs_arm_servo::setupCameraParameters(const sensor_msgs::CameraInfoConstPtr &cam_rgb)
{
  if (! m_cam_is_initialized) {
    //init m_camera parameters
    m_cam_rgb = visp_bridge::toVispCameraParameters(*cam_rgb);

    m_cam_is_initialized = true;
    ROS_INFO_STREAM("cam info : " << m_cam_rgb);
  }

}

//int main( int argc, char** argv )
//{
//  ros::init( argc, argv, "pbvs_arm_servo" );

//  ros::NodeHandle n(std::string("~"));

//  pbvs_arm_servo *node = new pbvs_arm_servo(n);

//  node->spin();

//  delete node;

//  return 0;
//}


