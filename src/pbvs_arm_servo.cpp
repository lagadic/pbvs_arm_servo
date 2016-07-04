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

    m_cMh_isInitialized = false;
    m_cMdh_isInitialized = false;

    m_statusPoseHand = 0;
    m_statusPoseDesired = 0;
    m_servo_time_init = 0;
    m_computed.data = false;
    m_activation = 0;
    m_init = false;

    n.param( "frequency", freq, 100);
    n.param<std::string>("actualPoseTopicName", actualPoseTopicName, "/visp_blobs_tracker/object_position");
    n.param<std::string>("desiredPoseTopicName", desiredPoseTopicName, "/visp_blobs_tracker/object_des_position");
    n.param<std::string>("cmdVelTopicName", cmdVelTopicName, "joint_state");
    n.param<std::string>("statusPoseHandTopicName", statusPoseHandTopicName, "/visp_blobs_tracker/status");
    n.param<std::string>("activationTopicName", activationTopicName, "/demo_romeo_door/pbvs_active");
    n.param<std::string>("armToControl", m_opt_arm, "right");
    n.param<std::string>("offsetFileName", m_offsetFileName, "/udd/bheintz/data_romeo/pose.xml");
    n.param<std::string>("offsetName", m_offsetName, "HandleOffset");
    n.param<std::string>("cameraFrame", m_cameraFrameName, "SR300_rgb_optical_frame");
    n.param( "statusPoseDesired_isEnable", m_statusPoseDesired_isEnable, false );
    n.param("savePose", m_savePose, false);
    n.param("useOffset", m_useOffset, false);

    // Initialize subscriber and publisher
    if ( m_statusPoseDesired_isEnable )
    {
        n.param<std::string>("StatusPoseDesiredTopicName", statusPoseDesiredTopicName, "/visp_blobs_tracker/status2");
        statusPoseDesiredSub = n.subscribe ( statusPoseDesiredTopicName, 1, (boost::function < void(const std_msgs::Int8::ConstPtr  &)>) boost::bind( &pbvs_arm_servo::getStatusPoseDesiredCb, this, _1 ));
    }
    else
        m_statusPoseDesired = 1;

    desiredPoseSub = n.subscribe( desiredPoseTopicName, 1, (boost::function < void(const geometry_msgs::PoseStampedConstPtr&)>) boost::bind( &pbvs_arm_servo::getDesiredPoseCb, this, _1 ));
    actualPoseSub = n.subscribe( actualPoseTopicName, 1, (boost::function < void(const geometry_msgs::PoseStampedConstPtr &)>) boost::bind( &pbvs_arm_servo::getActualPoseCb, this, _1 ));
    statusPoseHandSub = n.subscribe ( statusPoseHandTopicName, 1, (boost::function < void(const std_msgs::Int8::ConstPtr  &)>) boost::bind( &pbvs_arm_servo::getStatusPoseHandCb, this, _1 ));
    activationSub = n.subscribe ( activationTopicName, 1, (boost::function < void(const std_msgs::Int8::ConstPtr  &)>) boost::bind( &pbvs_arm_servo::getActivationCb, this, _1 ));
    cmdVelPub = n.advertise<sensor_msgs::JointState >(cmdVelTopicName, 10);
    pbvsComputedPub = n.advertise< std_msgs::Bool >("pbvs_computed", 1);
//    poseWithOffsetPub = n.advertise< std_msgs::Bool >("pbvs_computed", 1);
    if ( !m_savePose )
    {
        if (m_opt_arm == "right")
            m_chain_name = "RArm";
        else
            m_chain_name = "LArm";

        std::string filename_transform = std::string(ROMEOTK_DATA_FOLDER) + "/transformation.xml";
        std::string name_transform = "qrcode_M_e_" + m_chain_name;
        vpXmlParserHomogeneousMatrix pm; // Create a XML parser

        if (pm.parse(oMe_Arm, filename_transform, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
            std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
            ros::shutdown();
        }
        else
            std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << oMe_Arm << std::endl;

        ROS_INFO("Launch NaoqiRobotros node");
        robot.open();

        m_jointNames_arm =  robot.getBodyNames(m_chain_name);
        m_jointNames_arm.pop_back(); // Delete last joints LHand, that we don't consider in the servo

        m_numJoints = m_jointNames_arm.size();
        m_q.resize(m_numJoints);
        m_q_dot.resize(m_numJoints);
        m_q2_dot.resize(m_numJoints);
        m_q_dot_msg.velocity.resize(m_numJoints);
        m_q_dot_msg.name = m_jointNames_arm;
        m_jointMin.resize(m_numJoints);
        m_jointMax.resize(m_numJoints);

        if (m_useOffset)
        {
           if( pm.parse(m_dhMoffset, m_offsetFileName, m_offsetName) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
                std::cout << "Cannot found the homogeneous matrix named " << m_offsetName << "." << std::endl;
                ros::shutdown();
            }
            else
                std::cout << "Homogeneous matrix " << m_offsetName <<": " << std::endl << m_dhMoffset << std::endl;
        }

        //Get joint limits
        robot.getJointMinAndMax(m_jointNames_arm, m_jointMin, m_jointMax);

        //Set the stiffness
        robot.setStiffness(m_jointNames_arm, 1.f);
        vpXmlParserHomogeneousMatrix xml;
        vpHomogeneousMatrix c3dMc2d;
        vpXmlParserHomogeneousMatrix p;
        if (pm.parse(c3dMc2d, "/udd/bheintz/data_romeo/Calib_romeo/forehead/pose_c2Mc1.xml", "Pose c2Mc1") != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
            std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
            ros::shutdown();
        }
        else
            std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << oMe_Arm << std::endl;

//        std::string outputFileNametMrf;
//        outputFileNametMrf = "/udd/bheintz/data_romeo/TorsoRightForehead.xml";
//        std::string outputFileNametMhr;
//        outputFileNametMhr = "/udd/bheintz/data_romeo/TorsoHeadRoll.xml";

//        vpHomogeneousMatrix torsoLeftForehead(robot.getProxy()->getTransform("CameraLeft", 0, true));
//        vpHomogeneousMatrix torsoHeadRoll(robot.getProxy()->getTransform("HeadRoll", 0, true));
//        vpHomogeneousMatrix camera3dMheadRoll;
//        camera3dMheadRoll = c3dMc2d * torsoLeftForehead.inverse() * torsoHeadRoll;
//        if( xml.save(camera3dMheadRoll, "/udd/bheintz/data_romeo/camera3dMheadRoll.xml", "Pose") == vpXmlParserHomogeneousMatrix::SEQUENCE_OK )
//            std::cout << "Pose between the hand and the object successfully saved in \"" << m_offsetFileName << "\"" << std::endl;
//        else {
//            std::cout << "Failed to save the pose in \"" << m_offsetFileName << "\"" << std::endl;
//            std::cout << "A file with the same name exists. Remove it to be able to save the parameters..." << std::endl;
//        }
//        if( xml.save(torsoHeadRoll, "/udd/bheintz/data_romeo/TorsoHeadRoll.xml", "Pose") == vpXmlParserHomogeneousMatrix::SEQUENCE_OK )
//            std::cout << "Pose between the hand and the object successfully saved in \"" << m_offsetFileName << "\"" << std::endl;
//        else {
//            std::cout << "Failed to save the pose in \"" << m_offsetFileName << "\"" << std::endl;
//            std::cout << "A file with the same name exists. Remove it to be able to save the parameters..." << std::endl;
//        }

    }

}

pbvs_arm_servo::~pbvs_arm_servo(){

}


void pbvs_arm_servo::spin()
{
    ros::Rate loop_rate(freq);
    while(ros::ok()){
        if( m_savePose )
            this->savePoses();
        else
            this->computeControlLaw();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void pbvs_arm_servo::computeControlLaw()
{
    vpHomogeneousMatrix currentFeature;
    vpHomogeneousMatrix cMhandle_des;
    vpRotationMatrix cRh;
    vpTranslationVector cTh;
    vpHomogeneousMatrix cMh;
    geometry_msgs::Pose cMh_msg;
    tf::Transform transformdh;
    static tf::TransformBroadcaster br;

    std::cout << m_cMh_isInitialized << "  " << m_cMdh_isInitialized  << "  " <<  m_statusPoseHand << "  " <<  m_statusPoseDesired << "  " <<  m_activation << std::endl;
    if ( m_cMh_isInitialized && m_cMdh_isInitialized  && m_statusPoseHand && m_statusPoseDesired && m_activation == 1)
    {
        static bool first_time = true;
        if (first_time) {
            std::cout << "-- Start visual servoing of the arm" << std::endl;
            m_servo_time_init = vpTime::measureTimeSecond();
            first_time = false;
        }
        vpAdaptiveGain lambda(1, 0.05, 8);
        m_servo_arm.setLambda(lambda);
        m_servo_arm.set_eJe(robot.get_eJe(m_chain_name));
        if ( m_useOffset )
        {
            currentFeature = m_dhMoffset.inverse() * m_cMdh.inverse() * m_cMh;
            cMhandle_des = m_cMdh * m_dhMoffset;
        }
        else
            currentFeature = m_cMdh.inverse() * m_cMh;

        ////Publish the TF BEGIN////
        transformdh.setOrigin( tf::Vector3(cMhandle_des[0][3], cMhandle_des[1][3], cMhandle_des[2][3] ));
        cTh = cMhandle_des.getTranslationVector();
        cRh = cMhandle_des.getRotationMatrix();
        cMh = vpHomogeneousMatrix(cTh, cRh);
        cMh_msg = visp_bridge::toGeometryMsgsPose(cMh);

        tf::Quaternion qdh;
        qdh.setX(cMh_msg.orientation.x);
        qdh.setY(cMh_msg.orientation.y);
        qdh.setZ(cMh_msg.orientation.z);
        qdh.setW(cMh_msg.orientation.w);

        transformdh.setRotation(qdh);
        br.sendTransform(tf::StampedTransform(transformdh, ros::Time::now(), m_cameraFrameName, "desired_pose_tf"));
        ////Publish the TF END////
        m_servo_arm.setCurrentFeature(currentFeature) ;
        // Create twist matrix from target Frame to Arm end-effector (WristPitch)
        vpVelocityTwistMatrix oVe_LArm(oMe_Arm);
        m_servo_arm.m_task.set_cVe(oVe_LArm);

        //Compute velocities PBVS task
        m_q_dot = - m_servo_arm.computeControlLaw(vpTime::measureTimeSecond() - m_servo_time_init);

        m_q = robot.getPosition(m_jointNames_arm);
        m_q2_dot  = m_servo_arm.m_task.secondaryTaskJointLimitAvoidance(m_q, m_q_dot, m_jointMin, m_jointMax);

        publishCmdVel(m_q_dot + m_q2_dot);

        vpTranslationVector t_error_grasp = currentFeature.getTranslationVector();
        vpRotationMatrix R_error_grasp;
        currentFeature.extract(R_error_grasp);
        vpThetaUVector tu_error_grasp;
        tu_error_grasp.buildFrom(R_error_grasp);
        double theta_error_grasp;
        vpColVector u_error_grasp;
        tu_error_grasp.extract(theta_error_grasp, u_error_grasp);
        double error_t_treshold = 0.001;

        m_init = false;

        if ( (sqrt(t_error_grasp.sumSquare()) < error_t_treshold) && (theta_error_grasp < vpMath::rad(3)) )
        {
          m_computed.data = true;
          pbvsComputedPub.publish(m_computed);
        }
        std::cout << "We are servoing the arm " << m_activation << std::endl;
    }
    else if (!m_init && m_activation == 0)
    {
      m_init = true;
      vpColVector q_dot_zero(m_numJoints,0);
      publishCmdVel(q_dot_zero);
      std::cout << "publishing just once" << std::endl;
    }
    else if ( m_activation == 1 &&( m_statusPoseHand == 0 || m_statusPoseDesired == 0) )
    {
      vpColVector q_dot_zero(m_numJoints,0);
      publishCmdVel(q_dot_zero);
    }
    else if (m_activation == 2)
    {
      vpColVector q_dot_zero(m_numJoints,0);
      publishCmdVel(q_dot_zero);
      std::cout << "Shutting down the node" << std::endl;
      ros::shutdown();
    }

}

void pbvs_arm_servo::publishCmdVel(const vpColVector &q)
{
    for (int i = 0; i < q.size(); i++)
    {
        m_q_dot_msg.velocity[i] = q[i];
    }

    cmdVelPub.publish(m_q_dot_msg);

}


void pbvs_arm_servo::getDesiredPoseCb(const geometry_msgs::PoseStamped::ConstPtr &desiredPose)
{
    m_cMdh = visp_bridge::toVispHomogeneousMatrix(desiredPose->pose);

    if ( !m_cMdh_isInitialized )
    {
        ROS_INFO("DesiredPose received");
        m_cMdh_isInitialized = true;
    }

}


void pbvs_arm_servo::getActualPoseCb(const geometry_msgs::PoseStamped::ConstPtr &actualPose)
{
    m_cMh = visp_bridge::toVispHomogeneousMatrix(actualPose->pose);
    if ( !m_cMh_isInitialized )
    {
        ROS_INFO("ActualPose received");
        m_cMh_isInitialized = true;
    }
}


void pbvs_arm_servo::getStatusPoseHandCb(const std_msgs::Int8::ConstPtr  &status)
{
    m_statusPoseHand = status->data;
}


void pbvs_arm_servo::getStatusPoseDesiredCb(const std_msgs::Int8::ConstPtr  &status)
{
    m_statusPoseDesired = status->data;
}

void pbvs_arm_servo::getActivationCb(const std_msgs::Int8::ConstPtr  &status)
{
    m_activation = status->data;
}


void pbvs_arm_servo::savePoses()
{
    vpHomogeneousMatrix dhMoffset;
    dhMoffset = m_cMdh.inverse() * m_cMh;
    vpXmlParserHomogeneousMatrix xml;

    if ( m_cMh_isInitialized && m_cMdh_isInitialized  && m_statusPoseHand && m_statusPoseDesired)
    {
        ROS_INFO_STREAM("cMdh = " << m_cMdh << "\n cMh = " << m_cMh << "\n dhMh = " << dhMoffset);
        if( xml.save(dhMoffset, m_offsetFileName.c_str(), m_offsetName) == vpXmlParserHomogeneousMatrix::SEQUENCE_OK )
            std::cout << "Pose between the hand and the object successfully saved in \"" << m_offsetFileName << "\"" << std::endl;
        else {
            std::cout << "Failed to save the pose in \"" << m_offsetFileName << "\"" << std::endl;
            std::cout << "A file with the same name exists. Remove it to be able to save the parameters..." << std::endl;
        }
        ros::shutdown();
    }
}

