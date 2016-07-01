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

    n.param( "frequency", freq, 100);
    n.param<std::string>("actualPoseTopicName", actualPoseTopicName, "/visp_blobs_tracker/object_position");
    n.param<std::string>("desiredPoseTopicName", desiredPoseTopicName, "/visp_blobs_tracker/object_des_position");
    n.param<std::string>("cmdVelTopicName", cmdVelTopicName, "joint_state");
    n.param<std::string>("statusPoseHandTopicName", statusPoseHandTopicName, "/visp_blobs_tracker/status");
    n.param<std::string>("armToControl", m_opt_arm, "right");
    n.param<std::string>("offsetFileName", m_offsetFileName, "/udd/bheintz/data_romeo/pose.xml");
    n.param<std::string>("offsetName", m_offsetName, "HandleOffset");
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
    cmdVelPub = n.advertise<sensor_msgs::JointState >(cmdVelTopicName, 10);
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

        vpHomogeneousMatrix torsoLeftForehead(robot.getProxy()->getTransform("CameraLeft", 0, true));
        vpHomogeneousMatrix torsoHeadRoll(robot.getProxy()->getTransform("HeadRoll", 0, true));
        vpHomogeneousMatrix camera3dMheadRoll;
        camera3dMheadRoll = c3dMc2d * torsoLeftForehead.inverse() * torsoHeadRoll;
        if( xml.save(camera3dMheadRoll, "/udd/bheintz/data_romeo/camera3dMheadRoll.xml", "Pose") == vpXmlParserHomogeneousMatrix::SEQUENCE_OK )
            std::cout << "Pose between the hand and the object successfully saved in \"" << m_offsetFileName << "\"" << std::endl;
        else {
            std::cout << "Failed to save the pose in \"" << m_offsetFileName << "\"" << std::endl;
            std::cout << "A file with the same name exists. Remove it to be able to save the parameters..." << std::endl;
        }
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

    if ( m_cMh_isInitialized && m_cMdh_isInitialized  && m_statusPoseHand && m_statusPoseDesired)
    {
        static bool first_time = true;
        if (first_time) {
            std::cout << "-- Start visual servoing of the arm" << std::endl;
            m_servo_time_init = vpTime::measureTimeSecond();
            first_time = false;
        }
//        vpHomogeneousMatrix torsoRightForehead(robot.getProxy()->getTransform("CameraLeft", 0, true));
//        vpHomogeneousMatrix torsoHeadRoll(robot.getProxy()->getTransform("HeadRoll", 0, true));

//        m_cMhr = torsoRightForehead.inverse() * torsoHeadRoll;
        vpAdaptiveGain lambda(0.8, 0.05, 8);
        m_servo_arm.setLambda(lambda);
        m_servo_arm.set_eJe(robot.get_eJe(m_chain_name));
        if ( m_useOffset )
        {
            currentFeature = m_dhMoffset.inverse() * m_cMdh.inverse() * m_cMh;
        }
        else
            currentFeature = m_cMdh.inverse() * m_cMh;
        m_servo_arm.setCurrentFeature(currentFeature) ;
        // Create twist matrix from target Frame to Arm end-effector (WristPitch)
        vpVelocityTwistMatrix oVe_LArm(oMe_Arm);
        m_servo_arm.m_task.set_cVe(oVe_LArm);

        //Compute velocities PBVS task
        m_q_dot = - m_servo_arm.computeControlLaw(vpTime::measureTimeSecond() - m_servo_time_init);

        m_q = robot.getPosition(m_jointNames_arm);
        m_q2_dot  = m_servo_arm.m_task.secondaryTaskJointLimitAvoidance(m_q, m_q_dot, m_jointMin, m_jointMax);

        publishCmdVel(m_q_dot + m_q2_dot);

    }
    else
    {
        vpColVector q_dot_zero(m_numJoints,0);
        publishCmdVel(q_dot_zero);
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

