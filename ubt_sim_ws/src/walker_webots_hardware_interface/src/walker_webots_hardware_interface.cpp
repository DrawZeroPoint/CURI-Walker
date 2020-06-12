#include <sstream>
#include "walker_webots_hardware_interface.h"
#include <ubt_core_msgs/JointCommand.h>


using namespace hardware_interface;

namespace walker_webots_hardware_interface
{
    WalkerWebotsHardwareInterface::WalkerWebotsHardwareInterface(ros::NodeHandle& nh) : nh_(nh)
    {
        init();
        ROS_INFO("Init completed");
        controller_manager_ = std::make_shared<controller_manager::ControllerManager>(this, nh_);
        double loop_hz_;
        nh_.param("/walker/hardware_interface/loop_hz", loop_hz_, 100.0);//default 100Hz
        non_realtime_loop_ = nh_.createTimer(ros::Duration(1.0/loop_hz_), &WalkerWebotsHardwareInterface::update, this);
        ROS_INFO("Loop started");
    }

    WalkerWebotsHardwareInterface::~WalkerWebotsHardwareInterface()
    {

    }

    void WalkerWebotsHardwareInterface::init()
    {
      headStateSub =      nh_.subscribe("/walker/head/joint_states", 1, &WalkerWebotsHardwareInterface::headStateCallback, this);
      leftHandStateSub =  nh_.subscribe("/walker/leftHand/joint_states", 1, &WalkerWebotsHardwareInterface::leftHandStateCallback, this);
      leftLimbStateSub =  nh_.subscribe("/walker/leftLimb/joint_states", 1, &WalkerWebotsHardwareInterface::leftLimbStateCallback, this);
      rightHandStateSub = nh_.subscribe("/walker/rightHand/joint_states", 1, &WalkerWebotsHardwareInterface::rightHandStateCallback, this);
      rightLimbStateSub = nh_.subscribe("/walker/rightLimb/joint_states", 1, &WalkerWebotsHardwareInterface::rightLimbStateCallback, this);
      legsStateSub =      nh_.subscribe("/walker/Leg/MeasuredJoint", 1, &WalkerWebotsHardwareInterface::legsStateCallback, this);
      ROS_INFO_STREAM("Subscribed to joint states");

      leftLimbCommandPublisher = nh_.advertise<ubt_core_msgs::JointCommand>("/walker/leftLimb/controller", 1);
      rightLimbCommandPublisher = nh_.advertise<ubt_core_msgs::JointCommand>("/walker/rightLimb/controller", 1);
      headCommandPublisher = nh_.advertise<ubt_core_msgs::JointCommand>("/walker/head/controller", 1);
      ROS_INFO_STREAM("Advertised joint controls");


      // Get joint names
      nh_.getParam("/walker/hardware_interface/joints", jointNames);

      // Resize vectors
      currentJointPosition.resize(jointNames.size());
      currentJointVelocity.resize(jointNames.size());
      currentJointEffort.resize(jointNames.size());
      jointPositionCommand.resize(jointNames.size());

      std::fill(currentJointPosition.begin(), currentJointPosition.end(), 0);
      std::fill(currentJointVelocity.begin(), currentJointVelocity.end(), 0);
      std::fill(currentJointEffort.begin(),   currentJointEffort.end(), 0);
      std::fill(jointPositionCommand.begin(), jointPositionCommand.end(), 0);
      ROS_INFO_STREAM("Initialized buffers");


      // Initialize Controller
      for (int i = 0; i < jointNames.size(); ++i)
      {
           // Create joint state interface
          JointStateHandle jointStateHandle(jointNames[i], &currentJointPosition[i], &currentJointVelocity[i], &currentJointEffort[i]);
          joint_state_interface_.registerHandle(jointStateHandle);

          // Create position joint interface
          JointHandle jointPositionHandle(jointStateHandle, &jointPositionCommand[i]);
          position_joint_interface_.registerHandle(jointPositionHandle);
      }
      ROS_INFO_STREAM("Initialized interfaces");

      registerInterface(&joint_state_interface_);
      registerInterface(&position_joint_interface_);
    }

    void WalkerWebotsHardwareInterface::update(const ros::TimerEvent& e)
    {
      ///ROS_INFO("Looping");
      ros::Duration elapsed_time_ = ros::Duration(e.current_real - e.last_real);
      read();
      controller_manager_->update(ros::Time::now(), elapsed_time_);
      write(elapsed_time_);
    }

    void WalkerWebotsHardwareInterface::read()
    {
      for (int i = 0; i < jointNames.size(); i++)
      {
        try
        {
          //ROS_INFO_STREAM("Getting state of joint "<<jointNames[i]);
          std::unique_lock<std::timed_mutex> lock(jointStatesMutex, std::chrono::seconds(10));
          if(lock)
          {
            std::vector<double> jointState = jointStates.at(jointNames[i]);
            currentJointPosition.at(i) = jointState[0];
            currentJointVelocity.at(i) = jointState[1];
            currentJointEffort.at(i)   = jointState[2];
          }
          else
          {
            ROS_ERROR("Couldn't acquire mutex, couldn't get joint state");
          }
          //ROS_INFO_STREAM("Got state of joint "<<jointNames[i]);
        }
        catch(std::out_of_range& e)
        {
          ROS_WARN_STREAM("Failed to get joint "<<jointNames[i]<<" state, no message received yet");
        }
      }
    }

    void WalkerWebotsHardwareInterface::write(ros::Duration elapsed_time)
    {
      sendLeftLimbPositionCommand(getJointCommandedValue("left_limb_j1"),
                                  getJointCommandedValue("left_limb_j2"),
                                  getJointCommandedValue("left_limb_j3"),
                                  getJointCommandedValue("left_limb_j4"),
                                  getJointCommandedValue("left_limb_j5"),
                                  getJointCommandedValue("left_limb_j6"),
                                  getJointCommandedValue("left_limb_j7"));
      sendRightLimbPositionCommand(getJointCommandedValue("right_limb_j1"),
                                   getJointCommandedValue("right_limb_j2"),
                                   getJointCommandedValue("right_limb_j3"),
                                   getJointCommandedValue("right_limb_j4"),
                                   getJointCommandedValue("right_limb_j5"),
                                   getJointCommandedValue("right_limb_j6"),
                                   getJointCommandedValue("right_limb_j7"));
     sendHeadPositionCommand(getJointCommandedValue("head_j1"),
                             getJointCommandedValue("head_j2"));
    }

    double WalkerWebotsHardwareInterface::getJointCommandedValue(std::string jointName)
    {
      for (int i = 0; i < jointNames.size(); i++)
      {
        if(jointNames[i] == jointName)
          return jointPositionCommand[i];
      }
    }

    void WalkerWebotsHardwareInterface::setJointsStates(const sensor_msgs::JointState::ConstPtr& msg, const std::vector<std::string>& jointNames)
    {
      /*
      std::string js = "";
      for(std::string j : jointNames)
        js+=j;
      ROS_INFO_STREAM("Setting state of joints "<<js);
      */
      assert((msg->position.size()!=jointNames.size() || msg->velocity.size()!=jointNames.size() || msg->effort.size()!=jointNames.size(),"Mismatch in the number of joints"));

      std::unique_lock<std::timed_mutex> lock(jointStatesMutex, std::chrono::seconds(10));
      if(!lock)
      {
        ROS_ERROR("Couldn't acquire mutex, couldn't set joint state");
        return;
      }
      for(int i=0;i<jointNames.size();i++)
        jointStates[jointNames[i]] = std::vector<double>({msg->position.at(i),msg->velocity.at(i),msg->effort.at(i)});
    }

    void WalkerWebotsHardwareInterface::headStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      setJointsStates(msg,std::vector<std::string>({"head_j1",    // HeadYaw
                                                    "head_j2"})); // HeadPitch
    }

    void WalkerWebotsHardwareInterface::leftHandStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      setJointsStates(msg,std::vector<std::string>({"left_thumb_j1",      // LFirstFinger1
                                                    "left_thumb_j2",      // LFirstFinger2
                                                    "left_index_j1",      // LSecondFinger1
                                                    "left_index_j2",      // LSecondFinger2
                                                    "left_middle_j1",     // LThirdFinger1
                                                    "left_middle_j2",     // LThirdFinger2
                                                    "left_ring_j1",       // LForthFinger1
                                                    "left_ring_j2",       // LForthFinger2
                                                    "left_pinky_j1",      // LFifthFinger1
                                                    "left_pinky_j2"}));   // LFifthFinger2
    }

    void WalkerWebotsHardwareInterface::leftLimbStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      setJointsStates(msg,std::vector<std::string>({"left_limb_j1",    // LShoulderPitch
                                                    "left_limb_j2",    // LShoulderRoll
                                                    "left_limb_j3",    // LShoulderYaw
                                                    "left_limb_j4",    // LElbowRoll
                                                    "left_limb_j5",    // LElbowYaw
                                                    "left_limb_j6",    // LWristPitch
                                                    "left_limb_j7"})); // LWristRoll
    }

    void WalkerWebotsHardwareInterface::rightHandStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      setJointsStates(msg,std::vector<std::string>({"right_thumb_j1",    // RFirstFinger1
                                                    "right_thumb_j2",    // RFirstFinger2
                                                    "right_index_j1",    // RSecondFinger1
                                                    "right_index_j2",    // RSecondFinger2
                                                    "right_middle_j1",   // RThirdFinger1
                                                    "right_middle_j2",   // RThirdFinger2
                                                    "right_ring_j1",     // RForthFinger1
                                                    "right_ring_j2",     // RForthFinger2
                                                    "right_pinky_j1",    // RFifthFinger1
                                                    "right_pinky_j2"})); // RFifthFinger2
    }

    void WalkerWebotsHardwareInterface::rightLimbStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      setJointsStates(msg,std::vector<std::string>({"right_limb_j1",    // RShoulderPitch
                                                    "right_limb_j2",    // RShoulderRoll
                                                    "right_limb_j3",    // RShoulderYaw
                                                    "right_limb_j4",    // RElbowRoll
                                                    "right_limb_j5",    // RElbowYaw
                                                    "right_limb_j6",    // RWristPitch
                                                    "right_limb_j7"})); // RWristRoll
    }

    void WalkerWebotsHardwareInterface::legsStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      setJointsStates(msg,std::vector<std::string>({"left_leg_j1",    // LHipYaw
                                                    "left_leg_j2",    // LHipRoll
                                                    "left_leg_j3",    // LHipPitch
                                                    "left_leg_j4",    // LKneePitch
                                                    "left_leg_j5",    // LAnklePitch
                                                    "left_leg_j6",    // LAnkleRoll
                                                    "right_leg_j1",   // RHipYaw
                                                    "right_leg_j2",   // RHipRoll
                                                    "right_leg_j3",   // RHipPitch
                                                    "right_leg_j4",   // RKneePitch
                                                    "right_leg_j5",   // RAnklePitch
                                                    "right_leg_j6"}));// RAnkleRoll
    }


    void WalkerWebotsHardwareInterface::sendLeftLimbPositionCommand(double shoulderPitch, double shoulderRoll, double shoulderYaw, double elbowRoll, double elbowYaw, double wristPitch, double wristRoll)
    {
      ubt_core_msgs::JointCommand msg;
      msg.mode = 5;
      msg.command = std::vector<double>({shoulderPitch, shoulderRoll, shoulderYaw, elbowRoll, elbowYaw, wristPitch, wristRoll});
      leftLimbCommandPublisher.publish(msg);
    }


    void WalkerWebotsHardwareInterface::sendRightLimbPositionCommand(double shoulderPitch, double shoulderRoll, double shoulderYaw, double elbowRoll, double elbowYaw, double wristPitch, double wristRoll)
    {
      ubt_core_msgs::JointCommand msg;
      msg.mode = 5;
      msg.command = std::vector<double>({shoulderPitch, shoulderRoll, shoulderYaw, elbowRoll, elbowYaw, wristPitch, wristRoll});
      rightLimbCommandPublisher.publish(msg);
    }


    void WalkerWebotsHardwareInterface::sendHeadPositionCommand(double pitch, double yaw)
    {
      ubt_core_msgs::JointCommand msg;
      msg.mode = 5;
      msg.command = std::vector<double>({pitch, yaw});
      headCommandPublisher.publish(msg);
    }
}
