#ifndef WALKER_WEBOTS_HARDWARE_INTERFACE_H_20200611
#define WALKER_WEBOTS_HARDWARE_INTERFACE_H_20200611

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <mutex>

using namespace hardware_interface;

namespace walker_webots_hardware_interface
{
    class WalkerWebotsHardwareInterface: public hardware_interface::RobotHW
    {
        public:
            WalkerWebotsHardwareInterface(ros::NodeHandle& nh);
            ~WalkerWebotsHardwareInterface();
            void init();
            void update(const ros::TimerEvent& e);
            void read();
            void write(ros::Duration elapsed_time);

        private:
          void headStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
          void leftHandStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
          void leftLimbStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
          void rightHandStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
          void rightLimbStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
          void legsStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
          void setJointsStates(const sensor_msgs::JointState::ConstPtr& msg, const std::vector<std::string>& jointNames);

          void sendLeftLimbPositionCommand(double shoulderPitch, double shoulderRoll, double shoulderYaw, double elbowRoll, double elbowYaw, double wristPitch, double wristRoll);
          void sendRightLimbPositionCommand(double shoulderPitch, double shoulderRoll, double shoulderYaw, double elbowRoll, double elbowYaw, double wristPitch, double wristRoll);
          void sendHeadPositionCommand(double pitch, double yaw);
          double getJointCommandedValue(std::string jointName);

          ros::Subscriber headStateSub;
          ros::Subscriber leftHandStateSub;
          ros::Subscriber leftLimbStateSub;
          ros::Subscriber rightHandStateSub;
          ros::Subscriber rightLimbStateSub;
          ros::Subscriber legsStateSub;

          ros::Publisher leftLimbCommandPublisher;
          ros::Publisher rightLimbCommandPublisher;
          ros::Publisher headCommandPublisher;

        protected:
            ros::NodeHandle& nh_;
            ros::Timer non_realtime_loop_;
            PositionJointInterface positionJointInterface;
            std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

            // Interfaces
            hardware_interface::JointStateInterface joint_state_interface_;
            hardware_interface::PositionJointInterface position_joint_interface_;


            std::map<std::string,std::vector<double>>  jointStates;
            std::timed_mutex jointStatesMutex;
            // Shared memory
            std::vector<std::string> jointNames;
            std::vector<double> currentJointPosition;
            std::vector<double> currentJointVelocity;
            std::vector<double> currentJointEffort;
            std::vector<double> jointPositionCommand;
    };

}

#endif
