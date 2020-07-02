#ifndef CONTROL_UTILS_HPP_20200702
#define CONTROL_UTILS_HPP_20200702

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>


std::vector<control_msgs::JointTolerance> buildTolerance(double position, double velocity, double acceleration, int jointNum);

control_msgs::FollowJointTrajectoryGoal buildControllerGoal(const trajectory_msgs::JointTrajectory& trajectory);

void executeTrajectoryDirect(const trajectory_msgs::JointTrajectory& trajectory, std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> controllerActionClient);

#endif
