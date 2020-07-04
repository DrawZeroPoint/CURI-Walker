#ifndef CONTROL_UTILS_HPP_20200702
#define CONTROL_UTILS_HPP_20200702

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>

std::vector<control_msgs::JointTolerance> buildTolerance(double position, double velocity, double acceleration, int jointNum);

control_msgs::FollowJointTrajectoryGoal buildControllerGoal(const trajectory_msgs::JointTrajectory& trajectory);

void executeTrajectoryDirect(const trajectory_msgs::JointTrajectory& trajectory, std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> controllerActionClient);

void executeDualTrajectoryDirect( const trajectory_msgs::JointTrajectory& trajectory_left,
                                  const trajectory_msgs::JointTrajectory& trajectory_right,
                                  std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> controllerClientLeft,
                                  std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> controllerClientRight);


trajectory_msgs::JointTrajectory buildTrajectory( const std::vector<geometry_msgs::PoseStamped>& poses,
                                                  const std::vector<ros::Duration>& times_from_start,
                                                  std::string eeLink,
                                                  std::shared_ptr<tf2_ros::Buffer> tfBuffer,
                                                  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInt,
                                                  std::string planning_group_name);
#endif
