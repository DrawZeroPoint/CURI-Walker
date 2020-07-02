
#include "control_utils.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


#include <tf/tf.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>


std::vector<control_msgs::JointTolerance> buildTolerance(double position, double velocity, double acceleration, int jointNum)
{
  control_msgs::JointTolerance gjt;
  gjt.position = position;
  gjt.velocity = velocity;
  gjt.acceleration = acceleration;
  return std::vector<control_msgs::JointTolerance>(jointNum,gjt);
}

control_msgs::FollowJointTrajectoryGoal buildControllerGoal(const trajectory_msgs::JointTrajectory& trajectory)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.goal_time_tolerance = ros::Duration(1);
  goal.goal_tolerance = buildTolerance(0.01,0,0,7);
  goal.path_tolerance = buildTolerance(0.02,0,0,7);
  goal.trajectory = trajectory;
  return goal;
}


void executeTrajectoryDirect(const trajectory_msgs::JointTrajectory& trajectory, std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> controllerActionClient)
{
  control_msgs::FollowJointTrajectoryGoal goal = buildControllerGoal(trajectory);

  controllerActionClient->sendGoal(goal);
  bool completed = controllerActionClient->waitForResult(ros::Duration(120.0));
  if(!completed)
    throw std::runtime_error("Arm move failed: execution timed out");
}
