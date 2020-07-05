
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

void executeDualTrajectoryDirect(const trajectory_msgs::JointTrajectory& trajectory_left,
  const trajectory_msgs::JointTrajectory& trajectory_right,
  std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> controllerClientLeft,
  std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> controllerClientRight)
{
  control_msgs::FollowJointTrajectoryGoal leftGoal = buildControllerGoal(trajectory_left);
  control_msgs::FollowJointTrajectoryGoal rightGoal = buildControllerGoal(trajectory_right);

  controllerClientLeft->sendGoal(leftGoal);
  controllerClientRight->sendGoal(rightGoal);
  bool completed = controllerClientLeft->waitForResult(ros::Duration(120.0));
  if(!completed)
    throw std::runtime_error("Dual trajectory execution failed: left execution timed out");
  completed = controllerClientRight->waitForResult(ros::Duration(120.0));
  if(!completed)
    throw std::runtime_error("Dual trajectory execution failed: right execution timed out");
}


trajectory_msgs::JointTrajectory buildTrajectory(const std::vector<geometry_msgs::PoseStamped>& poses, const std::vector<ros::Duration>& times_from_start, std::string eeLink, std::shared_ptr<tf2_ros::Buffer> tfBuffer, std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInt, std::string planning_group_name)
{
  if(poses.size()!=times_from_start.size())
    throw std::invalid_argument("followEePoseTrajectory: poses and times_from_start should have the same size (they are respectively "+std::to_string(poses.size())+" and "+std::to_string(times_from_start.size())+")");
  std::vector<geometry_msgs::Pose> posesBaseLink;
  for(const geometry_msgs::PoseStamped& p : poses)
    posesBaseLink.push_back(tfBuffer->transform(p, "base_link", ros::Duration(1)).pose);


  std::vector<std::vector<double>> jointPoses;
  moveit::core::RobotStatePtr robotState = moveGroupInt->getCurrentState();
  const moveit::core::JointModelGroup* jointModelGroup = moveGroupInt->getCurrentState()->getJointModelGroup(planning_group_name);

  std::chrono::steady_clock::time_point timePreIK = std::chrono::steady_clock::now();
  for(unsigned int i=0;i<posesBaseLink.size();i++)
  {
    const geometry_msgs::Pose& p = posesBaseLink[i];
    std::vector<double> jointPose;
    bool foundIk = robotState->setFromIK(jointModelGroup,p,eeLink,1.0); //TODO: choose sensible parameters (Attempts and solver timeout)
    if(!foundIk)
      throw std::runtime_error("followEePoseTrajectory failed, IK solution not found for pose "+std::to_string(i)+
                    " ( requested pose is ("+
                    std::to_string(p.position.x)+", "+std::to_string(p.position.y)+", "+std::to_string(p.position.z)+") ("+
                    std::to_string(p.orientation.x)+", "+std::to_string(p.orientation.y)+", "+std::to_string(p.orientation.z)+", "+std::to_string(p.orientation.w)+") in frame base_link, transformed from "+poses.at(i).header.frame_id+")");
    robotState->copyJointGroupPositions(jointModelGroup,jointPose);
    jointPoses.push_back(jointPose);
  }
  auto timePostIK = std::chrono::steady_clock::now();
  std::chrono::duration<double> duration = timePostIK-timePreIK;
  ROS_INFO_STREAM("followEePoseTrajectory: IK took "<<duration.count()*1000<<"ms");

  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = jointModelGroup->getActiveJointModelNames();
  for(unsigned int i=0;i<jointPoses.size();i++)
  {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = jointPoses.at(i);
    point.time_from_start = times_from_start.at(i);
    trajectory.points.push_back(point);
  }

  return trajectory;
}
