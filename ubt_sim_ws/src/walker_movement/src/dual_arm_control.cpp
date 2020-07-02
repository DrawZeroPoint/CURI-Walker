#include <stdexcept>
#include <regex>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <actionlib/server/simple_action_server.h>
#include <tf/tf.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <walker_movement/DualArmEeMoveAction.h>
#include <walker_movement/DualArmJointMoveAction.h>
#include <walker_movement/DualArmMirroredEeMoveAction.h>

#include "control_utils.hpp"

std::shared_ptr<actionlib::SimpleActionServer<walker_movement::DualArmEeMoveAction>> dualArmEeMoveActionServer;
std::shared_ptr<actionlib::SimpleActionServer<walker_movement::DualArmJointMoveAction>> dualArmJointMoveActionServer;
std::shared_ptr<actionlib::SimpleActionServer<walker_movement::DualArmMirroredEeMoveAction>> dualArmMirroredEeMoveActionServer;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInt_left;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInt_right;
std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::string defaultLeftEeLink = "left_tcp";
std::string defaultRightEeLink = "right_tcp";
std::string leftPlanningGroupName = "walker_left_arm";
std::string rightPlanningGroupName = "walker_right_arm";
std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> rightControllerActionClient;
std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> leftControllerActionClient;





void planEeMoveTraj(moveit::planning_interface::MoveGroupInterface& moveGroupInt, geometry_msgs::PoseStamped pose, std::string eeLink, bool do_cartesian, moveit_msgs::RobotTrajectory& ret)
{
  //ROS_WARN_STREAM("reference frame is "<<pose.header.frame_id);
  moveGroupInt.setPoseReferenceFrame(pose.header.frame_id);
  moveGroupInt.setEndEffectorLink(eeLink);
  if(!do_cartesian)
  {
    moveGroupInt.clearPoseTargets();
    moveGroupInt.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto r = moveGroupInt.plan(plan);
    if(r != moveit::planning_interface::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("planning on group "+moveGroupInt.getName()+" for end effector "+eeLink+" failed with MoveItErrorCode "+std::to_string(r.val));
    ret = plan.trajectory_;
  }
  else
  {
    const double jump_threshold = 0.0; // No joint-space jump contraint (see moveit_msgs/GetCartesianPath)
    const double eef_step = 0.01;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(pose.pose);
    double fraction = moveGroupInt.computeCartesianPath(waypoints, eef_step, jump_threshold, ret);
    if(fraction != 1)
      throw std::runtime_error("planning on group "+moveGroupInt.getName()+" for end effector "+eeLink+" failed with fraction "+std::to_string(fraction));
  }
}

std::vector<double> multiplyElementWise(std::vector<double> a, std::vector<double> b)
{
  if(a.size()!=b.size())
    throw std::runtime_error("Incompatible vectors, different number of elements "+std::to_string(a.size())+" vs "+std::to_string(b.size()));
  std::vector<double> ret;
  for(unsigned int i=0;i<a.size();i++)
    ret.push_back(a[i]*b[i]);
  return ret;
}

trajectory_msgs::JointTrajectory mirrorTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  std::vector<double> transformation;
  std::vector<std::string> possibleNames({"left_limb_j1","left_limb_j2","left_limb_j3","left_limb_j4","left_limb_j5","left_limb_j6","left_limb_j7"});
  std::vector<std::string> rightNames({"right_limb_j1","right_limb_j2","right_limb_j3","right_limb_j4","right_limb_j5","right_limb_j6","right_limb_j7"});
  std::vector<double>               flip({-1.0,          1.0,          -1.0,           1.0,          -1.0,          -1.0,           1.0});

  ROS_WARN_STREAM("(trajectory.joint_names.size() = "<<trajectory.joint_names.size());
  ROS_WARN_STREAM("(trajectory.points.size() = "<<trajectory.points.size());
  ROS_WARN_STREAM("(trajectory.points[0].positions.size() = "<<trajectory.points[0].positions.size());
  if(trajectory.joint_names.size()!=possibleNames.size())
    throw std::runtime_error("Incompatible trajectory, different number of joints "+std::to_string(trajectory.joint_names.size())+" instead of "+std::to_string(possibleNames.size()));


  trajectory_msgs::JointTrajectory result;
  for(std::string jointName : trajectory.joint_names)
  {
    for(unsigned int i=0;i<possibleNames.size();i++)
    {
      if(jointName==possibleNames[i])
      {
        transformation.push_back(flip[i]);
        result.joint_names.push_back(rightNames[i]);
      }
    }
  }
  //at this point transformation tells if each joint has to be flipped or not

  result.header = trajectory.header;
  for(const trajectory_msgs::JointTrajectoryPoint& point : trajectory.points)
  {
    trajectory_msgs::JointTrajectoryPoint transformedPoint;
    transformedPoint.time_from_start = point.time_from_start;
    if(point.positions.size()!=0)
      transformedPoint.positions       = multiplyElementWise(point.positions,transformation);
    if(point.velocities.size()!=0)
      transformedPoint.velocities    = multiplyElementWise(point.velocities,transformation);
    if(point.accelerations.size()!=0)
      transformedPoint.accelerations = multiplyElementWise(point.accelerations,transformation);
    if(point.effort.size()!=0)
      transformedPoint.effort        = multiplyElementWise(point.effort,transformation);
    result.points.push_back(transformedPoint);
  }
  return result;
}


void executeDualTraj(const trajectory_msgs::JointTrajectory& trajectory_left, const trajectory_msgs::JointTrajectory& trajectory_right)
{
  control_msgs::FollowJointTrajectoryGoal leftGoal = buildControllerGoal(trajectory_left);
  control_msgs::FollowJointTrajectoryGoal rightGoal = buildControllerGoal(trajectory_right);

  leftControllerActionClient->sendGoal(leftGoal);
  rightControllerActionClient->sendGoal(rightGoal);
  bool completed = leftControllerActionClient->waitForResult(ros::Duration(120.0));
  if(!completed)
    throw std::runtime_error("Dual arm move failed: left execution timed out");
  completed = rightControllerActionClient->waitForResult(ros::Duration(120.0));
  if(!completed)
    throw std::runtime_error("Dual arm move failed: right execution timed out");
}



void dualArmMirroredEeMove(const walker_movement::DualArmMirroredEeMoveGoalConstPtr &goal)
{
  // Transform the two poses to base_link
  std::string referenceFrame = "base_link";
  geometry_msgs::PoseStamped leftPose_baseLink;

  leftPose_baseLink  = tfBuffer->transform(goal->left_pose,  referenceFrame, ros::Duration(1));

  // Plan trajectories for both arms
  moveit_msgs::RobotTrajectory trajectory_left;
  planEeMoveTraj( *moveGroupInt_left,
                  leftPose_baseLink,
                  goal->left_end_effector_link==""? defaultLeftEeLink : goal->left_end_effector_link,
                  goal->do_cartesian,
                  trajectory_left);
  trajectory_msgs::JointTrajectory trajectory_right = mirrorTrajectory(trajectory_left.joint_trajectory);

  //Execute both trajecctories simultaneously
  ROS_INFO_STREAM("Executing on both arms...");
  executeDualTraj(trajectory_left.joint_trajectory, trajectory_right);
}


void dualArmMirroredEeMoveActionCallback(const walker_movement::DualArmMirroredEeMoveGoalConstPtr &goal)
{
  try
  {
    dualArmMirroredEeMove(goal);
  }
  catch(std::runtime_error& e)
  {
    std::string error_message = "Dual arm mirrored end effector move failed: "+std::string(e.what());
    ROS_WARN_STREAM(error_message);
    walker_movement::DualArmMirroredEeMoveResult result;
    result.succeded = false;
    result.error_message = error_message;
    dualArmMirroredEeMoveActionServer->setAborted(result);
    return;
  }

  ROS_INFO_STREAM("Dual arm end effector move completed.");
  walker_movement::DualArmMirroredEeMoveResult result;
  result.succeded = true;
  result.error_message = "No error";
  dualArmMirroredEeMoveActionServer->setSucceeded(result);
}


void dualArmEeMove(const walker_movement::DualArmEeMoveGoalConstPtr &goal)
{
  // Transform the two poses to base_link
  std::string referenceFrame = "base_link";
  geometry_msgs::PoseStamped leftPose_baseLink;
  geometry_msgs::PoseStamped rightPose_baseLink;

  leftPose_baseLink  = tfBuffer->transform(goal->left_pose,  referenceFrame, ros::Duration(1));
  rightPose_baseLink = tfBuffer->transform(goal->right_pose, referenceFrame, ros::Duration(1));

  // Plan trajectories for both arms
  moveit_msgs::RobotTrajectory trajectory_left;
  planEeMoveTraj( *moveGroupInt_left,
                  leftPose_baseLink,
                  goal->left_end_effector_link==""? defaultLeftEeLink : goal->left_end_effector_link,
                  goal->do_cartesian,
                  trajectory_left);
  moveit_msgs::RobotTrajectory trajectory_right;
  planEeMoveTraj( *moveGroupInt_right,
                  rightPose_baseLink,
                  goal->right_end_effector_link==""? defaultRightEeLink : goal->right_end_effector_link,
                  goal->do_cartesian,
                  trajectory_right);

  //Execute both trajecctories simultaneously
  ROS_INFO_STREAM("Executing on both arms...");
  executeDualTraj(trajectory_left.joint_trajectory, trajectory_right.joint_trajectory);
}

void dualArmEeMoveActionCallback(const walker_movement::DualArmEeMoveGoalConstPtr &goal)
{
  try
  {
    dualArmEeMove(goal);
  }
  catch(std::runtime_error& e)
  {
    std::string error_message = "Dual arm end effector move failed: "+std::string(e.what());
    ROS_WARN_STREAM(error_message);
    walker_movement::DualArmEeMoveResult result;
    result.succeded = false;
    result.error_message = error_message;
    dualArmEeMoveActionServer->setAborted(result);
    return;
  }

  ROS_INFO_STREAM("Dual arm end effector move completed.");
  walker_movement::DualArmEeMoveResult result;
  result.succeded = true;
  result.error_message = "No error";
  dualArmEeMoveActionServer->setSucceeded(result);
}






void dualArmJointMove(const walker_movement::DualArmJointMoveGoalConstPtr &goal)
{
  if(moveGroupInt_left->getVariableCount()!=goal->left_pose.size())
    throw std::invalid_argument("Provided left joint pose has wrong size. Should be "+std::to_string(moveGroupInt_left->getVariableCount())+" it's "+std::to_string(goal->left_pose.size()));
  if(moveGroupInt_right->getVariableCount()!=goal->right_pose.size())
    throw std::invalid_argument("Provided right joint pose has wrong size. Should be "+std::to_string(moveGroupInt_right->getVariableCount())+" it's "+std::to_string(goal->right_pose.size()));

  //plan
  moveGroupInt_left->setJointValueTarget(goal->left_pose);
  moveGroupInt_right->setJointValueTarget(goal->right_pose);
  moveit::planning_interface::MoveGroupInterface::Plan planLeft;
  auto r = moveGroupInt_left->plan(planLeft);
  if(r != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    throw std::runtime_error("planning for right arm failed with MoveItErrorCode "+std::to_string(r.val));
  moveit::planning_interface::MoveGroupInterface::Plan planRight;
  r = moveGroupInt_right->plan(planRight);
  if(r != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    throw std::runtime_error("planning for left arm failed with MoveItErrorCode "+std::to_string(r.val));

  //Execute both trajecctories simultaneously
  ROS_INFO_STREAM("Executing on both arms...");
  executeDualTraj(planLeft.trajectory_.joint_trajectory, planRight.trajectory_.joint_trajectory);

}

void dualArmJointMoveActionCallback(const walker_movement::DualArmJointMoveGoalConstPtr &goal)
{
  try
  {
    dualArmJointMove(goal);
  }
  catch(std::runtime_error& e)
  {
    std::string error_message = "Dual Arm joint move failed: "+std::string(e.what());
    ROS_WARN_STREAM(error_message);
    walker_movement::DualArmJointMoveResult result;
    result.succeded = false;
    result.error_message = error_message;
    dualArmJointMoveActionServer->setAborted(result);
  }

  ROS_INFO_STREAM("Dual arm movement completed.");
  walker_movement::DualArmJointMoveResult result;
  result.succeded = true;
  result.error_message = "No error";
  dualArmJointMoveActionServer->setSucceeded(result);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_arm_control");
  ros::NodeHandle node_handle("~");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ROS_INFO("Creating MoveGroupInterface...");
  moveGroupInt_left  = std::make_shared<moveit::planning_interface::MoveGroupInterface>(leftPlanningGroupName,std::shared_ptr<tf2_ros::Buffer>(),ros::WallDuration(30));
  moveGroupInt_right = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rightPlanningGroupName,std::shared_ptr<tf2_ros::Buffer>(),ros::WallDuration(30));
  ROS_INFO("MoveGroupInterface created.");


  rightControllerActionClient = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>("walker_right_arm_controller/follow_joint_trajectory", true);
  leftControllerActionClient = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>("walker_left_arm_controller/follow_joint_trajectory", true);

  ROS_INFO("Waiting for controller action servers...");
  rightControllerActionClient->waitForServer();
  leftControllerActionClient->waitForServer();
  ROS_INFO("Controllers connected.");


  tfBuffer = std::make_shared<tf2_ros::Buffer>();
  tf2_ros::TransformListener tfListener(*tfBuffer);

  dualArmEeMoveActionServer = std::make_shared<actionlib::SimpleActionServer<walker_movement::DualArmEeMoveAction>>(node_handle,
                                                                                                                "move_to_ee_pose",
                                                                                                                dualArmEeMoveActionCallback,
                                                                                                                false);

  dualArmJointMoveActionServer = std::make_shared<actionlib::SimpleActionServer<walker_movement::DualArmJointMoveAction>>(node_handle,
                                                                                                                "move_to_joint_pose",
                                                                                                                dualArmJointMoveActionCallback,
                                                                                                                false);


  dualArmMirroredEeMoveActionServer = std::make_shared<actionlib::SimpleActionServer<walker_movement::DualArmMirroredEeMoveAction>>(node_handle,
                                                                                                                            "move_to_ee_pose_mirrored",
                                                                                                                            dualArmMirroredEeMoveActionCallback,
                                                                                                                            false);
  dualArmEeMoveActionServer->start();
  dualArmJointMoveActionServer->start();
  dualArmMirroredEeMoveActionServer->start();
  ROS_INFO("Action server started");

  ros::waitForShutdown();
  return 0;
}
