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

#include <walker_movement/DualArmEeMoveAction.h>
#include <walker_movement/DualArmJointMoveAction.h>

std::shared_ptr<actionlib::SimpleActionServer<walker_movement::DualArmEeMoveAction>> dualArmEeMoveActionServer;
std::shared_ptr<actionlib::SimpleActionServer<walker_movement::DualArmJointMoveAction>> dualArmJointMoveActionServer;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInt_left;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInt_right;
std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::string leftEeLink = "left_tcp";
std::string rightEeLink = "right_tcp";
std::string leftPlanningGroupName = "walker_left_arm";
std::string rightPlanningGroupName = "walker_right_arm";
std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> rightControllerActionClient;
std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> leftControllerActionClient;


std::vector<control_msgs::JointTolerance> buildTolerance(double position, double velocity, double acceleration, int jointNum)
{
  control_msgs::JointTolerance gjt;
  gjt.position = position;
  gjt.velocity = velocity;
  gjt.acceleration = acceleration;
  return std::vector<control_msgs::JointTolerance>(jointNum,gjt);
}

control_msgs::FollowJointTrajectoryGoal buildControllerGoal(const moveit_msgs::RobotTrajectory& trajectory)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.goal_time_tolerance = ros::Duration(1);
  goal.goal_tolerance = buildTolerance(0.01,0,0,7);
  goal.path_tolerance = buildTolerance(0.02,0,0,7);
  goal.trajectory = trajectory.joint_trajectory;
  return goal;
}

void dualArmEeMoveActionAbort(std::string error_message)
{
  ROS_WARN_STREAM(error_message);
  walker_movement::DualArmEeMoveResult result;
  result.succeded = false;
  result.error_message = error_message;
  dualArmEeMoveActionServer->setAborted(result);
}

void dualArmEeMoveActionCallback(const walker_movement::DualArmEeMoveGoalConstPtr &goal)
{

  // Transform the two poses to base_link

  std::string referenceFrame = "base_link";
  geometry_msgs::PoseStamped leftPose_baseLink;
  geometry_msgs::PoseStamped rightPose_baseLink;
  try
  {
    leftPose_baseLink  = tfBuffer->transform(goal->left_pose,  referenceFrame, ros::Duration(1));
    rightPose_baseLink = tfBuffer->transform(goal->right_pose, referenceFrame, ros::Duration(1));
  }
  catch (tf2::TransformException &ex)
  {
    dualArmEeMoveActionAbort("Dual arm move failed: failed to transform target ee pose to base_link: "+std::string(ex.what()));
    return;
  }

  // Plan trajectories for both arms


  moveit_msgs::RobotTrajectory trajectory_left;
  moveit_msgs::RobotTrajectory trajectory_right;

  moveGroupInt_left->setPoseReferenceFrame(referenceFrame);
  moveGroupInt_right->setPoseReferenceFrame(referenceFrame);
  moveGroupInt_left->setEndEffectorLink(goal->left_end_effector_link==""? leftEeLink : goal->left_end_effector_link);
  moveGroupInt_right->setEndEffectorLink(goal->right_end_effector_link==""? rightEeLink : goal->right_end_effector_link);
  if(!goal->do_cartesian)
  {
    moveGroupInt_left->clearPoseTargets();
    moveGroupInt_left->setPoseTarget(leftPose_baseLink);
    moveit::planning_interface::MoveGroupInterface::Plan planLeft;
    auto r = moveGroupInt_left->plan(planLeft);
    if(r != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      dualArmEeMoveActionAbort("Dual arm move failed: planning for left arm failed with MoveItErrorCode "+std::to_string(r.val));
      return;
    }
    trajectory_left = planLeft.trajectory_;

    moveGroupInt_right->clearPoseTargets();
    moveGroupInt_right->setPoseTarget(rightPose_baseLink);
    moveit::planning_interface::MoveGroupInterface::Plan planRight;
    r = moveGroupInt_right->plan(planRight);
    if(r != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      dualArmEeMoveActionAbort("Dual arm move failed: planning for right arm failed with MoveItErrorCode "+std::to_string(r.val));
      return;
    }
    trajectory_right = planRight.trajectory_;
  }
  else
  {
    const double jump_threshold = 0.0; // No joint-space jump contraint (see moveit_msgs/GetCartesianPath)
    const double eef_step = 0.01;

    std::vector<geometry_msgs::Pose> waypoints_left;
    waypoints_left.push_back(leftPose_baseLink.pose);
    double fraction = moveGroupInt_left->computeCartesianPath(waypoints_left, eef_step, jump_threshold, trajectory_left);
    if(fraction != 1)
    {
      dualArmEeMoveActionAbort("Dual arm move failed: planning cartesian path for left arm failed with fraction "+std::to_string(fraction));
      return;
    }

    std::vector<geometry_msgs::Pose> waypoints_right;
    waypoints_right.push_back(rightPose_baseLink.pose);
    fraction = moveGroupInt_right->computeCartesianPath(waypoints_right, eef_step, jump_threshold, trajectory_right);
    if(fraction != 1)
    {
      dualArmEeMoveActionAbort("Dual arm move failed: planning cartesian path for right arm failed with fraction "+std::to_string(fraction));
      return;
    }
  }


  //Execute both trajecctories simultaneously

  ROS_INFO_STREAM("Executing on both arms...");

  control_msgs::FollowJointTrajectoryGoal leftGoal = buildControllerGoal(trajectory_left);
  control_msgs::FollowJointTrajectoryGoal rightGoal = buildControllerGoal(trajectory_right);

  leftControllerActionClient->sendGoal(leftGoal);
  rightControllerActionClient->sendGoal(rightGoal);
  bool completed = leftControllerActionClient->waitForResult(ros::Duration(120.0));
  if(!completed)
  {
    dualArmEeMoveActionAbort("Dual arm move failed: left execution timed out");
    return;
  }
  completed = rightControllerActionClient->waitForResult(ros::Duration(120.0));
  if(!completed)
  {
    dualArmEeMoveActionAbort("Dual arm move failed: right execution timed out");
    return;
  }

  ROS_INFO_STREAM("Dual arm movement completed.");

  walker_movement::DualArmEeMoveResult result;
  result.succeded = true;
  result.error_message = "No error";
  dualArmEeMoveActionServer->setSucceeded(result);
}



void dualArmJointMoveActionAbort(std::string error_message)
{
  ROS_WARN_STREAM(error_message);
  walker_movement::DualArmJointMoveResult result;
  result.succeded = false;
  result.error_message = error_message;
  dualArmJointMoveActionServer->setAborted(result);
}

void dualArmJointMoveActionCallback(const walker_movement::DualArmJointMoveGoalConstPtr &goal)
{
  if(moveGroupInt_left->getVariableCount()!=goal->left_pose.size())
  {
    dualArmJointMoveActionAbort("Provided left joint pose has wrong size. Should be "+std::to_string(moveGroupInt_left->getVariableCount())+" it's "+std::to_string(goal->left_pose.size()));
    return;
  }
  moveGroupInt_left->setJointValueTarget(goal->left_pose);

  if(moveGroupInt_right->getVariableCount()!=goal->right_pose.size())
  {
    dualArmJointMoveActionAbort("Provided right joint pose has wrong size. Should be "+std::to_string(moveGroupInt_right->getVariableCount())+" it's "+std::to_string(goal->right_pose.size()));
    return;
  }
  moveGroupInt_right->setJointValueTarget(goal->right_pose);




  moveit::planning_interface::MoveGroupInterface::Plan planLeft;
  auto r = moveGroupInt_left->plan(planLeft);
  if(r != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    dualArmJointMoveActionAbort("Dual arm move failed: planning for left arm failed with MoveItErrorCode "+std::to_string(r.val));
    return;
  }


  moveit::planning_interface::MoveGroupInterface::Plan planRight;
  r = moveGroupInt_right->plan(planRight);
  if(r != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    dualArmJointMoveActionAbort("Dual arm move failed: planning for right arm failed with MoveItErrorCode "+std::to_string(r.val));
    return;
  }


  //Execute both trajecctories simultaneously

  ROS_INFO_STREAM("Executing on both arms...");

  control_msgs::FollowJointTrajectoryGoal leftGoal = buildControllerGoal(planLeft.trajectory_);
  control_msgs::FollowJointTrajectoryGoal rightGoal = buildControllerGoal(planRight.trajectory_);

  leftControllerActionClient->sendGoal(leftGoal);
  rightControllerActionClient->sendGoal(rightGoal);
  bool completed = leftControllerActionClient->waitForResult(ros::Duration(120.0));
  if(!completed)
  {
    dualArmJointMoveActionAbort("Dual arm joint move failed: left execution timed out");
    return;
  }
  completed = rightControllerActionClient->waitForResult(ros::Duration(120.0));
  if(!completed)
  {
    dualArmJointMoveActionAbort("Dual arm joint move failed: right execution timed out");
    return;
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

  dualArmEeMoveActionServer->start();
  dualArmJointMoveActionServer->start();
  ROS_INFO("Action server started");

  ros::waitForShutdown();
  return 0;
}
