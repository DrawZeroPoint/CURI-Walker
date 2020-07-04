#include <stdexcept>

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

std::shared_ptr<actionlib::SimpleActionServer<walker_movement::DualArmEeMoveAction>> dualArmEeMoveActionServer;
std::shared_ptr<actionlib::SimpleActionServer<walker_movement::DualArmJointMoveAction>> dualArmJointMoveActionServer;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInt_left;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInt_right;
std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::string defaultLeftEeLink = "left_tcp";
std::string defaultRightEeLink = "right_tcp";
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
    throw std::runtime_error("Incompatible vectors, different number of elements "+std::to_string(a.size())+" instead of "+std::to_string(b.size()));
  std::vector<double> ret;
  for(unsigned int i=0;i<a.size();i++)
    ret.push_back(a[i]*b[i]);
  return ret;
}

void mirrorLeftTrajectoryToRight(const trajectory_msgs::JointTrajectory& left_traj,
                                 trajectory_msgs::JointTrajectory &right_traj)
{
  std::vector<double> flip = {-1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0};

  right_traj.points.clear();
  for (const auto& point : left_traj.points) {
    trajectory_msgs::JointTrajectoryPoint transformedPoint;
    transformedPoint.time_from_start = point.time_from_start;
    transformedPoint.positions = multiplyElementWise(point.positions, flip);
    transformedPoint.velocities = multiplyElementWise(point.velocities, flip);
    transformedPoint.accelerations = multiplyElementWise(point.accelerations, flip);
    // Do not flip effort since that does not exist
    // transformedPoint.effort = multiplyElementWise(point.effort, flip);
    right_traj.points.push_back(transformedPoint);
  }
}


void executeDualTraj(const moveit_msgs::RobotTrajectory& trajectory_left, const moveit_msgs::RobotTrajectory& trajectory_right)
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
                  goal->left_end_effector_link.empty()? defaultLeftEeLink : goal->left_end_effector_link,
                  goal->do_cartesian,
                  trajectory_left);
  moveit_msgs::RobotTrajectory trajectory_right;
  planEeMoveTraj( *moveGroupInt_right,
                  rightPose_baseLink,
                  goal->right_end_effector_link.empty()? defaultRightEeLink : goal->right_end_effector_link,
                  goal->do_cartesian,
                  trajectory_right);

  //Execute both trajectories simultaneously
  ROS_INFO_STREAM("Executing on both arms...");
  executeDualTraj(trajectory_left, trajectory_right);
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

  //Execute both trajectories simultaneously
  ROS_INFO_STREAM("Executing on both arms...");

  if (goal->mirror) {
    //std::cerr << planLeft.trajectory_.joint_trajectory << std::endl;
    mirrorLeftTrajectoryToRight(planLeft.trajectory_.joint_trajectory, planRight.trajectory_.joint_trajectory);
    //std::cerr << planRight.trajectory_.joint_trajectory << std::endl;
  }

  executeDualTraj(planLeft.trajectory_, planRight.trajectory_);
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

  dualArmEeMoveActionServer->start();
  dualArmJointMoveActionServer->start();
  ROS_INFO("Action server started");

  ros::waitForShutdown();
  return 0;
}
