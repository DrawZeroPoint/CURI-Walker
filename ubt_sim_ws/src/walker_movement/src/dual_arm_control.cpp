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

std::shared_ptr<actionlib::SimpleActionServer<walker_movement::DualArmEeMoveAction>> dualArmEeMoveActionServer;
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

control_msgs::FollowJointTrajectoryGoal planToGoal(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.goal_time_tolerance = ros::Duration(1);
  goal.goal_tolerance = buildTolerance(0.01,0,0,7);
  goal.path_tolerance = buildTolerance(0.02,0,0,7);
  goal.trajectory = plan.trajectory_.joint_trajectory;
  return goal;
}


void dualArmEeMoveActionCallback(const walker_movement::DualArmEeMoveGoalConstPtr &goal)
{

  // Transform the two poses to base_link

  geometry_msgs::PoseStamped leftPose_baseLink;
  geometry_msgs::PoseStamped rightPose_baseLink;
  try{
    leftPose_baseLink  = tfBuffer->transform(goal->left_pose,  "base_link", ros::Duration(1));
    rightPose_baseLink = tfBuffer->transform(goal->right_pose, "base_link", ros::Duration(1));
  }
  catch (tf2::TransformException &ex) {
    std::string errorMsg = "Dual arm move failed: failed to transform target ee pose to base_link: "+std::string(ex.what());
    ROS_WARN_STREAM(errorMsg);
    walker_movement::DualArmEeMoveResult result;
    result.succeded = false;
    result.error_message = errorMsg;
    dualArmEeMoveActionServer->setAborted(result);
    return ;
  }

  // Plan trajectories for both arms

  moveit::planning_interface::MoveGroupInterface::Plan planLeft;
  moveit::planning_interface::MoveGroupInterface::Plan planRight;
  moveGroupInt_left->setPoseTarget(leftPose_baseLink,goal->left_end_effector_link);
  auto r = moveGroupInt_left->plan(planLeft);
  if(r != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    std::string errorMsg = "Dual arm move failed: planning for left arm failed with MoveItErrorCode "+std::to_string(r.val);
    ROS_WARN_STREAM(errorMsg);
    walker_movement::DualArmEeMoveResult result;
    result.succeded = false;
    result.error_message = errorMsg;
    dualArmEeMoveActionServer->setAborted(result);
    return;
  }
  moveGroupInt_right->setPoseTarget(rightPose_baseLink,goal->right_end_effector_link);
  r = moveGroupInt_right->plan(planRight);
  if(r != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    std::string errorMsg = "Dual arm move failed: planning for right arm failed with MoveItErrorCode "+std::to_string(r.val);
    ROS_WARN_STREAM(errorMsg);
    walker_movement::DualArmEeMoveResult result;
    result.succeded = false;
    result.error_message = errorMsg;
    dualArmEeMoveActionServer->setAborted(result);
    return;
  }


  //Execute both trajecctories simultaneously

  ROS_INFO_STREAM("Executing on both arms...");

  control_msgs::FollowJointTrajectoryGoal leftGoal = planToGoal(planLeft);
  control_msgs::FollowJointTrajectoryGoal rightGoal = planToGoal(planRight);

  leftControllerActionClient->sendGoal(leftGoal);
  rightControllerActionClient->sendGoal(rightGoal);
  bool completed = leftControllerActionClient->waitForResult(ros::Duration(120.0));
  if(!completed)
  {
    std::string errorMsg = "Dual arm move failed: left execution timed out ";
    ROS_WARN_STREAM(errorMsg);
    walker_movement::DualArmEeMoveResult result;
    result.succeded = false;
    result.error_message = errorMsg;
    dualArmEeMoveActionServer->setAborted(result);
    return;
  }
  completed = rightControllerActionClient->waitForResult(ros::Duration(120.0));
  if(!completed)
  {
    std::string errorMsg = "Dual arm move failed: right execution timed out ";
    ROS_WARN_STREAM(errorMsg);
    walker_movement::DualArmEeMoveResult result;
    result.succeded = false;
    result.error_message = errorMsg;
    dualArmEeMoveActionServer->setAborted(result);
    return;
  }

  ROS_INFO_STREAM("Dual arm movement completed.");

  walker_movement::DualArmEeMoveResult result;
  result.succeded = true;
  result.error_message = "No error";
  dualArmEeMoveActionServer->setSucceeded(result);
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
                                                                                                                "dual_arm_move_to_ee_pose",
                                                                                                                dualArmEeMoveActionCallback,
                                                                                                                false);
  dualArmEeMoveActionServer->start();
  ROS_INFO("Action server started");

  ros::waitForShutdown();
  return 0;
}
