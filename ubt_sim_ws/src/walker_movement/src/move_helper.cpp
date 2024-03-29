#include <stdexcept>
#include <chrono>

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

#include <walker_movement/MoveToEePoseAction.h>
#include <walker_movement/MoveToJointPoseAction.h>
#include <walker_movement/FollowEePoseTrajectoryAction.h>
#include <walker_movement/GetJointState.h>
#include <walker_movement/GetEePose.h>

#include "control_utils.hpp"

std::shared_ptr<actionlib::SimpleActionServer<walker_movement::MoveToEePoseAction>> moveToEePoseActionServer;
std::shared_ptr<actionlib::SimpleActionServer<walker_movement::MoveToJointPoseAction>> moveToJointPoseActionServer;
std::shared_ptr<actionlib::SimpleActionServer<walker_movement::FollowEePoseTrajectoryAction>> followEePoseTrajectoryActionServer;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInt;
std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::string defaultEeLink = "";
const robot_state::JointModelGroup* joint_model_group;
std::string planning_group_name;
std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> controllerActionClient;


int waitActionCompletion(moveit::planning_interface::MoveGroupInterface& move_group)
{
  move_group.getMoveGroupClient().waitForResult(ros::Duration(0, 0));//TODO: set a sensible timeout
  if (move_group.getMoveGroupClient().getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO_STREAM(move_group.getMoveGroupClient().getState().toString() << ": " << move_group.getMoveGroupClient().getState().getText());
    ROS_ERROR_STREAM("Action execution failed with MoveItErrorCode "<<moveit::planning_interface::MoveItErrorCode(move_group.getMoveGroupClient().getResult()->error_code));
    return -1;
  }
  else
  {
    ROS_INFO_STREAM("Moved.");
  }
  return 0;
}


int submitMoveToJointPose(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double> jointPose)
{
  if(move_group.getVariableCount()!=jointPose.size())
  {
    ROS_ERROR_STREAM("Provided joint pose has wrong size. Should be "<<move_group.getVariableCount()<<" it's "<<jointPose.size());
    return -1;
  }
  move_group.setJointValueTarget(jointPose);

  ROS_INFO_STREAM("Joint pose target set.");
  auto r = move_group.asyncMove();
  if(r != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR_STREAM("Pose-based asyncMove submission failed with MoveItErrorCode "<<r);
    return -2;
  }
  ROS_INFO_STREAM("Moving...");
  return 0;
}


int submitMoveToEePose(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::PoseStamped& targetPose, std::string endEffectorLink)
{
/*  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers"));
  visual_tools_->publishXArrow (targetPose,rviz_visual_tools::colors::RED,rviz_visual_tools::scales::XLARGE);
  visual_tools_->trigger();
  ROS_INFO("displaying debug arrow");
  ros::WallDuration(5).sleep();
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();*/

  geometry_msgs::PoseStamped targetPoseBaseLink;
  try{
    targetPoseBaseLink = tfBuffer->transform(targetPose, "base_link", ros::Duration(1));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN_STREAM("Failed to transform target ee pose to base_link: "<<ex.what());
    return -1;
  }

  move_group.clearPoseTargets();
  std::string eeLink = endEffectorLink;
  if(eeLink=="")
    eeLink = defaultEeLink;

  move_group.setPoseTarget(targetPoseBaseLink,endEffectorLink);

  ROS_INFO_STREAM("Pose target set.");
  auto r = move_group.asyncMove();
  if(r != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR_STREAM("Pose-based asyncMove submission failed with MoveItErrorCode "<<r);
    return -2;
  }
  ROS_INFO_STREAM("Moving...");
  return 0;
}








int moveToJointPose(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double> jointPose)
{
  int r = submitMoveToJointPose(move_group,jointPose);
  if(r<0)
  {
    ROS_ERROR_STREAM("Failed to submit move to joint pose, error "<<r);
    return -1;
  }

  r = waitActionCompletion(move_group);
  if(r<0)
  {
    ROS_ERROR_STREAM("Failed to execute move to joint pose, error "<<r);
    return -2;
  }
  return 0;
}


int moveToEePose(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::PoseStamped& targetPose, std::string endEffectorLink)
{
  int r = submitMoveToEePose(move_group, targetPose, endEffectorLink);
  if(r<0)
  {
    ROS_ERROR_STREAM("Failed to submit move to end effectore pose, error "<<r);
    return -1;
  }

  r = waitActionCompletion(move_group);
  if(r<0)
  {
    ROS_ERROR_STREAM("Failed to execute move to end effector pose, error "<<r);
    return -2;
  }
  return 0;
}

void moveToEePoseActionCallback(const walker_movement::MoveToEePoseGoalConstPtr &goal)
{

  int r = moveToEePose(*moveGroupInt,goal->pose,goal->end_effector_link);

  if(r<0)
  {
    std::string errorMsg = "moveToEePose failed with error "+std::to_string(r);
    ROS_ERROR_STREAM(errorMsg);
    walker_movement::MoveToEePoseResult result;
    result.succeded = false;
    result.error_message = errorMsg;
    moveToEePoseActionServer->setAborted(result);
    return;
  }

  walker_movement::MoveToEePoseResult result;
  result.succeded = true;
  result.error_message = "No error";
  moveToEePoseActionServer->setSucceeded(result);

}


void moveToJointPoseActionCallback(const walker_movement::MoveToJointPoseGoalConstPtr &goal)
{

  int r = moveToJointPose(*moveGroupInt,goal->pose);

  if(r<0)
  {
    std::string errorMsg = "moveToJointPose failed with error "+std::to_string(r);
    ROS_ERROR_STREAM(errorMsg);
    walker_movement::MoveToJointPoseResult result;
    result.succeded = false;
    result.error_message = errorMsg;
    moveToJointPoseActionServer->setAborted(result);
    return;
  }

  walker_movement::MoveToJointPoseResult result;
  result.succeded = true;
  result.error_message = "No error";
  moveToJointPoseActionServer->setSucceeded(result);

}


bool getJointStateServiceCallback(walker_movement::GetJointState::Request& req, walker_movement::GetJointState::Response& res)
{
  ROS_INFO("Getting joint state...");
  moveGroupInt->getCurrentState(10)->copyJointGroupPositions(joint_model_group, res.joint_poses);
  ROS_INFO("Got joint state.");
  return true;
}

bool getEePoseServiceCallback(walker_movement::GetEePose::Request& req, walker_movement::GetEePose::Response& res)
{
  ROS_INFO("Getting end effector pose...");
  res.pose = moveGroupInt->getCurrentPose(req.end_effector_link_name);
  ROS_INFO("Got end effector pose...");
  return true;
}


void followEePoseTrajectory(const std::vector<geometry_msgs::PoseStamped>& poses, const std::vector<ros::Duration>& times_from_start, std::string eeLink)
{
  executeTrajectoryDirect(buildTrajectory(poses,
                                          times_from_start,
                                          eeLink == ""? defaultEeLink : eeLink,tfBuffer,
                                          moveGroupInt,
                                          planning_group_name),
                          controllerActionClient);
}




void followEePoseTrajectoryActionCallback(const walker_movement::FollowEePoseTrajectoryGoalConstPtr& goal)
{
  try
  {
    followEePoseTrajectory(goal->poses, goal->times_from_start, goal->end_effector_link);
  }
  catch(std::runtime_error& e)
  {
    std::string error_message = "Follow ee pose trajectory failed: "+std::string(e.what());
    ROS_WARN_STREAM(error_message);
    walker_movement::FollowEePoseTrajectoryResult result;
    result.succeded = false;
    result.error_message = error_message;
    followEePoseTrajectoryActionServer->setAborted(result);
    return;
  }

  ROS_INFO_STREAM("Follow ee pose trajectory completed.");
  walker_movement::FollowEePoseTrajectoryResult result;
  result.succeded = true;
  result.error_message = "No error";
  followEePoseTrajectoryActionServer->setSucceeded(result);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_helper");
  ros::NodeHandle node_handle("~");

  node_handle.getParam("planning_group", planning_group_name);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ROS_INFO("Creating MoveGroupInterface...");
  moveGroupInt = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group_name,std::shared_ptr<tf2_ros::Buffer>(),ros::WallDuration(30));
  ROS_INFO("MoveGroupInterface created.");
  joint_model_group = moveGroupInt->getCurrentState()->getJointModelGroup(planning_group_name);

  std::string controllerActionNAme;
  if(planning_group_name=="walker_left_arm")
  {
    defaultEeLink = "left_tcp";
    controllerActionNAme = "walker_left_arm_controller/follow_joint_trajectory";
  }
  else if(planning_group_name=="walker_right_arm")
  {
    defaultEeLink = "right_tcp";
    controllerActionNAme = "walker_right_arm_controller/follow_joint_trajectory";
  }
  else if(planning_group_name=="walker_head")
  {
    defaultEeLink = "head_l3";
    controllerActionNAme = "walker_head_controller/follow_joint_trajectory";
  }
  else if(planning_group_name=="walker_left_leg")
  {
    defaultEeLink = "left_foot_sole";
    controllerActionNAme = "walker_left_leg_controller/follow_joint_trajectory";
  }
  else if(planning_group_name=="walker_right_leg")
  {
    defaultEeLink = "right_foot_sole";
    controllerActionNAme = "walker_right_leg_controller/follow_joint_trajectory";
  }
  else
  {
    ROS_ERROR_STREAM("Invalid planning group name");
    return -1;
  }

  tfBuffer = std::make_shared<tf2_ros::Buffer>();
  tf2_ros::TransformListener tfListener(*tfBuffer);


  controllerActionClient = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(controllerActionNAme, true);

  ROS_INFO("Waiting for controller action server...");
  controllerActionClient->waitForServer();
  ROS_INFO("Controllers connected.");


  moveToEePoseActionServer = std::make_shared<actionlib::SimpleActionServer<walker_movement::MoveToEePoseAction>>(node_handle,
                                                                                                                  "move_to_ee_pose",
                                                                                                                  moveToEePoseActionCallback,
                                                                                                                  false);
  moveToEePoseActionServer->start();

  moveToJointPoseActionServer = std::make_shared<actionlib::SimpleActionServer<walker_movement::MoveToJointPoseAction>>(node_handle,
                                                                                                                        "move_to_joint_pose",
                                                                                                                        moveToJointPoseActionCallback,
                                                                                                                        false);
  moveToJointPoseActionServer->start();

  followEePoseTrajectoryActionServer = std::make_shared<actionlib::SimpleActionServer<walker_movement::FollowEePoseTrajectoryAction>>(node_handle,
                                                                                                                                "follow_ee_pose_trajectory",
                                                                                                                                followEePoseTrajectoryActionCallback,
                                                                                                                                false);
  followEePoseTrajectoryActionServer->start();

  ros::ServiceServer service = node_handle.advertiseService("get_joint_state", getJointStateServiceCallback);
  ros::ServiceServer getEePoseService = node_handle.advertiseService("get_ee_pose", getEePoseServiceCallback);

  ROS_INFO("Action and service servers started");
  ros::waitForShutdown();
  return 0;
}
