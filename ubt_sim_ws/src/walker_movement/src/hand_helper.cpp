#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <walker_movement/GraspAction.h>

#include <ubt_core_msgs/JointCommand.h>


std::shared_ptr<actionlib::SimpleActionServer<walker_movement::GraspAction>> GraspActionServer;
ros::Publisher handCommandPublisher;

void setHandPosition( double thumb1,
                      double thumb2,
                      double index1,
                      double index2,
                      double middle1,
                      double middle2,
                      double pinky1,
                      double pinky2,
                      double ring1,
                      double ring2)
{
    ubt_core_msgs::JointCommand msg;
    msg.mode = 5;
    msg.command = std::vector<double>({ thumb1,
                                        thumb2,
                                        index1,
                                        index2,
                                        middle1,
                                        middle2,
                                        pinky1,
                                        pinky2,
                                        ring1,
                                        ring2});
    handCommandPublisher.publish(msg);
}


void graspActionCallback(const walker_movement::GraspGoalConstPtr &goal)
{
  ROS_INFO("Grasping...");
  if(goal->grasp_type == walker_movement::GraspGoal::GRASP_TYPE_CAN)
  {
    setHandPosition(0.1,0.5, 0.6,1, 0.6,1, 0.6,1, 0.6,1);
  }
  else if(goal->grasp_type == walker_movement::GraspGoal::GRASP_TYPE_OPEN)
  {
    setHandPosition(0,0, 0,0, 0,0, 0,0, 0,0);
  }
  else if(goal->grasp_type == walker_movement::GraspGoal::GRASP_TYPE_CART_HANDLE_CORNER)
  {
    setHandPosition(0.2,0.9, 0.7,0.7, 1,0.8, 0.5,0.7, 0.3,0.3);
  }
  else if(goal->grasp_type == walker_movement::GraspGoal::GRASP_TYPE_FRIDGE_HANDLE)
  {
    setHandPosition(0.3,0.5, 1.1,1.2, 1.1,1.2, 1.1,1.2, 1.1,1.2);
  }
  else
  {
    ROS_ERROR("Invalid grasp type");
  }
  ROS_INFO("Grasped");
  walker_movement::GraspResult result;
  result.succeded = true;
  result.error_message = "No error";
  GraspActionServer->setSucceeded(result);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_helper");
  ros::NodeHandle node_handle("~");

  bool isLeft;
  node_handle.getParam("is_left", isLeft);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ROS_INFO("Action server started");
  if(isLeft)
    handCommandPublisher = node_handle.advertise<ubt_core_msgs::JointCommand>("/walker/leftHand/controller", 1);
  else
    handCommandPublisher = node_handle.advertise<ubt_core_msgs::JointCommand>("/walker/rightHand/controller", 1);

  GraspActionServer = std::make_shared<actionlib::SimpleActionServer<walker_movement::GraspAction>>(node_handle,
                                                                                                                        "grasp",
                                                                                                                        graspActionCallback,
                                                                                                                        false);
  GraspActionServer->start();



  ros::waitForShutdown();
  return 0;
}
