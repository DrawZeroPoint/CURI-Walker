#include <walker_brain/bt_service_node.h>
#include <walker_brain/bt_action_node.h>
#include <walker_brain/bt_generic_types.h>
#include <walker_brain/rosout_logger.h>

// ROS
#include <ros/ros.h>

// Services (customized)

// Actions (customized)
#include <walker_movement/MoveToJointPoseAction.h>
#include <walker_movement/GraspAction.h>


using namespace BT;


class ExecuteFKMove : public RosActionNode<walker_movement::MoveToJointPoseAction>
{
public:
  ExecuteFKMove(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & cfg):
    RosActionNode<walker_movement::MoveToJointPoseAction>(handle, name, cfg), name_(name) {}

  static PortsList providedPorts() {
    return {
      InputPort<JointAngles>("execute_pose"),
      //OutputPort<int>("result_status")
    };
  }

  bool onSendGoal(GoalType& goal) override {
    JointAngles execute_pose{};
    getInput<JointAngles>("execute_pose", execute_pose);
    goal.pose = execute_pose.toROS();
    ROS_INFO("Brain: %s sending request", name_.c_str());
    return true;
  }

  NodeStatus onResult(const ResultType& res) override {
    ROS_INFO("Brain: %s result received", name_.c_str());
    if (res.succeded) {
      ROS_INFO("Brain: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("Brain: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override {
    ROS_ERROR("Brain: %s request failed %d", name_.c_str(), static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override {
    if(status() == NodeStatus::RUNNING) {
      ROS_WARN("Brain: %s halted", name_.c_str());
      BaseClass::halt();
    }
  }

private:
  std::string name_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "switch_light_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string tree_file;
  pnh.getParam("tree_file", tree_file);
  if (tree_file.empty()) {
    ROS_ERROR("Brain: No valid tree file.");
    return -1;
  }

  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;

  // The recommended way to create a Node is through inheritance.
  RegisterRosAction<ExecuteFKMove>(factory, "ExecutePrePose", nh);

  auto tree = factory.createTreeFromFile(tree_file);

  RosoutLogger logger(tree.rootNode());
  printTreeRecursively(tree.rootNode());

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  while(ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
    ros::spinOnce();
    status = tree.tickRoot();
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  return 0;
}