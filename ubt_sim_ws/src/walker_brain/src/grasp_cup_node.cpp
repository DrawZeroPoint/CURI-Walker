#include <walker_brain/bt_service_node.h>
#include <walker_brain/bt_action_node.h>
#include <walker_brain/bt_generic_types.h>

// ROS
#include <ros/ros.h>

// Services and actions (customized)
#include <hope/ExtractObjectOnTop.h>

// dummy for test
#include <walker_brain/Dummy.h>
#include <walker_brain/DummyActionAction.h>


using namespace BT;

/**
 * Service node for extracting the poses of cups on top of the table
 */
class SenseCupPoses : public RosServiceNode<hope::ExtractObjectOnTop>
{
public:
  SenseCupPoses(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<hope::ExtractObjectOnTop>(nh, name, cfg) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("goal_id"),
      BT::OutputPort<int>("result_status")
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput("goal_id", request.goal_id.id);
    ros::spinOnce();  // synchronize clock time
    request.header.stamp.sec = this->time_sec_;
    ROS_INFO("Brain: SenseCupPoses sending request.");
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("Brain: SenseCupPoses response SUCCEEDED.");
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_INFO("Brain: SenseCupPoses response FAILURE.");
      return BT::NodeStatus::FAILURE;
    }
  }

  virtual BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("Brain: SenseCupPoses request failed %d.", static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }
};

class ExecuteMovement : public RosServiceNode<walker_brain::Dummy>
{
public:
  ExecuteMovement(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<walker_brain::Dummy>(nh, name, cfg) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("planning_group"),
      BT::OutputPort<int>("result_status") };
  }

  void onSendRequest(RequestType &request) override {
    if (!getInput("planning_group", request.header.frame_id)) return;  // TODO
    ros::spinOnce();
    request.header.stamp.sec = this->time_sec_;
    ROS_INFO("Brain: ExecuteMovement sending request.");
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("Brain: ExecuteMovement response SUCCEEDED.");
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_INFO("Brain: ExecuteMovement response FAILURE.");
      return BT::NodeStatus::FAILURE;
    }
  }

  virtual BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("Brain: ExecuteMovement request failed %d.", static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }
};

class ExecuteNavigation : public RosActionNode<walker_brain::DummyActionAction>
{
public:
  ExecuteNavigation(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & cfg):
    RosActionNode<walker_brain::DummyActionAction>(handle, name, cfg) {}

  static PortsList providedPorts() {
    return {
      InputPort<std::string>("goal_id"),
      OutputPort<int>("result_status")};
  }

  bool onSendGoal(GoalType& goal) override {
    // TODO
    if(!getInput<std::string>("goal_id", goal.header.frame_id)) return false;
    ros::spinOnce();
    ROS_INFO("Brain: ExecuteNavigation sending request");
    return true;
  }

  NodeStatus onResult(const ResultType& res) override {
    ROS_INFO("Brain: ExecuteNavigation result received");
    if (res.result_status == res.SUCCEEDED) {
      setOutput<int>("result_status", res.SUCCEEDED);
      ROS_INFO("Brain: ExecuteNavigation response SUCCEEDED.");
      return NodeStatus::SUCCESS;
    }
    else{
      ROS_INFO("Brain: ExecuteNavigation response FAILURE.");
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override {
    ROS_ERROR("Brain: ExecuteNavigation request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override {
    if(status() == NodeStatus::RUNNING) {
      ROS_WARN("Brain: ExecuteNavigation halted");
      BaseClass::halt();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_cup_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string tree_file;
  pnh.getParam("tree_file", tree_file);
  if (tree_file.empty()) {
    ROS_ERROR("Brain: No valid tree file.");
    return -1;
  }

  int task_id = 0;
  pnh.getParam("task_id", task_id);
  if (!task_id) {
    ROS_ERROR("Brain: 'task_id' is not given.");
    return -1;
  }

  int cup_id = 0;
  pnh.getParam("cup_id", cup_id);
  if (!cup_id) {
    ROS_ERROR("Brain: 'cup_id' is not given.");
    return -1;
  }

  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;

  // The recommended way to create a Node is through inheritance.
  RegisterRosAction<ExecuteNavigation>(factory, "ExecuteNavigation", nh);
  RegisterRosService<SenseCupPoses>(factory, "SenseCupPoses", nh);
  RegisterRosService<ExecuteMovement>(factory, "ExecuteMovement", nh);

  auto tree = factory.createTreeFromFile(tree_file);

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  while(ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
    ros::spinOnce();
    status = tree.tickRoot();
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  return 0;
}