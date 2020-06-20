// RoSEE
#include <walker_brain/bt_service_node.h>
#include <walker_brain/bt_action_node.h>

// ROS
#include <ros/ros.h>

// Services and actions (customized)
#include <hope/ExtractObjectOnTop.h>


using namespace BT;
/**
 * Service node for extracting the poses of cups on top of the table
 * The server is provided by hope
 */
class SenseCupPoses : public RosServiceNode<hope::ExtractObjectOnTop>
{
public:
  SenseCupPoses(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg):
    RosServiceNode<hope::ExtractObjectOnTop>(nh, name, cfg) {}

  static BT::PortsList providedPorts() {
    return  {
      BT::InputPort<std::string>("goal_id"),
      BT::OutputPort<int>("result_status") };
  }

  void onSendRequest(RequestType &request) override {
    getInput("goal_id", request.goal_id.id);
    ros::spinOnce();
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

//class FibonacciServer: public RosActionNode<behaviortree_ros::FibonacciAction>
//{
//
//public:
//  FibonacciServer( ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
//    RosActionNode<behaviortree_ros::FibonacciAction>(handle, name, conf) {}
//
//  static PortsList providedPorts()
//  {
//    return  {
//      InputPort<int>("order"),
//      OutputPort<int>("result") };
//  }
//
//  bool sendGoal(GoalType& goal) override
//  {
//    if( !getInput<int>("order", goal.order) )
//    {
//      // abourt the entire action. Result in a FAILURE
//      return false;
//    }
//    expected_result_ = 0 + 1 + 1 + 2 + 3 + 5 + 8; // supposing order is 5
//    ROS_INFO("FibonacciAction: sending request");
//    return true;
//  }
//
//  NodeStatus onResult( const ResultType& res) override
//  {
//    ROS_INFO("FibonacciAction: result received");
//    int fibonacci_result = 0;
//    for( int n: res.sequence)
//    {
//      fibonacci_result += n;
//    }
//    if( fibonacci_result == expected_result_)
//    {
//      setOutput<int>("result", fibonacci_result);
//      return NodeStatus::SUCCESS;
//    }
//    else{
//      ROS_ERROR("FibonacciAction replied something unexpected: %d", fibonacci_result);
//      return NodeStatus::FAILURE;
//    }
//  }
//
//  virtual NodeStatus onFailedRequest(FailureCause failure) override
//  {
//    ROS_ERROR("FibonacciAction request failed %d", static_cast<int>(failure));
//    return NodeStatus::FAILURE;
//  }
//
//  void halt() override
//  {
//    if( status() == NodeStatus::RUNNING )
//    {
//      ROS_WARN("FibonacciAction halted");
//      BaseClass::halt();
//    }
//  }
//
//private:
//  int expected_result_;
//};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_cup_node");
  ros::NodeHandle nh;

  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;

  // The recommended way to create a Node is through inheritance.
  RegisterRosService<SenseCupPoses>(factory, "SenseCupPoses", nh);
  // RegisterRosAction<FibonacciServer>(factory, "Fibonacci", nh);

  auto tree = factory.createTreeFromFile("/home/dzp/grasp_cup.xml");

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  while(ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = tree.tickRoot();
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  return 0;
}