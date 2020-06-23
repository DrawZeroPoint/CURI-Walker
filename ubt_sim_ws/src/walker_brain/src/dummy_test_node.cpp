#include <walker_brain/bt_service_node.h>
#include <walker_brain/bt_action_node.h>
#include <walker_brain/bt_generic_types.h>
#include <walker_brain/rosout_logger.h>

// ROS
#include <ros/ros.h>

// dummy for test
#include <walker_brain/Dummy.h>
#include <walker_brain/DummyActionAction.h>


using namespace BT;


class ExecuteMovement : public RosServiceNode<walker_brain::Dummy>
{
public:
  ExecuteMovement(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<walker_brain::Dummy>(nh, name, cfg) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("goal_id"),
      BT::OutputPort<Pose2D>("nav_pose"),
      BT::OutputPort<PoseArray>("obj_poses"),
      BT::OutputPort<int>("result_status")
    };
  }

  void onSendRequest(RequestType &request) override {
    if (!getInput("goal_id", request.header.frame_id)) return;

    ros::spinOnce();
    request.header.stamp.sec = this->time_sec_;
    ROS_INFO("Brain: ExecuteMovement sending request.");
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("Brain: ExecuteMovement response SUCCEEDED.");

      Pose2D nav_pose{};
      nav_pose.fromROS(response.nav_pose);
      setOutput("nav_pose", nav_pose);

      PoseArray obj_poses{};
      obj_poses.fromROS(response.obj_poses);
      setOutput("obj_poses", obj_poses);
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
      InputPort<Pose>("tcp_pose"),
      InputPort<Pose2D>("nav_pose"),
      InputPort<PoseArray>("obj_poses"),
      OutputPort<int>("result_status")
    };
  }

  bool onSendGoal(GoalType& goal) override {
    // TODO
    if (!getInput<std::string>("goal_id", goal.header.frame_id)) return false;

    Pose tcp_pose{};
    if (!getInput<Pose>("tcp_pose", tcp_pose))
      return false;
    else
      goal.tcp_pose = tcp_pose.toROS();

    Pose2D nav_pose{};
    if (!getInput<Pose2D>("nav_pose", nav_pose))
      return false;
    else
      goal.nav_pose = nav_pose.toROS();

    PoseArray obj_poses{};
    try {
      getInput<PoseArray>("obj_poses", obj_poses);
      goal.obj_poses = obj_poses.toROS();
    } catch (const std::exception& e) {
      ROS_WARN("Brain: No obj_poses given on start.");
    }

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
  ros::init(argc, argv, "dummy_test_node");
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

  // Make sure Pose1 Pose2 are in the same with the action ids in xml file.
  RegisterRosAction<ExecuteNavigation>(factory, "Pose1", nh);
  RegisterRosService<ExecuteMovement>(factory, "Move1", nh);
  RegisterRosAction<ExecuteNavigation>(factory, "Pose2", nh);

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
