#include <walker_brain/bt_service_node.h>
#include <walker_brain/bt_action_node.h>
#include <walker_brain/bt_generic_types.h>
#include <walker_brain/rosout_logger.h>

// ROS
#include <ros/ros.h>

// Services (customized)
#include <walker_brain/EstimateTargetPose.h>
#include <walker_brain/MoveToPose2D.h>
#include <walker_brain/Dummy.h>
#include <hope/ExtractObjectOnTop.h>
#include <walker_nav/MoveToAbsPos.h>

// Actions (customized)
#include <walker_movement/MoveToEePoseAction.h>
#include <walker_movement/MoveToJointPoseAction.h>
#include <walker_movement/GraspAction.h>


using namespace BT;

class ExecuteWalkCmd : public RosServiceNode<walker_brain::Dummy>
{
public:
  ExecuteWalkCmd(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<walker_brain::Dummy>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<Pose>("cmd")
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("cmd", request.header.frame_id);
    ROS_INFO("Brain: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("Brain: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("Brain: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  virtual BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("Brain: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteMoveToAbsPos : public RosServiceNode<walker_nav::MoveToAbsPos>
{
public:
  ExecuteMoveToAbsPos(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<walker_nav::MoveToAbsPos>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<Pose>("pose")
    };
  }

  void onSendRequest(RequestType &request) override {
    Pose tgt_pose{};
    getInput("pose", tgt_pose);
    request.pose = tgt_pose.toROS();
    ROS_INFO("Brain: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    // TODO specify response in the srv
    return BT::NodeStatus::SUCCESS;
  }

  virtual BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("Brain: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteJointStates : public RosActionNode<walker_movement::MoveToJointPoseAction>
{
public:
  ExecuteJointStates(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & cfg):
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

class SenseObjectPoses : public RosServiceNode<hope::ExtractObjectOnTop>
{
public:
  SenseObjectPoses(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<hope::ExtractObjectOnTop>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("goal_id"),
      BT::OutputPort<int>("result_status"),
      BT::OutputPort<PoseArray>("poses")
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput("goal_id", request.goal_id.id);
    ros::spinOnce();  // synchronize clock time
    request.header.stamp.sec = this->time_sec_;
    ROS_INFO("Brain: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("Brain: %s response SUCCEEDED.", name_.c_str());

      PoseArray poses{};
      poses.fromROS(response.obj_poses);
      setOutput("poses", poses);

      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_INFO("Brain: %s response FAILURE.", name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  virtual BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("Brain: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class EstimateTargetPose : public RosServiceNode<walker_brain::EstimateTargetPose>
{
public:
  EstimateTargetPose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<walker_brain::EstimateTargetPose>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<PoseArray>("obj_poses"),
      BT::OutputPort<int>("result_status"),
      BT::OutputPort<Pose2D>("tgt_nav_pose"),
      BT::OutputPort<Pose2D>("compensate_pose")
    };
  }

  void onSendRequest(RequestType &request) override {
    PoseArray obj_poses;
    getInput("obj_poses", obj_poses);
    request.obj_poses = obj_poses.toROS();
    ros::spinOnce();
    request.header.stamp.sec = this->time_sec_;
    ROS_INFO("Brain: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      Pose2D tgt_nav_pose{};
      tgt_nav_pose.fromROS(response.tgt_nav_pose);
      setOutput("tgt_nav_pose", tgt_nav_pose);

      Pose2D compensate_pose{};
      compensate_pose.fromROS(response.compensate_pose);
      setOutput("compensate_pose", compensate_pose);

      ROS_INFO("Brain: %s response SUCCEEDED.", name_.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_INFO("Brain: %s response FAILURE.", name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  virtual BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("Brain: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class EstimateAdjustPose : public RosServiceNode<walker_brain::EstimateTargetPose>
{
public:
  EstimateAdjustPose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<walker_brain::EstimateTargetPose>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      BT::OutputPort<Pose2D>("tgt_nav_pose")
    };
  }

  void onSendRequest(RequestType &request) override {
    ROS_INFO("Brain: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      Pose2D tgt_nav_pose{};
      tgt_nav_pose.fromROS(response.tgt_nav_pose);
      setOutput("tgt_nav_pose", tgt_nav_pose);

      ROS_INFO("Brain: %s response SUCCEEDED.", name_.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_INFO("Brain: %s response FAILURE.", name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  virtual BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("Brain: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteMoveBase : public RosServiceNode<walker_brain::MoveToPose2D>
{
public:
  ExecuteMoveBase(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<walker_brain::MoveToPose2D>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<Pose2D>("pose")
    };
  }

  void onSendRequest(RequestType &request) override {
    Pose2D pose{};
    getInput("pose", pose);
    request.nav_pose = pose.toROS();
    ROS_INFO("Brain: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED)
      return BT::NodeStatus::SUCCESS;
    else
      return BT::NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("Brain: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecutePrePose : public RosServiceNode<walker_brain::Dummy>
{
public:
  ExecutePrePose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<walker_brain::Dummy>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {};
  }

  void onSendRequest(RequestType &request) override {
    ROS_INFO("Brain: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED)
      return BT::NodeStatus::SUCCESS;
    else
      return BT::NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("Brain: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_fridge_node");
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
  RegisterRosService<ExecuteWalkCmd>(factory, "ExecuteWalkCmd", nh);
  RegisterRosService<ExecuteMoveToAbsPos>(factory, "ExecuteMoveToAbsPos", nh);
  RegisterRosAction<ExecuteJointStates>(factory, "ExecuteHeadMove", nh);
  RegisterRosService<SenseObjectPoses>(factory, "SenseObjectPoses", nh);
  RegisterRosService<EstimateTargetPose>(factory, "EstimateTargetPose", nh);
  RegisterRosService<ExecuteMoveBase>(factory, "ExecuteMoveBase", nh);
  RegisterRosService<EstimateAdjustPose>(factory, "EstimateAdjustPose", nh);
  RegisterRosService<ExecutePrePose>(factory, "ExecutePrePose", nh);

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