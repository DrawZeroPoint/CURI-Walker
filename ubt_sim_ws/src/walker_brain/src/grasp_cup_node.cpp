#include <walker_brain/bt_service_node.h>
#include <walker_brain/bt_action_node.h>
#include <walker_brain/bt_generic_types.h>
#include <walker_brain/rosout_logger.h>

// ROS
#include <ros/ros.h>

// Services (customized)
#include <walker_brain/EstimateTargetPose.h>
#include <walker_brain/MoveToPose2D.h>
#include <hope/ExtractObjectOnTop.h>
//#include <walker_nav/MoveToRelPos.h>

// Actions (customized)
#include <walker_movement/MoveToEePoseAction.h>
#include <walker_movement/MoveToJointPoseAction.h>
#include <walker_movement/GraspAction.h>


using namespace BT;

class SenseObjectPoses : public RosServiceNode<hope::ExtractObjectOnTop>
{
public:
  SenseObjectPoses(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<hope::ExtractObjectOnTop>(nh, name, cfg) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("goal_id"),
      BT::OutputPort<int>("result_status"),
      BT::OutputPort<PoseArray>("obj_poses")
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput("goal_id", request.goal_id.id);
    ros::spinOnce();  // synchronize clock time
    request.header.stamp.sec = this->time_sec_;
    ROS_INFO("Brain: SenseObjectPoses sending request.");
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("Brain: SenseObjectPoses response SUCCEEDED.");

      PoseArray obj_poses{};
      obj_poses.fromROS(response.obj_poses);
      setOutput("obj_poses", obj_poses);

      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_INFO("Brain: SenseObjectPoses response FAILURE.");
      return BT::NodeStatus::FAILURE;
    }
  }

  virtual BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("Brain: SenseObjectPoses request failed %d.", static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }
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
      BT::OutputPort<Pose>("tgt_grasp_pose"),
      BT::OutputPort<Pose>("tgt_pre_grasp_pose")
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

      Pose tgt_grasp_pose{};
      tgt_grasp_pose.fromROS(response.tgt_grasp_pose);
      setOutput("tgt_grasp_pose", tgt_grasp_pose);

      Pose tgt_pre_grasp_pose{};
      tgt_pre_grasp_pose.fromROS(response.tgt_pre_grasp_pose);
      setOutput("tgt_pre_grasp_pose", tgt_pre_grasp_pose);

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

class ExecuteEEMove : public RosActionNode<walker_movement::MoveToEePoseAction>
{
public:
  ExecuteEEMove(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & cfg):
    RosActionNode<walker_movement::MoveToEePoseAction>(handle, name, cfg), name_(name) {}

  static PortsList providedPorts() {
    return {
      InputPort<std::string>("ee_link"),
      InputPort<Pose>("execute_pose"),
      InputPort<std::string>("ref_frame")
      //OutputPort<int>("result_status")
    };
  }

  bool onSendGoal(GoalType& goal) override {
    Pose execute_pose{};
    getInput<Pose>("execute_pose", execute_pose);
    goal.pose.pose = execute_pose.toROS();
    std::cerr << goal.pose.pose << std::endl;
    getInput<std::string>("ref_frame", goal.pose.header.frame_id);
    ros::spinOnce();
    goal.pose.header.stamp.sec = this->time_sec_;
    goal.pose.header.stamp.nsec = this->time_nsec_;
    getInput<std::string>("ee_link", goal.end_effector_link);

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

class ExecuteCloseHand : public RosActionNode<walker_movement::GraspAction>
{
public:
  ExecuteCloseHand(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & cfg):
    RosActionNode<walker_movement::GraspAction>(handle, name, cfg), name_(name) {}

  static PortsList providedPorts() {
    return {};
  }

  bool onSendGoal(GoalType& goal) override {
    goal.grasp_type = goal.GRASP_TYPE_CAN;
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

  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;

  // The recommended way to create a Node is through inheritance.
  RegisterRosService<SenseObjectPoses>(factory, "SenseObjectPoses", nh);
  RegisterRosService<EstimateTargetPose>(factory, "EstimateTargetPose", nh);
  RegisterRosService<ExecuteMoveBase>(factory, "ExecuteMoveBase", nh);
  RegisterRosAction<ExecuteFKMove>(factory, "ExecutePrePose", nh);
  RegisterRosAction<ExecuteEEMove>(factory, "ExecuteGrasp", nh);
  RegisterRosAction<ExecuteCloseHand>(factory, "ExecuteCloseHand", nh);
  RegisterRosAction<ExecuteEEMove>(factory, "ExecutePostGrasp", nh);

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