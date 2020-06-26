#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/server/simple_action_server.h>
#include <walker_brain/Dummy.h>
#include <walker_brain/DummyActionAction.h>


using namespace std;

class DummyServiceServer
{
public:
  explicit DummyServiceServer(const string &name) {
    server_ = nh_.advertiseService(name, &DummyServiceServer::requestCB, this);
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceServer server_;

  bool requestCB(walker_brain::Dummy::Request &request, walker_brain::Dummy::Response &response) {
    bool success = true;
    if (request.header.frame_id.empty()) {
      success = false;
    } else {
      int cnt = 3;
      while (cnt) {
        ROS_INFO("Brain: Dummy action server executing %d", cnt);
        if (!ros::ok()) {
          success = false;
          break;
        }
        geometry_msgs::Pose pose{};
        pose.position.x = cnt;
        pose.position.y = 2*cnt;
        pose.position.z = 3*cnt;
        pose.orientation.w = 1;
        response.obj_poses.poses.push_back(pose);
        cnt--;
      }
      response.nav_pose.x = 99;
      response.nav_pose.y = -99;
      response.nav_pose.theta = 1;
    }
    success ? response.result_status = response.SUCCEEDED : response.result_status = response.FAILED;
    return true;
  }
};

class DummyActionServer
{
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<walker_brain::DummyActionAction> as_;
  string action_name_;
  // create messages that are used to published feedback/result
  boost::shared_ptr<const walker_brain::DummyActionGoal_ <std::allocator<void> > > goal_;
  walker_brain::DummyActionFeedback feedback_;
  walker_brain::DummyActionResult result_;

public:
  explicit DummyActionServer(const std::string& name) :
    as_(nh_, name, false),
    action_name_(name)
  {
    as_.registerGoalCallback([this] { goalCB(); });
    as_.registerPreemptCallback([this] { preemptCB(); });
    as_.start();
  }

  ~DummyActionServer() = default;

  void goalCB() {
    ros::Rate r(1);
    bool success = true;

    goal_ = as_.acceptNewGoal();
    if (!goal_->header.frame_id.empty()) {
      int cnt = 5;
      while (cnt) {
        if (as_.isPreemptRequested() || !ros::ok()) {
          success = false;
          break;
        }
        feedback_.feedback_status = 2;
        as_.publishFeedback(feedback_);
        ROS_INFO("Brain: Dummy action server executing %d", cnt);
        ROS_INFO("Brain: TCP moving to x=%.2f y=%.2f z=%.2f", goal_->tcp_pose.position.x, goal_->tcp_pose.position.y,
                 goal_->tcp_pose.position.z);
        ROS_INFO("Brain: Nav moving to x=%.2f y=%.2f theta=%.2f", goal_->nav_pose.x, goal_->nav_pose.y,
                 goal_->nav_pose.theta);
        if (goal_->obj_poses.poses.size() >= 3) {
          ROS_INFO("Brain: Object poses 3 x=%.2f y=%.2f z=%.2f", goal_->obj_poses.poses[2].position.x,
                   goal_->obj_poses.poses[2].position.y, goal_->obj_poses.poses[2].position.z);
        }
        r.sleep();
        cnt--;
      }
    } else {
      success = false;
    }
    result_.result_status = success ? result_.SUCCEEDED : result_.FAILED;
    as_.setSucceeded(result_);
  }

  void preemptCB() {
    as_.setPreempted();
  }

private:

};


int main(int argc, char** argv) {
  ros::init(argc, argv, "dummy_node");

  DummyServiceServer ss("dummy_sense_service");
  DummyActionServer as("dummy_action_server");
  ros::spin();

  return 0;
}
