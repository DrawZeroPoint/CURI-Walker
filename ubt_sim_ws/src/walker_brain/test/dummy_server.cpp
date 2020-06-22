#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/server/simple_action_server.h>
#include <walker_brain/DummyActionAction.h>


using namespace std;

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
  DummyActionServer(const std::string& name) :
    as_(nh_, name,  false),
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

  DummyActionServer as("dummy_action_server");
  ros::spin();

  return 0;
}
