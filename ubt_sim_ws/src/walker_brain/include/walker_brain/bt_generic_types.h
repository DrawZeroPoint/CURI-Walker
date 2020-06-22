#ifndef SRC_BT_GENERIC_TYPES_H
#define SRC_BT_GENERIC_TYPES_H

#include <ros/time.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>

#include <behaviortree_cpp_v3/bt_factory.h>


// std_msgs/Header
struct Header {
  uint32_t seq;
  ros::Time stamp;
  std::string frame_id;
};

// geometry_msgs/Pose2D
struct Pose2D {
  double x;
  double y;
  double theta;

  inline geometry_msgs::Pose2D toROS() const {
    geometry_msgs::Pose2D ros_pose;
    ros_pose.x = this->x;
    ros_pose.y = this->y;
    ros_pose.theta = this->theta;
    return ros_pose;
  }

  inline void fromROS(geometry_msgs::Pose2D ros_pose) {
    this->x = ros_pose.x;
    this->y = ros_pose.y;
    this->theta = ros_pose.theta;
  }
};

// geometry_msgs/Pose
struct Pose {
  double px;
  double py;
  double pz;
  double ox;
  double oy;
  double oz;
  double ow;

  inline geometry_msgs::Pose convertToROS() const {
    geometry_msgs::Pose ros_pose;
    ros_pose.position.x = this->px;
    ros_pose.position.y = this->py;
    ros_pose.position.z = this->pz;
    ros_pose.orientation.x = this->ox;
    ros_pose.orientation.y = this->oy;
    ros_pose.orientation.z = this->oz;
    ros_pose.orientation.w = this->ow;
    return ros_pose;
  }
};

// geometry_msgs/PoseArray
struct PoseArray {
  struct Pose poses[0];
};

/**
 * Note that BT::convertFromString() do not support template specialization of float,
 * so we use double for all the float values.
 */
namespace BT
{
  template <> inline Pose2D convertFromString(StringView str) {
    // We expect real numbers separated by spaces
    auto parts = splitString(str, ' ');
    if (parts.size() != 3) {
      throw RuntimeError("invalid input %s)", str);
    } else {
      Pose2D output{};
      output.x = convertFromString<double>(parts[0]);
      output.y = convertFromString<double>(parts[1]);
      output.theta = convertFromString<double>(parts[2]);
      return output;
    }
  }

  template <> inline Pose convertFromString(StringView str) {
    // We expect real numbers separated by spaces
    auto parts = splitString(str, ' ');
    if (parts.size() != 7) {
      throw RuntimeError("invalid input %s)", str);
    } else {
      Pose output{};
      output.px = convertFromString<double>(parts[0]);
      output.py = convertFromString<double>(parts[1]);
      output.pz = convertFromString<double>(parts[2]);
      output.ox = convertFromString<double>(parts[3]);
      output.oy = convertFromString<double>(parts[4]);
      output.oz = convertFromString<double>(parts[5]);
      output.ow = convertFromString<double>(parts[6]);
      return output;
    }
  }

  template <> inline PoseArray convertFromString(StringView str) {
    // We expect poses separated by semicolons
    auto poses_str = splitString(str, ';');
    int len = poses_str.size();
    PoseArray output{};
    output.poses[len];

    int cnt = 0;
    for (auto p_str : poses_str) {
      // We expect real numbers separated by spaces
      auto parts = splitString(p_str, ' ');
      if (parts.size() != 7) {
        throw RuntimeError("invalid input %s)", p_str);
      } else {
        Pose p{};
        p.px = convertFromString<double>(parts[0]);
        p.py = convertFromString<double>(parts[1]);
        p.pz = convertFromString<double>(parts[2]);
        p.ox = convertFromString<double>(parts[3]);
        p.oy = convertFromString<double>(parts[4]);
        p.oz = convertFromString<double>(parts[5]);
        p.ow = convertFromString<double>(parts[6]);
        output.poses[cnt] = p;
        cnt++;
      }
    }
    return output;
  }
} // end namespace BT

#endif //SRC_BT_GENERIC_TYPES_H
