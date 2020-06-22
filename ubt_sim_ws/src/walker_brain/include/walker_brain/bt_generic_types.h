#ifndef SRC_BT_GENERIC_TYPES_H
#define SRC_BT_GENERIC_TYPES_H

#include <ros/time.h>

#include <behaviortree_cpp_v3/bt_factory.h>


// std_msgs/Header
struct Header {
  uint32_t seq;
  ros::Time stamp;
  std::string frame_id;
};

// geometry_msgs/Pose2D
struct Pose2D {
  float x;
  float y;
  float theta;
};

// geometry_msgs/Pose
struct Pose {
  float px;
  float py;
  float pz;
  float ox;
  float oy;
  float oz;
  float ow;
};

// geometry_msgs/PoseArray
struct PoseArray {
  struct Pose poses[0];
};


namespace BT
{
  template <> inline Pose2D convertFromString(StringView str) {
    // We expect real numbers separated by spaces
    auto parts = splitString(str, ' ');
    if (parts.size() != 3) {
      throw RuntimeError("invalid input %s)", str);
    } else {
      Pose2D output{};
      output.x = convertFromString<float>(parts[0]);
      output.y = convertFromString<float>(parts[1]);
      output.theta = convertFromString<float>(parts[2]);
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
      output.px = convertFromString<float>(parts[0]);
      output.py = convertFromString<float>(parts[1]);
      output.pz = convertFromString<float>(parts[2]);
      output.ox = convertFromString<float>(parts[3]);
      output.oy = convertFromString<float>(parts[4]);
      output.oz = convertFromString<float>(parts[5]);
      output.ow = convertFromString<float>(parts[6]);
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
        p.px = convertFromString<float>(parts[0]);
        p.py = convertFromString<float>(parts[1]);
        p.pz = convertFromString<float>(parts[2]);
        p.ox = convertFromString<float>(parts[3]);
        p.oy = convertFromString<float>(parts[4]);
        p.oz = convertFromString<float>(parts[5]);
        p.ow = convertFromString<float>(parts[6]);
        output.poses[cnt] = p;
        cnt++;
      }
    }
    return output;
  }
} // end namespace BT

#endif //SRC_BT_GENERIC_TYPES_H
