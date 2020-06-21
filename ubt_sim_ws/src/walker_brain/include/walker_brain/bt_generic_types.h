#ifndef SRC_BT_GENERIC_TYPES_H
#define SRC_BT_GENERIC_TYPES_H

#include <behaviortree_cpp_v3/bt_factory.h>


// geometry_msgs/Pose2D
struct Pose2D
{
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

// 


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
} // end namespace BT

#endif //SRC_BT_GENERIC_TYPES_H
