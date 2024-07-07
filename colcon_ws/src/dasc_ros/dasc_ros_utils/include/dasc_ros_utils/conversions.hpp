

#include "dasc_msgs/msg/di_trajectory.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_geometry_msgs.hpp"
//#include <tf2/utils.h>

// #ifdef TF2_CPP_HEADERS
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #else
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #endif

namespace tf2 {

using namespace geometry_msgs::msg;
using namespace dasc_msgs::msg;

inline double getYaw(const tf2::Quaternion &q) {
  double yaw;

  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x() * q.x();
  sqy = q.y() * q.y();
  sqz = q.z() * q.z();
  sqw = q.w() * q.w();

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg =
      -2 * (q.x() * q.z() - q.w() * q.y()) /
      (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */

  if (sarg <= -0.99999) {
    yaw = -2 * atan2(q.y(), q.x());
  } else if (sarg >= 0.99999) {
    yaw = 2 * atan2(q.y(), q.x());
  } else {
    yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), sqw + sqx - sqy - sqz);
  }
  return yaw;
}

void doTransform(const Twist &in, Twist &out, const TransformStamped &t) {
  doTransform(in.linear, out.linear, t);
  doTransform(in.angular, out.angular, t);
}

void doTransform(const Accel &in, Accel &out, const TransformStamped &t) {
  doTransform(in.linear, out.linear, t);
  doTransform(in.angular, out.angular, t);
}

void doTransform(const DITrajectory &in, DITrajectory &out,
                 const TransformStamped &t) {
  // update the header
  out.header.stamp = t.header.stamp;
  out.header.frame_id = t.child_frame_id;
  out.dt = in.dt;

  // now do the transform of each pose
  out.poses.clear();
  for (auto p : in.poses) {
    geometry_msgs::msg::Pose pp;
    tf2::doTransform(p, pp, t);
    out.poses.push_back(pp);
  }
  // repeat for the twists
  out.twists.clear();
  for (auto p : in.twists) {
    geometry_msgs::msg::Twist pp;
    tf2::doTransform(p, pp, t);
    out.twists.push_back(pp);
  }
  // repeat for the accels
  out.accelerations.clear();
  for (auto p : in.accelerations) {
    geometry_msgs::msg::Accel pp;
    tf2::doTransform(p, pp, t);
    out.accelerations.push_back(pp);
  }
}

} // namespace tf2

namespace tf2 {
inline double getYaw(const geometry_msgs::msg::Quaternion &q) {

  return tf2::getYaw(tf2::Quaternion(q.x, q.y, q.z, q.w));
}
} // namespace tf2
