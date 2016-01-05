#ifndef RAGNAR_PLANNING_H
#define RAGNAR_PLANNING_H

#include "ragnar_kinematics/ragnar_kinematics.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

namespace ragnar_kinematics
{

// Utility structures
struct RagnarPoint
{
  RagnarPoint(double j1, double j2, double j3, double j4) {
    joints[0] = j1;
    joints[1] = j2;
    joints[2] = j3;
    joints[3] = j4;
  }

  RagnarPoint() {}

  double joints[4];
};

struct RagnarPose
{
  RagnarPose(double x, double y, double z) {
    pose[0] = x;
    pose[1] = y;
    pose[2] = z;
    pose[3] = 0.0;
  }

  RagnarPose() {}

  double pose[4];
};

// Wrapper to make the kinematics a little more friendly
inline bool inverse_kinematics(const RagnarPose& pose, RagnarPoint& point)
{
  return inverse_kinematics(pose.pose, point.joints);
}

inline bool forward_kinematics(const RagnarPoint& point, RagnarPose& pose)
{
  return forward_kinematics(point.joints, pose.pose);
}

// Create 'path' segments; these deal with position only, not velocities
namespace position
{
  bool linear(const RagnarPose& start, const RagnarPose& stop, double ds, std::vector<RagnarPoint>& results);

  bool linear(const RagnarPose& start, const RagnarPose& stop, unsigned steps, std::vector<RagnarPoint>& results);

  bool arc(const RagnarPose& origin, double start_angle, double stop_angle, double dtheta, std::vector<RagnarPoint>& results);

  void append(const std::vector<RagnarPoint>& a, const std::vector<RagnarPoint>& b, std::vector<RagnarPoint>& out);
}

namespace timing
{
  bool linear(std::size_t steps, double total_t, std::vector<double>& timing);

  void append(const std::vector<double>& a, const std::vector<double>& b, std::vector<double>& out);
}

namespace motion
{
  class Segment
  {
  public:
    Segment(const std::vector<RagnarPoint>& joints, const std::vector<double>& timing);

    const std::vector<RagnarPoint>& joints() const { return joints_; }
    const std::vector<double>& timing() const { return timing_; }

    std::vector<trajectory_msgs::JointTrajectoryPoint> toROSJointPoints() const;

  protected:
    Segment() {}

    std::vector<RagnarPoint> joints_;
    std::vector<double> timing_;
  };

  class LinearMove : public Segment {
  public:
    LinearMove(const RagnarPose& start, const RagnarPose& stop, double time, double discretization);
  };

  Segment operator+(const Segment& lhs, const Segment& rhs);
}

std::vector<trajectory_msgs::JointTrajectoryPoint>
toROSJointPoints(const std::vector<RagnarPoint>& position, const std::vector<double>& timing);

}

#endif // RAGNAR_PLANNING_H
