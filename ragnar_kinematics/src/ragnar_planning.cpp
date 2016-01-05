#include "ragnar_kinematics/ragnar_planning.h"
#include <ros/assert.h>

// Linear interpolate
static inline double interpolate(double start, double stop, double ratio)
{
  return start + (stop - start) * ratio;
}

// Utility function for pose interpolation
static inline ragnar_kinematics::RagnarPose interpPose(const ragnar_kinematics::RagnarPose& start,
                                                       const ragnar_kinematics::RagnarPose& stop,
                                                       double ratio)
{
  ragnar_kinematics::RagnarPose pose;
  for (unsigned i = 0; i < 4; i++)
    pose.pose[i] = interpolate(start.pose[i], stop.pose[i], ratio);
  return pose;
}


bool ragnar_kinematics::position::linear(const ragnar_kinematics::RagnarPose& start,
                                         const ragnar_kinematics::RagnarPose& stop,
                                         double ds,
                                         std::vector<ragnar_kinematics::RagnarPoint>& results)
{
  double delta_x = stop.pose[0] - start.pose[0];
  double delta_y = stop.pose[1] - start.pose[1];
  double delta_z = stop.pose[2] - start.pose[2];
  double delta_s = std::sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

  unsigned steps = static_cast<unsigned>(delta_s / ds) + 1;

  return linear(start, stop, steps, results);
}


bool ragnar_kinematics::position::linear(const ragnar_kinematics::RagnarPose& start,
                                         const ragnar_kinematics::RagnarPose& stop,
                                         unsigned steps,
                                         std::vector<ragnar_kinematics::RagnarPoint>& results)
{
  // We mandate that steps be greater than zero, otherwise we get a divide by zero
  if (steps == 0) return false;
  std::vector<ragnar_kinematics::RagnarPoint> pts;
  pts.reserve(steps);

  for (unsigned i = 1; i <= steps; i++)
  {
    double ratio = static_cast<double>(i) / steps;
    RagnarPose pose = interpPose(start, stop, ratio);
    RagnarPoint pt;
    if (!inverse_kinematics(pose, pt)) return false;
    pts.push_back(pt); // if we can do the IK, add the point
  }

  results = pts;
  return true;
}

bool ragnar_kinematics::position::arc(const ragnar_kinematics::RagnarPose& origin,
                                      double start_angle,
                                      double stop_angle,
                                      double dtheta,
                                      std::vector<ragnar_kinematics::RagnarPoint>& results)
{
  return false;
}

bool ragnar_kinematics::timing::linear(std::size_t steps, double total_t, std::vector<double>& timing)
{
  // pre-condition that steps > 0; matches behavior of linear
  if (steps == 0) return false;

  std::vector<double> times;
  times.reserve(steps);
  for (unsigned i = 1; i <= steps; ++i)
  {
    double ratio = static_cast<double>(i) / steps;
    times.push_back(interpolate(0.0, total_t, ratio));
  }
  timing = times; // copy to results
  return true;
}


std::vector<trajectory_msgs::JointTrajectoryPoint>
ragnar_kinematics::toROSJointPoints(const std::vector<ragnar_kinematics::RagnarPoint>& position,
                               const std::vector<double>& timing)
{
  ROS_ASSERT(position.size() == timing.size());
  // results vector
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;
  points.reserve(position.size());
  // loop
  for (size_t i = 0; i < position.size(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions.assign(position[i].joints, position[i].joints + 4); // copy positions
    pt.time_from_start = ros::Duration(timing[i]); // copy timing
    points.push_back(pt); // push into result
  }
  return points;
}

void ragnar_kinematics::position::append(const std::vector<ragnar_kinematics::RagnarPoint>& a, const std::vector<ragnar_kinematics::RagnarPoint>& b, std::vector<ragnar_kinematics::RagnarPoint>& out)
{
  out.clear();
  out.reserve(a.size() + b.size());
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
}

void ragnar_kinematics::timing::append(const std::vector<double>& a, const std::vector<double>& b, std::vector<double>& out)
{
  out.clear();
  out.reserve(a.size() + b.size());
  // Copy the first trajectory
  out.assign(a.begin(), a.end());
  // Copy the final value in a
  double end_of_a = a.back();
  // Iterate through each element in b and add 'end_of_a'
  for (size_t i = 0; i < b.size(); ++i)
    out.push_back(b[i] + end_of_a);
}

ragnar_kinematics::motion::Segment::Segment(const std::vector<ragnar_kinematics::RagnarPoint>& joints, const std::vector<double>& timing)
  : joints_(joints)
  , timing_(timing)
{
  // assert size > 0
  ROS_ASSERT(joints.size() > 0);
  ROS_ASSERT(timing.size() > 0);
}

std::vector<trajectory_msgs::JointTrajectoryPoint> ragnar_kinematics::motion::Segment::toROSJointPoints() const
{
  return ::ragnar_kinematics::toROSJointPoints(joints_, timing_);
}

ragnar_kinematics::motion::Segment ragnar_kinematics::motion::operator+(const ragnar_kinematics::motion::Segment& lhs, const ragnar_kinematics::motion::Segment& rhs)
{
  std::vector<RagnarPoint> joints;
  std::vector<double> time;
  position::append(lhs.joints(), rhs.joints(), joints);
  timing::append(lhs.timing(), rhs.timing(), time);
  return Segment(joints, time);
}

ragnar_kinematics::motion::LinearMove::LinearMove(const ragnar_kinematics::RagnarPose& start, const ragnar_kinematics::RagnarPose& stop, double time, double discretization)
{
  if (!position::linear(start, stop, discretization, joints_)) throw std::runtime_error("Could not compute joints for linear motion");
  if (!timing::linear(joints_.size(), time, timing_)) throw std::runtime_error("Could not compute timing for linear motion");
}
