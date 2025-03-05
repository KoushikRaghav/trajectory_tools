#ifndef TRAJECTORY_PUBLISHER_SAVER_H
#define TRAJECTORY_PUBLISHER_SAVER_H

#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "amr_trajectory_tools/SaveTrajectory.h"
#include <deque>
#include <mutex>
#include <vector>
#include <fstream>
#include <ros/package.h>

struct StampedPose
{
  ros::Time stamp;
  turtlesim::Pose pose;
};

class TrajectoryPublisherSaver
{
public:
  explicit TrajectoryPublisherSaver(ros::NodeHandle& nh);

private:
  void pose_callback(const turtlesim::Pose::ConstPtr& msg);
  void publish_markers(const ros::TimerEvent&);
  bool save_trajectory_callback(amr_trajectory_tools::SaveTrajectory::Request &request,
                                amr_trajectory_tools::SaveTrajectory::Response &response);

  ros::NodeHandle nh_;
  std::deque<StampedPose> trajectory_buffer;
  std::mutex buffer_mutex;

  ros::Subscriber subscription;
  ros::Publisher marker_pub;
  ros::Timer marker_timer;
  ros::ServiceServer service;
};

#endif // TRAJECTORY_PUBLISHER_SAVER_H
