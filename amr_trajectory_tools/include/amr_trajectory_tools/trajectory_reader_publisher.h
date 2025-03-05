#ifndef TRAJECTORY_READER_PUBLISHER_H
#define TRAJECTORY_READER_PUBLISHER_H

#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_listener.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <ros/package.h>

class TrajectoryReaderPublisher
{
public:
  explicit TrajectoryReaderPublisher(ros::NodeHandle& nh);

private:
  void publishStaticTransform();
  bool readTrajectoryFile(const std::string & filename);
  void processLine(const std::string & line);
  void publishMarkers(const ros::TimerEvent&);

  ros::NodeHandle nh_;
  std::string marker_topic;
  std::string trajectory_file;
  std::vector<geometry_msgs::PointStamped> points;
  ros::Publisher marker_pub;
  ros::Timer marker_timer;
  tf::TransformListener listener_;
};

#endif // TRAJECTORY_READER_PUBLISHER_H
