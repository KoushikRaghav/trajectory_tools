#include "amr_trajectory_tools/trajectory_reader_publisher.h"

TrajectoryReaderPublisher::TrajectoryReaderPublisher(ros::NodeHandle& nh) : nh_(nh)
{
  publishStaticTransform();

  // nh_.param<std::string>("trajectory_file", trajectory_file, "trajectory_data.csv");
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("trajectory_file", trajectory_file, "trajectory_data.csv");
  ROS_INFO("Using trajectory file: %s", trajectory_file.c_str());

  if (!readTrajectoryFile(trajectory_file)) {
    ROS_ERROR("Unable to read trajectory file: %s", trajectory_file.c_str());
  }

  marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("saved_trajectory_markers", 10);
  marker_timer = nh_.createTimer(ros::Duration(1.0), 
                                  &TrajectoryReaderPublisher::publishMarkers, this);

  ROS_INFO("Trajectory reader node initiated");
}

void TrajectoryReaderPublisher::publishStaticTransform()
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transform;

  static_transform.header.stamp = ros::Time::now();
  static_transform.header.frame_id = "odom";
  static_transform.child_frame_id = "map";

  static_transform.transform.translation.x = 0.0;
  static_transform.transform.translation.y = 0.0;
  static_transform.transform.translation.z = 0.0;

  static_transform.transform.rotation.x = 0.0;
  static_transform.transform.rotation.y = 0.0;
  static_transform.transform.rotation.z = 0.0;
  static_transform.transform.rotation.w = 1.0;

  static_broadcaster.sendTransform(static_transform);

  ROS_INFO("Published static transform from 'odom' to 'map'");
}

bool TrajectoryReaderPublisher::readTrajectoryFile(const std::string & filename)
{
  std::string package_path = ros::package::getPath("amr_trajectory_tools");

  std::string file_path = package_path + "/trajectory_files/" + filename;

  std::ifstream file(file_path.c_str());
  if (!file.is_open()) {
    ROS_ERROR("Failed to open file: %s", file_path.c_str());
    return false;
  }

  std::string line;
  if (std::getline(file, line)) {
    if (line.find("timestamp") != std::string::npos) {
      ROS_INFO("Skipping header");
    } else {
      processLine(line);
    }
  }

  while (std::getline(file, line)) {
    if (!line.empty()) {
      processLine(line);
    }
  }

  file.close();
  return true;
}

void TrajectoryReaderPublisher::processLine(const std::string & line)
{
  std::istringstream ss(line);
  std::string token;
  
  std::getline(ss, token, ',');
  double timestamp = std::stod(token);
  std::getline(ss, token, ',');
  double x = std::stod(token);
  std::getline(ss, token, ',');
  double y = std::stod(token);
  std::getline(ss, token, ',');
  double theta = std::stod(token);

  geometry_msgs::PointStamped point;
  point.header.frame_id = "map";
  point.header.stamp = ros::Time(timestamp);
  point.point.x = x - 5.5;
  point.point.y = y - 5.5;
  point.point.z = 0.0;

  points.push_back(point);
}

void TrajectoryReaderPublisher::publishMarkers(const ros::TimerEvent&)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  for (const auto & point_in_map : points) {
    geometry_msgs::PointStamped point_in_odom;
    try {
      listener_.transformPoint("odom", point_in_map, point_in_odom);
      marker.points.push_back(point_in_odom.point);
    } catch (tf::TransformException &ex) {
      ROS_WARN("Failed to transform point: %s", ex.what());
    }
  }

  marker_array.markers.push_back(marker);
  marker_pub.publish(marker_array);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "trajectory_reader_publisher");
  ros::NodeHandle nh;
  
  TrajectoryReaderPublisher trajectory_reader(nh);
  
  ros::spin();
  return 0;
}
