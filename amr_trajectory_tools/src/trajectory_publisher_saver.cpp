#include "amr_trajectory_tools/trajectory_publisher_saver.h"

TrajectoryPublisherSaver::TrajectoryPublisherSaver(ros::NodeHandle& nh) : nh_(nh)
{
  std::string pose_topic, marker_topic, save_trajectory_service;
  double marker_timer_duration;

  nh_.param<std::string>("pose_topic", pose_topic, "/turtle1/pose");
  nh_.param<std::string>("marker_topic", marker_topic, "/robot_trajectory_markers");
  nh_.param<std::string>("save_trajectory", save_trajectory_service, "/save_trajectory");
  nh_.param<double>("marker_timer_duration", marker_timer_duration, 0.5);

  subscription = nh_.subscribe(pose_topic, 10, &TrajectoryPublisherSaver::pose_callback, this);
  marker_pub = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic, 10);
  marker_timer = nh_.createTimer(ros::Duration(marker_timer_duration),
                                  &TrajectoryPublisherSaver::publish_markers, this);
  service = nh_.advertiseService(save_trajectory_service,
                                  &TrajectoryPublisherSaver::save_trajectory_callback, this);

  ROS_INFO("Trajectory node initiated with topics: %s, %s, service: %s",
           pose_topic.c_str(), marker_topic.c_str(), save_trajectory_service.c_str());
}


void TrajectoryPublisherSaver::pose_callback(const turtlesim::Pose::ConstPtr& msg)
{
  ros::Time current_time = ros::Time::now();
  StampedPose stamped_pose{current_time, *msg};

  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    trajectory_buffer.push_back(stamped_pose);
  }
}

void TrajectoryPublisherSaver::publish_markers(const ros::TimerEvent&)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "trajectory";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;
  marker.color.r = 1.0;
  marker.color.a = 1.0;

  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    for (const auto & stamped_pose : trajectory_buffer) {
      geometry_msgs::Point pt;
      pt.x = stamped_pose.pose.x - 5.5;
      pt.y = stamped_pose.pose.y - 5.5;
      pt.z = 0.0;
      marker.points.push_back(pt);
    }
  }

  marker_array.markers.push_back(marker);
  marker_pub.publish(marker_array);
}

bool TrajectoryPublisherSaver::save_trajectory_callback(
  amr_trajectory_tools::SaveTrajectory::Request &request,
  amr_trajectory_tools::SaveTrajectory::Response &response)
{
  std::string package_path = ros::package::getPath("amr_trajectory_tools");
  if (package_path.empty()) {
    response.success = false;
    response.message = "Failed to find package path";
    ROS_ERROR("Failed to find package path for amr_trajectory_tools");
    return false;
  }

  std::string file_path = package_path + "/trajectory_files/" + request.filename;

  ros::Time current_time = ros::Time::now();
  std::vector<StampedPose> filtered_poses;

  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    for (auto rit = trajectory_buffer.rbegin(); rit != trajectory_buffer.rend(); ++rit) {
      if ((current_time - rit->stamp).toSec() <= request.duration) {
        filtered_poses.insert(filtered_poses.begin(), *rit);
      } else {
        break;
      }
    }
  }

  std::ofstream outfile(file_path.c_str());
  if (!outfile.is_open()) {
    response.success = false;
    response.message = "Failed to open file: " + file_path;
    ROS_ERROR("Failed to open file: %s", file_path.c_str());
    return false;
  }

  outfile << "timestamp,x,y,theta\n";
  for (const auto & pose : filtered_poses) {
    outfile << pose.stamp.toSec() << "," << pose.pose.x << ","
            << pose.pose.y << "," << pose.pose.theta << "\n";
  }
  outfile.close();

  response.success = true;
  response.message = "Trajectory saved";
  return true;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "trajectory_publisher_saver");
  ros::NodeHandle nh;
  TrajectoryPublisherSaver trajectory_node(nh);
  ros::spin();
  return 0;
}
