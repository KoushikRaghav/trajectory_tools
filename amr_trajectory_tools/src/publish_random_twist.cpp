#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_random_twist");
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Rate loop_rate(10);
    srand(time(0));

    while (ros::ok()) {
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.5* (double)rand() / (double)RAND_MAX;
        cmd_vel_msg.angular.z = 4.0 * ((double)rand() / (double)RAND_MAX - 0.5);
        cmd_vel_pub.publish(cmd_vel_msg);
        ROS_INFO("Publishing cmd_vel: linear.x = %f, angular.z = %f", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
        loop_rate.sleep();
    }
    return 0;
}