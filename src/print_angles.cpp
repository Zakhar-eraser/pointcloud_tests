#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>

float leftDistance = 0;
float rightDistance = 0;
float upperDistance = 0;
float lowerDistance = 0;
float yaw = 0;
float pitch = 0;

void Callback(sensor_msgs::PointCloud2ConstPtr msg)
{
    sensor_msgs::PointCloud2ConstIterator<float> iter(*(msg.get()), "x");
    upperDistance = iter[2];
    leftDistance = (iter + 1)[2];
    rightDistance = (iter + 3)[2];
    lowerDistance = (iter + 4)[2];
    yaw = atan2(leftDistance - rightDistance, (iter + 3)[0] * 2);
    pitch = atan2(upperDistance - lowerDistance, (iter + 4)[1] * 2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "print_angles_node");
    ros::NodeHandle nh;
    std::string topic2Subscribe;
    nh.getParam("mean_points_topic", topic2Subscribe);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic2Subscribe, 1, Callback);
    ros::Rate loop_rate(0.2);
    while(ros::ok())
    {
        ROS_INFO("Distances: left: %2.2f right: %2.2f upper: %2.2f lower: %2.2f; Angles: pitch: %1.2f yaw: %1.2f",
                        leftDistance, rightDistance, upperDistance, lowerDistance, pitch, yaw);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}