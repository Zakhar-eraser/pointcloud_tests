/**
 * @file mean_distances.cpp
 * @author Zakhar Anikin (79922187361@yandex.ru)
 * @brief Gets PointCloud2 message and calculate mean distances in 5 areas
 * @version 0.1
 * @date 2021-11-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include "PointCloud2MatConverts.hpp"

ros::NodeHandle *nh;
ros::Subscriber sub;
ros::Publisher pub;

std::string topic2Subscribe;
std::string topic2Publish;
int windowSize = 1;

int Saturate(int value, int leftBorder, int rightBorder)
{
    value += 1 - value % 2;
    if(value > rightBorder)
    {
        return rightBorder;
    }
    else if(value < leftBorder) return leftBorder;
    return value;
}

void Callback(sensor_msgs::PointCloud2ConstPtr msg)
{
    nh->getParam("window_size", windowSize);
    sensor_msgs::PointCloud2 point_cloud_msg = *msg.get();
    uint32_t width = point_cloud_msg.width;
    uint32_t height = point_cloud_msg.height;
    cv::Mat depthMat(height, width, CV_32FC1);
    cv::Mat colorMat(height, width, CV_8UC3);
    uint32_t windowWidth = width / 3;
    uint32_t windowHeight = height / 3;
    uint32_t sizeX = Saturate(windowSize, 1, windowWidth);
    uint32_t sizeY = Saturate(windowSize, 1, windowHeight);
    PointCloud22Mat(point_cloud_msg, depthMat, colorMat);
    sensor_msgs::PointCloud2 points;
    points.header = point_cloud_msg.header;
    points.is_bigendian = point_cloud_msg.is_bigendian;
    points.is_dense = point_cloud_msg.is_dense;
    sensor_msgs::PointCloud2Modifier pcd_modifier(points);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    pcd_modifier.resize(5);
    sensor_msgs::PointCloud2Iterator<float> read_x(point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> read_y(point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> write_x(points, "x");
    sensor_msgs::PointCloud2Iterator<float> write_y(points, "y");
    sensor_msgs::PointCloud2Iterator<float> write_z(points, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> write_rgb(points, "rgb");
    for(uint32_t i = 0; i < 5; i++, ++write_x, ++write_y, ++write_z, ++write_rgb)
    {
        int centerX;
        int centerY;
        if(i % 2 == 0)
        {
            centerX = width / 2;
            centerY = windowHeight * (i + 1) / 2;
        }
        else
        {
            centerX = windowWidth * (5 * i / 3) / 2;
            centerY = height / 2;
        }
        int order = centerY * width + centerX;
        *write_x = *(read_x + order);
        *write_y = *(read_y + order);
        cv::Rect rect(centerX - sizeX / 2, centerY - sizeY / 2, sizeX, sizeY);
        cv::Scalar_<float> depth = cv::mean(depthMat(rect));
        cv::Scalar_<uchar> color = cv::mean(colorMat(rect));
        *write_z = depth[0];
        write_rgb[0] = color[0];
        write_rgb[1] = color[1];
        write_rgb[2] = color[2];     
    }
    pub.publish(points);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mean_distances_node");
    nh = new ros::NodeHandle();
    nh->getParam("pointcloud_topic", topic2Subscribe);
    nh->getParam("mean_points_topic", topic2Publish);
    sub = nh->subscribe<sensor_msgs::PointCloud2>(topic2Subscribe, 1, Callback);
    pub = nh->advertise<sensor_msgs::PointCloud2>(topic2Publish, 1);

    ros::spin();
    delete nh;
}