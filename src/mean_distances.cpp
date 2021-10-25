#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include "mean_distances/PointCloud2MatConverts.hpp"

ros::NodeHandle *nh;
ros::Subscriber sub;
ros::Publisher pub;

std::string topic2Subscribe;
std::string topic2Publish;

void Callback(sensor_msgs::PointCloud2ConstPtr msg)
{
    sensor_msgs::PointCloud2 point_cloud_msg = *msg.get();
    uint32_t width = point_cloud_msg.width;
    uint32_t height = point_cloud_msg.height;
    cv::Mat depthMat(height, width, CV_32FC1);
    cv::Mat colorMat(height, width, CV_8UC3);
    uint32_t windowWidth = width / 3;
    windowWidth -= 1 - windowWidth % 2;
    uint32_t windowHeight = height / 3;
    windowHeight -= 1 - windowHeight % 2;
    PointCloud22Mat(point_cloud_msg, depthMat, colorMat);
    cv::Rect rectLeft(0, windowHeight - 1, windowWidth, windowHeight);
    cv::Rect  rectCenter(windowWidth, windowHeight - 1, windowWidth, windowHeight);
    cv::Rect  rectRight(windowWidth * 2, windowHeight - 1, windowWidth, windowHeight);
    cv::Rect  rectUp(windowWidth, 0, windowWidth, windowHeight);
    cv::Rect  rectDown(windowWidth, windowHeight * 2, windowWidth, windowHeight);

    cv::Scalar_<uchar> colorMeansList[] = {cv::mean(colorMat(rectUp)), cv::mean(colorMat(rectLeft)),
                                            cv::mean(colorMat(rectCenter)), cv::mean(colorMat(rectRight)),
                                            cv::mean(colorMat(rectDown))};
    
    cv::Scalar_<float> depthMeansList[] = {cv::mean(depthMat(rectUp)), cv::mean(depthMat(rectLeft)),
                                            cv::mean(depthMat(rectCenter)), cv::mean(depthMat(rectRight)),
                                            cv::mean(depthMat(rectDown))};
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
        int order;
        if(i % 2 == 0)
        {
            order = windowHeight / 2 * (i + 1) * width + windowWidth / 2 * 3;
        }
        else
        {
            order = windowHeight / 2 * 3 * width + windowWidth / 2 * (5 * i / 3);
        }
        *write_x = *(read_x + order);
        *write_y = *(read_y + order);
        *write_z = depthMeansList[i][0];
        write_rgb[0] = colorMeansList[i][0];
        write_rgb[1] = colorMeansList[i][1];
        write_rgb[2] = colorMeansList[i][2];      
    }
    pub.publish(points);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mean_distances_node");
    nh = new ros::NodeHandle();
    nh->getParam("/mean_distances_node/pointcloud_topic", topic2Subscribe);
    nh->getParam("/mean_distances_node/point_topic", topic2Publish);
    sub = nh->subscribe<sensor_msgs::PointCloud2>(topic2Subscribe, 1, Callback);
    pub = nh->advertise<sensor_msgs::PointCloud2>(topic2Publish, 1);

    ros::spin();
    delete nh;
}