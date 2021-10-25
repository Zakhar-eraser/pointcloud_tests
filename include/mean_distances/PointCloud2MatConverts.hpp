#ifndef POINTCLOUD2_MAT_CONVERTS
#define POINTCLOUD2_MAT_CONVERTS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/core.hpp>

void PointCloud22Mat(sensor_msgs::PointCloud2 &msg, cv::Mat depthMat, cv::Mat colorMat);
void Mat2PointCloud2(const cv::Mat detphMat,const cv::Mat colorMat, 
                                                sensor_msgs::PointCloud2 &msg);
#endif