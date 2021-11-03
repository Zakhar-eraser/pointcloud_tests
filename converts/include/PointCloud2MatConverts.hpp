#ifndef POINTCLOUD2_MAT_CONVERTS
#define POINTCLOUD2_MAT_CONVERTS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/core.hpp>

/**
 * @brief Copies depth and color channels of PointCloud2 message to OpenCV Mat
 * 
 * @param[in] msg PointCloud2 message to copy from
 * @param[in,out] depthMat one-channel depth cv::Mat to paste to 
 * @param[in,out] colorMat three-channel color (RGB) cv::Mat to paste to
 */
void PointCloud22Mat(sensor_msgs::PointCloud2 &msg, cv::Mat depthMat, cv::Mat colorMat);
/**
 * @brief Copies depth and color channels of cv::Mats to PointCloud2 message
 * 
 * @param[in] detphMat one-channel depth cv::Mat to copy from
 * @param[in] colorMat one-channel depth cv::Mat to copy from
 * @param[in,out] msg PointCloud2 message to paste to
 */
void Mat2PointCloud2(const cv::Mat detphMat,const cv::Mat colorMat, 
                                                sensor_msgs::PointCloud2 &msg);
#endif