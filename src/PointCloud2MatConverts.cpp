#include "pointcloud_tests/PointCloud2MatConverts.hpp"

void PointCloud22Mat(sensor_msgs::PointCloud2 &point_cloud_msg, cv::Mat depthMat, cv::Mat colorMat)
{
    uint32_t rows_arg = point_cloud_msg.height;
    uint32_t cols_arg = point_cloud_msg.width;
    sensor_msgs::PointCloud2Iterator<float> write_z(point_cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> write_rgb(point_cloud_msg, "rgb");
    for (uint32_t j = 0; j < rows_arg; j++)
    {
      for (uint32_t i = 0; i < cols_arg; i++, ++write_z, ++write_rgb)
      {
        cv::Vec3b &bgr = colorMat.at<cv::Vec3b>(j, i);
        depthMat.at<float>(j, i) = *write_z;
        bgr[0] = write_rgb[0];
        bgr[1] = write_rgb[1];
        bgr[2] = write_rgb[2];
      }
    }
}

void Mat2PointCloud2(const cv::Mat depthMat, const cv::Mat colorMat,
                                                sensor_msgs::PointCloud2 &point_cloud_msg)
{
    uint32_t rows_arg = point_cloud_msg.height;
    uint32_t cols_arg = point_cloud_msg.width;
    sensor_msgs::PointCloud2Iterator<float> read_z(point_cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> read_rgb(point_cloud_msg, "rgb");
    for (uint32_t j = 0; j < rows_arg; j++)
    {
      for (uint32_t i = 0; i < cols_arg; i++, ++read_z, ++read_rgb)
      {
        const cv::Vec3b &bgr = colorMat.at<cv::Vec3b>(j, i);
        read_rgb[0] = bgr[0];
        read_rgb[1] = bgr[1];
        read_rgb[2] = bgr[2];
        *read_z = depthMat.at<float>(j, i);
      }
    }
}