#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

ros::NodeHandle *nh;

ros::Publisher objPub;
ros::Publisher backPub;

std::string topic2Subscribe;
std::string topic2PublishObjects;
std::string topic2PublishBackground;

void Threshold(sensor_msgs::PointCloud2ConstIterator<float> src,
            sensor_msgs::PointCloud2Iterator<float> dstObj,
            sensor_msgs::PointCloud2Iterator<float> dstBack,
            int width, int height, float thMin, float thMax)
{
    for(int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++)
        {
            int order = width * i + j;
            float value = (src + order)[2];
            if(value < thMin || value > thMax)
            {
                (dstObj + order)[2] = std::numeric_limits<float>::quiet_NaN();
                (dstBack + order)[2] = value;
            }
            else
            {
                (dstObj + order)[2] = value;
                (dstBack + order)[2] = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
}

void Callback(sensor_msgs::PointCloud2ConstPtr msg)
{
    float thresholdMin = 0.2f;
    float thresholdMax = 5.0f;
    nh->getParam("threshold_max", thresholdMax);
    nh->getParam("threshold_min", thresholdMin);
    sensor_msgs::PointCloud2 objCloud = *(msg.get());
    sensor_msgs::PointCloud2 backCloud = *(msg.get());
    sensor_msgs::PointCloud2ConstIterator<float> readIter(*(msg.get()), "x");
    sensor_msgs::PointCloud2Iterator<float> objIter(objCloud, "x");
    sensor_msgs::PointCloud2Iterator<float> backIter(backCloud, "x");
    Threshold(readIter, objIter, backIter, msg->width, msg->height, thresholdMin, thresholdMax);
    if(objPub.getNumSubscribers() > 0) objPub.publish(objCloud);
    if(backPub.getNumSubscribers() > 0) backPub.publish(backCloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thresholding_node");
    nh = new ros::NodeHandle();
    nh->getParam("pointcloud_topic", topic2Subscribe);
    nh->getParam("objects_cloud_topic", topic2PublishObjects);
    nh->getParam("background_cloud_topic", topic2PublishBackground);
    ros::Subscriber cloudSub = nh->subscribe<sensor_msgs::PointCloud2>(topic2Subscribe, 1, Callback);
    objPub = nh->advertise<sensor_msgs::PointCloud2>(topic2PublishObjects, 1);
    backPub = nh->advertise<sensor_msgs::PointCloud2>(topic2PublishBackground, 1);
    ros::spin();
    delete nh;
    return 0;
}