#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

ros::NodeHandle *nh;

ros::Publisher objPub;
ros::Publisher backPub;

std::string topic2Subscribe;
std::string topic2PublishObjects;
std::string topic2PublishBackground;

float Mean(sensor_msgs::PointCloud2ConstIterator<float> src,
            int squareSize, int orderX, int orderY, int width)
{
    int halfSize = squareSize / 2;
    float summ = 0;
    for(int i = orderY - halfSize; i < orderY + halfSize + 1; i++)
    {
        for(int j = orderX - halfSize; j < orderX + halfSize + 1; j++)
        {
            summ += (src + (i * width + j))[2];
        }
    }
    return summ / (squareSize * squareSize);
}

void Threshold(sensor_msgs::PointCloud2ConstIterator<float> src,
            sensor_msgs::PointCloud2Iterator<float> dstObj,
            sensor_msgs::PointCloud2Iterator<float> dstBack,
            int squareSize, int width, int height, float error)
{
    int halfSize = squareSize / 2;
    for(int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++)
        {
            int order = i * width + j;
            if(i < halfSize || j < halfSize || i > height - halfSize - 1 || j > width - halfSize - 1)
            {
                (dstObj + order)[2] = std::numeric_limits<float>::quiet_NaN();
            }
            else
            {
                float mean = Mean(src, squareSize, j, i, width);
                float value = (src + order)[2] - mean;
                if(/*(src + order)[2] > mean*/ value * value / mean * 10000.0f > error)
                {
                    (dstBack + order)[2] = std::numeric_limits<float>::quiet_NaN();
                }
                else
                {
                    (dstBack + order)[2] = mean;
                    (dstObj + order)[2] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
    }
}

void Callback(sensor_msgs::PointCloud2ConstPtr msg)
{
    int squareSize = 1;
    float error = 5.0f;
    nh->getParam("square_size", squareSize);
    nh->getParam("error", error);
    sensor_msgs::PointCloud2 objCloud = *(msg.get());
    sensor_msgs::PointCloud2 backCloud = *(msg.get());
    sensor_msgs::PointCloud2ConstIterator<float> readIter(*(msg.get()), "x");
    sensor_msgs::PointCloud2Iterator<float> objIter(objCloud, "x");
    sensor_msgs::PointCloud2Iterator<float> backIter(backCloud, "x");
    Threshold(readIter, objIter, backIter, squareSize, msg->width, msg->height, error);
    if(objPub.getNumSubscribers() > 0) objPub.publish(objCloud);
    if(backPub.getNumSubscribers() > 0) backPub.publish(backCloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "adaptive_thresholding_node");
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