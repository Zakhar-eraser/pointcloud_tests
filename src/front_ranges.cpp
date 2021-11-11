#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

ros::Publisher rangePubLeft;
ros::Publisher rangePubRight;
ros::NodeHandle *nh;

int pointsFromCenter = 1;

sensor_msgs::Range leftRange;
sensor_msgs::Range rightRange;

void Callback(sensor_msgs::PointCloud2ConstPtr msg)
{
    int width = msg->width;
    int height = msg->height;
    int centerPointOrder = height / 2 * width + width / 2;
    nh->getParam("points_from_center", pointsFromCenter);
    sensor_msgs::PointCloud2ConstIterator<float> order(*(msg.get()), "x");
    leftRange.range = (order + (centerPointOrder - pointsFromCenter))[2];
    rightRange.range = (order + (centerPointOrder + pointsFromCenter))[2];
}

int main(int argc, char **argv)
{
    float minRange = 0.3;
    float maxRange = 10.0;

    ros::init(argc, argv, "front_ranges_node");
    nh = new ros::NodeHandle();
    nh->getParam("min_range", minRange);
    nh->getParam("max_range", maxRange);
    ros::Subscriber cloudSub = nh->subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, Callback);
    rangePubLeft = nh->advertise<sensor_msgs::Range>("/range_front_left", 1);
    rangePubRight = nh->advertise<sensor_msgs::Range>("/range_front_right", 1);
    rightRange.radiation_type = leftRange.radiation_type = sensor_msgs::Range::INFRARED;
    rightRange.min_range = leftRange.min_range = minRange;
    rightRange.max_range = leftRange.max_range = maxRange;
    leftRange.header.frame_id = "range_front_left";
    rightRange.header.frame_id = "range_front_right";

    ros::Rate rate(30);
    while(ros::ok())
    {
        rate.sleep();
            if(cloudSub.getNumPublishers() > 0)
            {
                leftRange.header.stamp = rightRange.header.stamp = ros::Time::now();
                rangePubLeft.publish(leftRange);
                rangePubRight.publish(rightRange);
            }
        ros::spinOnce();
    }

    delete nh;
    return 0;
}