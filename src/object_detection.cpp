#include <ros/ros.h>
#include "DetectionAlgorithms.hpp"

ros::NodeHandle *n;
ros::Publisher pub;

sensor_msgs::PointCloud2 centerPoints;
sensor_msgs::PointCloud2Modifier *modifier;

std::string subscribeTopic;
std::string publishTopic;

Position GetInitialPosition(PointCloud2Array<float> &src)
{
    int width = src.GetWidth();
    int height = src.GetHeight();
    for(int i = 0; i < height; i++)
    {
        for(int j = 0; j < width - 1; j++)
        {
            if(!std::isnan(src(i, j)[2])) return Position(i, j);
        }
    }
    return Position(height - 1, width - 1);
}

bool SwapPointsToGravityCenter(PointCloud2Array<float> &cloud,
                                std::map<int, std::vector<int>> &points,
                                int minObjectSize, std::vector<float> &center)
{
    center[0] = center[1] = center[2] = 0.0f;
    int count = 0;
    for(auto point : points)
    {
        int row = point.first;
        for(int column : point.second)
        {
            float z = cloud(row, column)[2];
            if(!std::isnan(z))
            {
                count++;
                center[0] += cloud(row, column)[0];
                center[1] += cloud(row, column)[1];
                center[2] += z;
                cloud(row, column)[2] = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    if(count < minObjectSize) return false;
    center[0] /= count;
    center[1] /= count;
    center[2] /= count;
    return true;
}

void Callback(sensor_msgs::PointCloud2ConstPtr msg)
{
    int minObjectSize = 20;
    n->getParam("min_object_size", minObjectSize);
    //Collect centers of gravity of every object
    PointCloud2Array<float> cloud(*(msg.get()));
    //PointCloud2Array<float> write(*(msg.get()));

    const Position endPos(msg->height - 1, msg->width - 1);
    Position initPos = GetInitialPosition(cloud);
    std::map<int, std::vector<int>> objectPoints;
    std::vector<std::vector<float>> centers;
    std::vector<float> center(3, 0.0f);
    while(initPos != endPos)
    {
        DetectObject(cloud, objectPoints, initPos);
        if(SwapPointsToGravityCenter(cloud, objectPoints, minObjectSize, center))
        {
            centers.emplace_back(center);
        }
        //read = write;
        initPos = GetInitialPosition(cloud);
    }
    //Pack centers of gravity to PointCloud2 message
    modifier->clear();
    modifier->resize(centers.size());
    sensor_msgs::PointCloud2Iterator<float> center_iter(centerPoints, "x");
    for(int i = 0; i < centers.size(); i++)
    {
        (center_iter + i)[0] = centers[i][0];
        (center_iter + i)[1] = centers[i][1];
        (center_iter + i)[2] = centers[i][2];
        ROS_INFO_STREAM("Point " << i << ": x: " << centers[i][0] << "y: " << centers[i][1] << "z: " << centers[i][2]);
    }
    //Publish
    centerPoints.header.stamp = ros::Time::now();
    if(pub.getNumSubscribers() > 0 && centers.size() > 0) pub.publish(centerPoints);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detection_node");
    n = new ros::NodeHandle();
    n->getParam("pointcloud_topic", subscribeTopic);
    n->getParam("gravity_centers_topic", publishTopic);
    ros::Subscriber sub = n->subscribe<sensor_msgs::PointCloud2>(subscribeTopic, 1, Callback);
    pub = n->advertise<sensor_msgs::PointCloud2>(publishTopic, 1);

    modifier = new sensor_msgs::PointCloud2Modifier(centerPoints);
    modifier->setPointCloud2FieldsByString(1, "xyz");
    centerPoints.header.frame_id = "camera_depth_optical_frame";

    ros::spin();

    delete n;
    delete modifier;
}