/**
 * @file print_angles.cpp
 * @author Zakhar Anikin (79922187361@yandex.ru)
 * @brief Gets PointCloud2 from mean_distances_node and log this distances and calculated angles
 * @version 0.1
 * @date 2021-11-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Range.h>

std::pair<float, float> Transform(std::pair<float, float> point, float rotation)
{
    std::pair<float, float> newPoint;
    float s = sin(rotation);
    float c = cos(rotation);
    newPoint.first = point.first * c - point.second * s;
    newPoint.second = point.second * c + point.first * s;
    return newPoint;
}

class Segment
{
    private:
        std::pair<float, float> start;
        std::pair<float, float> end;
        float k;
        float b;
    public:
        Segment(std::pair<float, float> start, std::pair<float, float> end)
        {
            this->start = start;
            this->end = end;
            k = (end.second - start.second) / (end.first - start.first);
            b = start.second - k * start.first;
        }
        bool IsInRange(std::pair<float, float> pos)
        {

        }
};



//Estimated absolute position
std::pair<float, float> GetPosition(std::vector<Segment> map, std::pair<float, float> initPos,
                                    float angle, float distance)
{
    
}

void Callback(sensor_msgs::PointCloud2ConstPtr msg)
{
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "print_angles_node");
    
}