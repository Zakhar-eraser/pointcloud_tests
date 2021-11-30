#ifndef DETECTION_ALGORITHMS
#define DETECTION_ALGORITHMS
#include "PointCloud2Array.hpp"

void PickContourPoints(Position initialPoint, PointCloud2Array<float> &src, std::map<int,
                        std::vector<int>> &contourPoints);

void DetectObject(PointCloud2Array<float> &src,
                    std::map<int, std::vector<int>> &objectPoints, Position initPos);
#endif