#include "DetectionAlgorithms.hpp"

void PickContourPoints(Position initialPoint, PointCloud2Array<float> &src, std::map<int,
                        std::vector<int>> &contourPoints)
{
    int rotation = 0;
    Position pos = initialPoint;
    do
    {
        std::vector<int> &checkRow = contourPoints[pos.GetRow()];
        if(std::find(checkRow.begin(), checkRow.end(), pos.GetColumn()) == checkRow.end())
        {
            checkRow.emplace_back(pos.GetColumn());
        }
        Position upperPos = pos.GetUpperPosition(rotation);
        if(std::isnan(src(upperPos)[2]) || !(src(upperPos) != src(upperPos).end()))
        {
            for(int rot = 0; rot < 4; rot++, rotation++, rotation %= 4)
            {
                Position rightPos = pos.GetRightPosition(rotation);
                if(!std::isnan(src(rightPos)[2]) && src(rightPos) != src(rightPos).end())
                {
                    pos = rightPos;
                    break;
                }
            }
        }
        else
        {
            if(rotation == 0)
            {
                rotation = 3;
            } else rotation--;
            pos = pos.GetRightPosition(rotation);
        }
    } while (pos != initialPoint);
}

void DetectObject(PointCloud2Array<float> &src,
                    std::map<int, std::vector<int>> &objectPoints, Position initPos)
{
    int width = src.GetWidth();
    int height = src.GetHeight();
    objectPoints.clear();
    if(initPos != Position(width - 1, height - 1))
    {
        std::map<int, std::vector<int>> contourPoints;
        PickContourPoints(initPos, src, contourPoints);
        objectPoints = contourPoints;
        //Pick all points inside contour
        for(auto &pair : contourPoints)
        {
            std::vector<int> &columns = pair.second;
            std::sort(columns.begin(), columns.end());
            for(int i = 0; i < columns.size() - 1; i++)
            {
                if(columns[i + 1] - columns[i] > 1 && !std::isnan(src(pair.first, columns[i] + 1)[2]))
                {
                    for(int column = columns[i] + 1; column < columns[i + 1]; column++)
                    {
                        objectPoints[pair.first].emplace_back(column);
                    }
                    i++;
                }
            }
        }
    }
}