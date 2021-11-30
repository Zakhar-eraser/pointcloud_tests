#ifndef POINTCLOUD2_ARRAY
#define POINTCLOUD2_ARRAY
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class Position
{
    private:
        int row = 0;
        int column = 0;
    public:
Position(int row, int column)
{
    this->row = row;
    this->column = column;
}
Position GetUpperPosition(int rotation)
{
    return Position(this->row + (rotation - 1) * (1 - rotation % 2),
                        this->column + (2 - rotation) * (rotation % 2));
}
Position GetRightPosition(int rotation)
{
    return Position(this->row + (2 - rotation) * (rotation % 2),
                        this->column + (1 - rotation) * (1 - rotation % 2));
}
int GetRow() const
{
    return this->row;
}
int GetColumn() const
{
    return this->column;
}
Position& operator=(const Position &point)
{
    this->row = point.row;
    this->column = point.column;
    return *this;
}
bool operator==(const Position &point) const
{
    return this->row == point.row && this->column == point.column;
}
bool operator!=(const Position &point) const
{
    return this->row != point.row || this->column != point.column;
}

};

template <typename T>
class PointCloud2Array
{
    private:
        sensor_msgs::PointCloud2 message;

        bool IsInFrame(int row, int column)
        {
            return row >= 0 && row < message.height && column >= 0 && column < message.width; 
        }
    public:
        PointCloud2Array(sensor_msgs::PointCloud2 message)
{
    this->message = message;
}

 int GetWidth()
{
    return this->message.width;
}
 int GetHeight()
{
    return this->message.height;
}
 sensor_msgs::PointCloud2Iterator<T> operator()(const Position &position)
{
    int row = position.GetRow();
    int column = position.GetColumn();
    sensor_msgs::PointCloud2Iterator<T> iter(message, "x");
    if(IsInFrame(row, column))
    {
        return iter + (message.width * row + column);
    } else return iter.end();
}
 sensor_msgs::PointCloud2Iterator<T> operator()(int row, int column)
{
    sensor_msgs::PointCloud2Iterator<T> iter(message, "x");
    if(IsInFrame(row, column))
    {
        return iter + (message.width * row + column);
    } else return iter.end();
}
 PointCloud2Array<T> &operator=(PointCloud2Array<T> &array)
{
    this->message = array.message;
    return *this;
}
};
#endif