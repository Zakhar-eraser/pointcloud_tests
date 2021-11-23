#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

ros::NodeHandle *n;
ros::Publisher pub;

sensor_msgs::PointCloud2 centerPoints;
sensor_msgs::PointCloud2Modifier *modifier;

std::string subscribeTopic;
std::string publishTopic;

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
    PointCloud2Array<float> read(*(msg.get()));
    PointCloud2Array<float> write(*(msg.get()));

    const Position endPos(msg->height - 1, msg->width - 1);
    Position initPos = GetInitialPosition(read);
    std::map<int, std::vector<int>> objectPoints;
    std::vector<std::vector<float>> centers;
    std::vector<float> center(3, 0.0f);
    while(initPos != endPos)
    {
        DetectObject(read, objectPoints, initPos);
        if(SwapPointsToGravityCenter(write, objectPoints, minObjectSize, center))
        {
            centers.emplace_back(center);
        }
        read = write;
        initPos = GetInitialPosition(read);
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