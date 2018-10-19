
#include <map>
#include <vector>
#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <ros/ros.h>

struct EgoCircularPoint
{
  float x,y,d;
  
  EgoCircularPoint()
  {}
  
  EgoCircularPoint(float x, float y) :
    x(x), y(y), d(x*x+y*y)
    {}
  
  float getKey() const
  {
    return x*x+y*y;
  }
  
//   bool operator<(const EgoCircularPoint& other) const
//   {
//     return getKey() < other.getKey();
//   }
  
};

namespace std
{
  template<> struct less<EgoCircularPoint>
  {
    bool operator() (const EgoCircularPoint& lhs, const EgoCircularPoint& rhs) const
    {
      return lhs.getKey() < rhs.getKey();
    }
  };
}


struct SE2Transform
{
  float r0,r1,r3,r4,t0,t1;
  
  SE2Transform(geometry_msgs::TransformStamped trans)
  {
    tf::Quaternion rotationQuaternion = tf::Quaternion(trans.transform.rotation.x,
                                                       trans.transform.rotation.y,
                                                       trans.transform.rotation.z,
                                                       trans.transform.rotation.w);
    
    
    tf::Matrix3x3 tempRotationMatrix = tf::Matrix3x3(rotationQuaternion);
    
    r0 = (float) tempRotationMatrix[0].getX();
    r1 = (float) tempRotationMatrix[0].getY();
    r3 = (float) tempRotationMatrix[1].getX();
    r4 = (float) tempRotationMatrix[1].getY();
    
    t0 = trans.transform.translation.x;
    t1 = trans.transform.translation.y;  
  }
};

void applyTransform(EgoCircularPoint& point, SE2Transform transform)
{
  float x = point.x;
  float y = point.y;
  
  point.x = transform.r0 * x + transform.r1 * y + transform.t0;
  point.y = transform.r3 * x + transform.r4 * y + transform.t1;
}

struct EgoCircularCell
{
  //std::vector<float> x,y;
  std::map<EgoCircularPoint, EgoCircularPoint> points_;
  
  void insertPoint(EgoCircularPoint point)
  {
    auto upper = points_.upper_bound(point);
    points_.erase(points_.begin(),upper);
    points_[point] = point;
  }
  
  void applyTransform(SE2Transform transform)
  {
    for(auto& point : points_)
    {
      ::applyTransform(point.second, transform);
    }
  }
  
  void printPoints() const
  {
    for(const auto& point : points_)
    {
      EgoCircularPoint p = point.second;
      ROS_INFO_STREAM("\t\t[" << p.x << "," << p.y << "]");
    }
  }
};



struct EgoCircle
{
  std::vector<EgoCircularCell> cells_;
  
  EgoCircle(int size)
  {
    cells_.resize(size);
  }
  
  int getIndex(EgoCircularPoint point)
  {
    float angle = std::atan2(point.y,point.x);
    float scale = cells_.size()/(2*std::acos(-1));
    
    int ind = angle * scale + cells_.size() / 2;
    return ind;
  }
  
  void insertPoints(std::vector<EgoCircularPoint> points)
  {
    for(auto point : points)
    {
      int ind = getIndex(point);
      cells_[ind].insertPoint(point);
    }
  }

  void applyTransform(geometry_msgs::TransformStamped transform)
  {
    SE2Transform trans(transform);
    
    for(EgoCircularCell& cell : cells_)
    {
      cell.applyTransform(trans);
    }
  }
  
  void printPoints() const
  {
    int id = 0;
    for(const EgoCircularCell& cell : cells_)
    {
      ROS_INFO_STREAM("\tCell #" << id << ":");
      cell.printPoints();
      id++;
    }
  }
  
};


std::vector<EgoCircularPoint> makePoints(int num)
{
  ROS_INFO_STREAM("Making " << num << " points");
  std::vector<EgoCircularPoint> points;
  
  for(int i = 0; i < num; i++)
  {
    float angle = 6.0 /num * i;
    float x = std::sin(angle)*i;
    float y = std::cos(angle)*i;
    EgoCircularPoint point(x,y);
    points.push_back(point);
  }
  return points;
}

int main()
{
  EgoCircle circle(512);
  circle.insertPoints(makePoints(200));
  circle.printPoints();
  
  geometry_msgs::TransformStamped trans;
  trans.transform.translation.x = 1;
  trans.transform.rotation.w = 1;
  circle.applyTransform(trans);
  circle.printPoints();
}
