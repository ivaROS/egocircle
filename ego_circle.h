#ifndef PIPS_DWA_EGO_CIRCLE_H
#define PIPS_DWA_EGO_CIRCLE_H

#include <map>
#include <vector>
#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <deque>
#include <sensor_msgs/LaserScan.h>


namespace ego_circle
{

class EgoCircleIter;


struct EgoCircularPoint
{
  float x,y; // May add 'depth', if needed
  
  EgoCircularPoint()
  {}
  
  EgoCircularPoint(float x, float y) :
    x(x), y(y) //, d(x*x+y*y)
    {}
  
  float getKey() const
  {
    return x*x+y*y;
  }
  
};


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

void applyTransform(EgoCircularPoint& point, SE2Transform transform);

// Begin code copied from http://eliang.blogspot.com/2011/07/gotcha-of-c-map-and-set.html
inline float discretize(float a)
{
  return floorf(a * 100.0f) / 100.0f;
}
struct FloatCmp
{
  bool operator()(float a, float b)
  {
    float aa = discretize(a);
    float bb = discretize(b);
    if (aa == bb)
      return false;
    return aa < bb;
  }
};
// End copied code

struct EgoCircularCell
{
  constexpr static float MAX_DEPTH = 50;  //TODO: Replace this with numeric_limits for type
  //std::vector<float> x,y;
  std::vector<EgoCircularPoint> points_;
  float current_min_ = 0;
  
  typedef std::vector<EgoCircularPoint>::iterator iterator;
    
  void insertPoint(EgoCircularPoint point, bool clearing);
  
  void applyTransform(SE2Transform transform);
  
  iterator begin()
  {
    return points_.begin();
  }
  
  iterator end()
  {
    return points_.end();
  }
  
  void reset();
  
  void printPoints() const;
};

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;



struct EgoCircle
{
  std::vector<EgoCircularCell> cells_;
  
  float max_depth_ = 5;
  float inscribed_radius_ = .18;
  float scale_;
  
  friend class EgoCircleIter;
  typedef EgoCircleIter iterator;
  
  
  EgoCircle(int size) :  scale_(size/(2*std::acos(-1)))
  {
    cells_.resize(size);
  }
  
  iterator begin();
  iterator end();
  
  //TODO: Do I need to perform rounding or is that part of casting?
  int getIndex(EgoCircularPoint point)
  {
    float angle = std::atan2(point.y,point.x);
    
    int ind = angle * scale_ + cells_.size() / 2;
    return ind;
  }
  
  void insertPoint(std::vector<EgoCircularCell>& cells, EgoCircularPoint point, bool clearing);
  
  //TODO: if clearing, should clear first, then add all the points, otherwise risk clearing new points too
  void insertPoints(std::vector<EgoCircularCell>& cells, std::vector<EgoCircularPoint> points, bool clearing);
  
  void insertPoints(std::vector<EgoCircularPoint> points, bool clearing);
  
  void insertPoints(EgoCircle& other);
  
  void insertPoints(PCLPointCloud points);
  
  void updateCells();

  void applyTransform(geometry_msgs::TransformStamped transform);
  
  std::vector<float> getDepths();
  
  std::vector<EgoCircularPoint> getNearestPoints();

  int getN(float depth);

  std::vector<float> inflateDepths(const std::vector<float>& depths);
  
  void printPoints() const;
  
  void countPoints() const;
  
  void reset();
  
};


void swap(EgoCircle& lhs, EgoCircle& rhs)
{
  std::swap(lhs.cells_, rhs.cells_);
  //Note: Currently, the rest of the fields are fixed, so no need to swap them
}



class EgoCircleIter
{
private:
  EgoCircle& circle_;
  int cell_id_;
  EgoCircularCell::iterator point_it_;
  
public:
  
  EgoCircleIter(EgoCircle& circle, int cell_id, EgoCircularCell::iterator point_it) :
    circle_(circle),
    cell_id_(cell_id),
    point_it_(point_it)
  {}
  
  int getCell() { return cell_id_; }
  
  EgoCircleIter & operator++() 
  {
    ++point_it_;
    
    bool keep_going = true;
    while(keep_going)
    {
      if(point_it_ == circle_.cells_[cell_id_].points_.end())
      {
        if(cell_id_ < circle_.cells_.size() - 1)
        {
          cell_id_++;
          point_it_ = circle_.cells_[cell_id_].points_.begin();
        }
        else
        {
          keep_going = false;
        }
      }
      else
      {
        keep_going = false;
      }
    }
    return *this; 
  }
  
  EgoCircleIter operator++(int)
  {
    EgoCircleIter clone(*this);
    
    ++point_it_;
    
    bool keep_going = true;
    while(keep_going)
    {
      if(point_it_ == circle_.cells_[cell_id_].points_.end())
      {
        if(cell_id_ < circle_.cells_.size() - 1)
        {
          cell_id_++;
          point_it_ = circle_.cells_[cell_id_].points_.begin();
        }
        else
        {
          keep_going = false;
        }
      }
      else
      {
        keep_going = false;
      }
    }
    
    return clone;
  }
  
  EgoCircularPoint & operator*() { return (*point_it_); }
  
  bool operator!=(EgoCircleIter other)
  {
    return cell_id_ != other.cell_id_ || point_it_ != other.point_it_;
  }
  
};

std_msgs::ColorRGBA getConfidenceColor(float confidence, float max_conf);

class EgoCircleROS
{
  

  
  typedef tf2_ros::MessageFilter<sensor_msgs::PointCloud2> TF_Filter;
private:
  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  message_filters::Subscriber<nav_msgs::Odometry> odom_subscriber_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_subscriber_;
  
  std::shared_ptr<TF_Filter> tf_filter_;
  ros::Publisher vis_pub_, scan_pub_, inflated_scan_pub_;
  
  std_msgs::Header old_header_;
  
  std::string odom_frame_id_ = "odom";
  std::string base_frame_id_ = "base_footprint";
  
public:
  EgoCircle ego_circle_,old_ego_circle_;
  
public:
  
  EgoCircleROS(ros::NodeHandle nh = ros::NodeHandle(), ros::NodeHandle pnh=ros::NodeHandle("~"));
  
  bool init();
  
  void publishPoints();
  
private:
  void odomCB(const nav_msgs::Odometry::ConstPtr& odom_msg);
  
  void pointcloudCB(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);
  
  visualization_msgs::Marker getVisualizationMsg();
  
  //visualization_msgs::Marker getVisualizationMsgNearest();
  
  void publishDepthScans();
  
  bool update(std_msgs::Header new_header);
  
  bool update(std_msgs::Header old_header, std_msgs::Header new_header);
  
  void prep();
};




}

#endif //PIPS_DWA_EGO_CIRCLE_H
