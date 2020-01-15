#ifndef EGOCIRCLE_EGO_CIRCLE_H
#define EGOCIRCLE_EGO_CIRCLE_H

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
#include <dynamic_reconfigure/server.h>

#include <egocircle/egocircleConfig.h>

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


struct PolarPoint
{
  float r, theta;
  
  PolarPoint() :
    r(0),
    theta(0)
  {}
  
  PolarPoint(float r, float theta) :
    r(r),
    theta(theta)
  {}
  
  PolarPoint(const EgoCircularPoint& pnt) :
    r(std::sqrt(pnt.x*pnt.x + pnt.y*pnt.y)),
    theta(std::atan2(pnt.y,pnt.x))
  {}
  
  operator EgoCircularPoint()
  {
    float x = std::cos(theta)*r;
    float y = std::sin(theta)*r;
    EgoCircularPoint point(x,y);
    return point;
  }
};

inline
float distance_sq(const EgoCircularPoint& p1, const EgoCircularPoint& p2)
{
  float xd = p1.x-p2.x;
  float yd = p1.y-p2.y;
  float d = xd*xd + yd*yd;
  return d;
}

struct SE2Transform
{
  float r0,r1,r3,r4,t0,t1;
  
  SE2Transform()
  {
  }
  
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

inline
void applyTransform(EgoCircularPoint& point, SE2Transform transform)
{
  float x = point.x;
  float y = point.y;
  
  point.x = transform.r0 * x + transform.r1 * y + transform.t0;
  point.y = transform.r3 * x + transform.r4 * y + transform.t1;
}

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
  float MAX_DEPTH_SQ;
  
  //std::vector<float> x,y;
  std::vector<EgoCircularPoint> points_;
  float current_min_;
  bool cleared_=false;
  bool border_=false;
  
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
  
  void reset(float max_depth_sq);
    
  void printPoints() const;
};

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;


struct EgoCircleIndexer
{
  unsigned int size;
  float scale;
  
  EgoCircleIndexer() {}
  
  EgoCircleIndexer(int size):
    size(size),
    scale(size/(2*std::acos(-1)))
  {}
    
  //NOTE: All that matters is that cells are evenly mapped to a circle, so no rounding is necessary here
  unsigned int getIndex(float angle) const
  {
    return ((unsigned int)(angle * scale + size / 2)) % size;
  }
  
  unsigned int getIndex(PolarPoint point) const
  {
    return getIndex(point.theta);
  }
  
  unsigned int getIndex(EgoCircularPoint point) const
  {
    return getIndex(PolarPoint(point));
  }
  
  void getIndex(float angle, unsigned int& index) const
  {
    index = getIndex(angle);
  }
  
  template <typename T>
  void getIndex(float angle, T& index) const
  {
    index = fmodf(angle * scale + size / 2, size);
  }
  
  template <typename S, typename T>
  void getIndex(S angle, T& index) const
  {
    getIndex(toAngle(angle), index);
  }
  
  float toAngle(float angle) const
  {
    return angle;
  }
  
  float toAngle(PolarPoint point) const
  {
    return point.theta;
  }
  
  float toAngle(EgoCircularPoint point) const
  {
    return toAngle(PolarPoint(point));
  }
  
};

struct EgoCircleWidthConverter
{
  float scale;
  
  EgoCircleWidthConverter() {}
  
  EgoCircleWidthConverter(EgoCircleIndexer idx, float inscribed_radius) : scale(inscribed_radius / (2 * std::sin(1/(idx.scale *2))))
  {
  }
  
  int getN(float depth) const
  {
    int n = std::ceil(scale/depth);

    return n;
  }
};


struct EgoCircle
{
  std::vector<EgoCircularCell> cells_;
  
  float max_depth_;
  float inscribed_radius_ = .18;
  EgoCircleIndexer indexer_;
  EgoCircleWidthConverter converter_;
  
  
  friend class EgoCircleIter;
  typedef EgoCircleIter iterator;
  
  bool clearing_enabled_;
  
  EgoCircle(unsigned int size) :
    indexer_(size),
    converter_(indexer_, inscribed_radius_)
  {
    cells_.resize(size);
  }
  
  iterator begin();
  iterator end();
  
  template <typename T>
  unsigned int getIndex(T point)
  {
    return indexer_.getIndex(point);
  }
  
  template <typename T>
  void insertPoint(std::vector<EgoCircularCell>& cells, T point, bool clearing)
  {
    unsigned int ind = getIndex(point);
    cells[ind].insertPoint(point, clearing);
  }
  
  void setMaxDepth(float max_depth, float max_depth_sq);
  
  //TODO: if clearing, should clear first, then add all the points, otherwise risk clearing new points too
  void insertPoints(std::vector<EgoCircularCell>& cells, std::vector<EgoCircularPoint> points, bool clearing);
  
  void insertPoints(std::vector<EgoCircularPoint> points, bool clearing);
  
  void insertPoints(EgoCircle& other);
  
  void insertPoints(PCLPointCloud points);
  
  void insertPoints(const sensor_msgs::LaserScan& scan);
  
  //void updateCells();

  void applyTransform(geometry_msgs::TransformStamped transform);
  
  std::vector<float> getDepths();
  
  std::vector<EgoCircularPoint> getNearestPoints();

  int getN(float depth);

  std::vector<float> inflateDepths(const std::vector<float>& depths);
  
  void printPoints() const;
  
  void countPoints() const;
  
  void reset();
  
  void reset(float max_depth);
  
};

inline
void swap(EgoCircle& lhs, EgoCircle& rhs)
{
  std::swap(lhs.cells_, rhs.cells_);
  std::swap(lhs.max_depth_, rhs.max_depth_);
  //Note: Currently, the rest of the fields are fixed, so no need to swap them
}



class EgoCircleIter
{
private:
  EgoCircle& circle_;
  unsigned int cell_id_;
  EgoCircularCell::iterator point_it_;
  
public:
  
  EgoCircleIter(EgoCircle& circle, unsigned int cell_id, EgoCircularCell::iterator point_it) :
    circle_(circle),
    cell_id_(cell_id),
    point_it_(point_it)
  {}
  
  unsigned int getCell() { return cell_id_; }
  
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




class LaserScanIter;

class LaserScanWrapper
{
  friend LaserScanIter;
  
  const sensor_msgs::LaserScan& scan_;
  
  
public:
  LaserScanWrapper(const sensor_msgs::LaserScan& scan) :
    scan_(scan)
  {
  }
  
  LaserScanIter begin();
  
  LaserScanIter end();
  
};



class LaserScanIter
{
private:
  float current_angle_;
  const float angle_inc_;
  
  typedef decltype(sensor_msgs::LaserScan::ranges)::const_iterator Iterator;
  Iterator it_;
  
public:
  
  LaserScanIter(Iterator it, float start_angle, float angle_inc) :
    current_angle_(start_angle),
    angle_inc_(angle_inc),
    it_(it)
  {}
    
  LaserScanIter & operator++() 
  {
    ++it_;
    current_angle_+=angle_inc_;
    
    return *this; 
  }
  
  LaserScanIter operator++(int)
  {
    LaserScanIter clone(*this);
    
    ++it_;
    current_angle_+=angle_inc_;

    return clone;
  }
  
  PolarPoint operator*() 
  {
    //float r = *it_;
    
    PolarPoint pnt(*it_, current_angle_);
    return pnt;
  }
  
  bool operator!=(LaserScanIter other)
  {
    return it_ != other.it_;// || current_angle_ != other.current_angle_;
  }
  
};

inline
LaserScanIter LaserScanWrapper::begin()
{
  return LaserScanIter(scan_.ranges.begin(),scan_.angle_min,scan_.angle_increment);
}

inline
LaserScanIter LaserScanWrapper::end()
{
  return LaserScanIter(scan_.ranges.end(),scan_.angle_max,scan_.angle_increment);
}

std_msgs::ColorRGBA getConfidenceColor(float confidence, float max_conf);

class EgoCircleROS
{
  

  
  typedef tf2_ros::MessageFilter<sensor_msgs::PointCloud2> PC_TF_Filter;
  typedef tf2_ros::MessageFilter<sensor_msgs::LaserScan> LS_TF_Filter;
  
private:
  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  message_filters::Subscriber<nav_msgs::Odometry> odom_subscriber_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_subscriber_;
  message_filters::Subscriber<sensor_msgs::LaserScan> ls_subscriber_;
  
  std::shared_ptr<PC_TF_Filter> pc_tf_filter_;
  std::shared_ptr<LS_TF_Filter> ls_tf_filter_;
  
  ros::Publisher vis_pub_, scan_pub_, inflated_scan_pub_;
  
  std_msgs::Header old_header_;
  
  std::string odom_frame_id_;
  std::string base_frame_id_;
  
  std::shared_ptr<dynamic_reconfigure::Server<egocircle::egocircleConfig> > dsrv_;
  egocircle::egocircleConfig cur_config_;
  
  
public:
  EgoCircle ego_circle_,old_ego_circle_;
  static constexpr float OFFSET=1;
  
public:
  
  EgoCircleROS(ros::NodeHandle nh = ros::NodeHandle(), ros::NodeHandle pnh=ros::NodeHandle("~"));
  
  bool init();
  
  void publishPoints();
  
private:
  void reconfigureCB(const egocircle::egocircleConfig& config, uint32_t level);
  
  void odomCB(const nav_msgs::Odometry::ConstPtr& odom_msg);
  
  void pointcloudCB(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);
  
  void laserscanCB(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
  
  visualization_msgs::Marker getVisualizationMsg();
  
  //visualization_msgs::Marker getVisualizationMsgNearest();
  
  void publishDepthScans();
  
  bool update(std_msgs::Header new_header);
  
  bool update(std_msgs::Header old_header, std_msgs::Header new_header);
  
  void prep();
};




}

#endif //EGOCIRCLE_EGO_CIRCLE_H
