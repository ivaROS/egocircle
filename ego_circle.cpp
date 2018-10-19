
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
  
  void removeCloserPoints(EgoCircularPoint point)
  {
    auto upper = points_.upper_bound(point);
    points_.erase(points_.begin(),upper);
  }
  
  void insertPoint(EgoCircularPoint point)
  {
    removeCloserPoints(point);
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
    float scale = cells_.size()/(2*std::acos(-1));  //Should make this a member variable
    
    int ind = angle * scale + cells_.size() / 2;
    return ind;
  }
  
  void insertPoint(std::vector<EgoCircularCell>& cells, EgoCircularPoint point)
  {
    int ind = getIndex(point);
    cells[ind].insertPoint(point);
  }
  
  void insertPoints(std::vector<EgoCircularCell>& cells, std::vector<EgoCircularPoint> points)
  {
    for(auto point : points)
    {
      insertPoint(cells, point);
    }
  }
  
  void insertPoints(std::vector<EgoCircularPoint> points)
  {
    insertPoints(cells_, points);
  }
  
  void updateCells()
  {
    std::vector<EgoCircularCell> cells(cells_.size());
    
    for(auto cell : cells_)
    {
      for(auto point : cell.points_)
      {
        insertPoint(cells, point.second);
      }
    }
    
    cells_ = cells; //TODO: hold onto current and previous cell vectors; clear each cell after copying points from it, and std::swap vectors here
    
  }

  void applyTransform(geometry_msgs::TransformStamped transform)
  {
    ros::WallTime start = ros::WallTime::now();
    SE2Transform trans(transform);
    
    for(EgoCircularCell& cell : cells_)
    {
      cell.applyTransform(trans);
    }
    updateCells();
    
    ros::WallTime end = ros::WallTime::now();
    
    ROS_INFO_STREAM_NAMED("timing", "Applying transform took " <<  (end - start).toSec() * 1e3 << "ms");
    
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

class EgoCircleROS
{
  typedef tf2_ros::MessageFilter<nav_msgs::Odometry> TF_Filter;
private:
  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  message_filters::Subscriber<nav_msgs::Odometry> odom_subscriber_;
  std::shared_ptr<TF_Filter> tf_filter_;
  std_msgs::Header old_header_;
  
  std::string odom_frame_id_ = "odom";
  
  EgoCircle ego_circle_;
  
public:
  
  EgoCircleROS(ros::NodeHandle nh = ros::NodeHandle(), ros::NodeHandle pnh=ros::NodeHandle("~")):
    nh_(nh),
    pnh_(pnh),
    tf_listener_(tf_buffer_),
    ego_circle_(512)
  {
    
  }
  
  bool init()
  {
    int odom_queue_size = 5;
    std::string odom_topic = "odom";
    
    tf_filter_ = std::make_shared<TF_Filter>(odom_subscriber_, tf_buffer_, odom_frame_id_, odom_queue_size, nh_);
    tf_filter_->registerCallback(boost::bind(&EgoCircleROS::odomCB, this, _1));
    tf_filter_->setTolerance(ros::Duration(0.01));
    
    ego_circle_ = EgoCircle(512);
    
    odom_subscriber_.subscribe(nh_, odom_topic, odom_queue_size);
    
  }
  
private:
  void odomCB(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
    if(old_header_.stamp > ros::Time(0))
    {
      update(old_header_, odom_msg->header);
    }
    
    old_header_ = odom_msg->header;
  }
  
  bool update(std_msgs::Header old_header, std_msgs::Header new_header)
  {
    try
    {
      ROS_DEBUG("Getting Transformation details");
      geometry_msgs::TransformStamped trans = tf_buffer_.lookupTransform(new_header.frame_id, new_header.stamp,
                                                                      old_header.frame_id, old_header.stamp,
                                                                      odom_frame_id_); 
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
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
  circle.insertPoints(makePoints(500));
  circle.printPoints();
  
  geometry_msgs::TransformStamped trans;
  trans.transform.translation.x = 1;
  trans.transform.rotation.w = 1;
  circle.applyTransform(trans);
  circle.printPoints();
}
