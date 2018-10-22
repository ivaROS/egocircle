
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
  typedef std::map<EgoCircularPoint, EgoCircularPoint>::iterator iterator;
  
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
  
  iterator begin()
  {
    return points_.begin();
  }
  
  iterator end()
  {
    return points_.end();
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
  
  friend class EgoCircleIter;
  typedef EgoCircleIter iterator;
  
  
  EgoCircle(int size)
  {
    cells_.resize(size);
  }
  
  iterator begin();
  iterator end();
  
  
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
    
    int num_points = 0;
    
    for(auto cell : cells_)
    {
      for(auto point : cell)
      {
        insertPoint(cells, point.second);
        num_points++;
      }
    }
    
    ROS_INFO_STREAM("Updated " << num_points << " points");
    
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
  
  void countPoints() const
  {
    int num_points = 0;
    
    for(auto cell : cells_)
    {
      for(auto point : cell)
      {
        num_points++;
      }
    }
    
    ROS_INFO_STREAM("Counted " << num_points << " points");
    
  }
  
};


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
  
  EgoCircularPoint & operator*() { return (*point_it_).second; }
  
  bool operator!=(EgoCircleIter other)
  {
    return cell_id_ != other.cell_id_ || point_it_ != other.point_it_;
  }
  
};

EgoCircle::iterator EgoCircle::begin()
{
  for(int i = 0; i < cells_.size(); i++)
  {
    if(cells_[i].points_.size() > 0)
    {
      return EgoCircle::iterator(*this,i,cells_[i].points_.begin());
    }
  }
  return end();
}

EgoCircle::iterator EgoCircle::end()
{
  return EgoCircle::iterator(*this,cells_.size()-1,cells_.back().points_.end());
}




std_msgs::ColorRGBA getConfidenceColor(float confidence, float max_conf)
{
  
  confidence = std::min(confidence, max_conf);
  
  //https://stackoverflow.com/questions/12875486/what-is-the-algorithm-to-create-colors-for-a-heatmap
  float fH = (1.0 - (confidence / max_conf)) * 240;
  float fS = 1.0;
  float fV = .5;
  
  //https://gist.github.com/fairlight1337/4935ae72bcbcc1ba5c72
  float fC = fV * fS; // Chroma
  float fHPrime = fmod(fH / 60.0, 6);
  float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
  float fM = fV - fC;
  
  float fB, fR, fG;
  
  if(0 <= fHPrime && fHPrime < 1) {
    fR = fC;
    fG = fX;
    fB = 0;
  } else if(1 <= fHPrime && fHPrime < 2) {
    fR = fX;
    fG = fC;
    fB = 0;
  } else if(2 <= fHPrime && fHPrime < 3) {
    fR = 0;
    fG = fC;
    fB = fX;
  } else if(3 <= fHPrime && fHPrime < 4) {
    fR = 0;
    fG = fX;
    fB = fC;
  } else if(4 <= fHPrime && fHPrime < 5) {
    fR = fX;
    fG = 0;
    fB = fC;
  } else if(5 <= fHPrime && fHPrime < 6) {
    fR = fC;
    fG = 0;
    fB = fX;
  } else {
    fR = 0;
    fG = 0;
    fB = 0;
  }
  
  fR += fM;
  fG += fM;
  fB += fM;
  
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = fR;
  color.b = fB;
  color.g = fG;
  
  return color;
}



class EgoCircleROS
{
  typedef tf2_ros::MessageFilter<nav_msgs::Odometry> TF_Filter;
private:
  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  message_filters::Subscriber<nav_msgs::Odometry> odom_subscriber_;
  std::shared_ptr<TF_Filter> tf_filter_;
  ros::Publisher vis_pub_;
  
  std_msgs::Header old_header_;
  
  std::string odom_frame_id_ = "odom";
  
public:
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
    
    //tf_filter_ = std::make_shared<TF_Filter>(odom_subscriber_, tf_buffer_, odom_frame_id_, odom_queue_size, nh_); //NOTE: this is the correct form for any message but an odometry message
    tf_filter_ = std::make_shared<TF_Filter>(odom_subscriber_, tf_buffer_, "base_footprint", odom_queue_size, nh_);
    tf_filter_->registerCallback(boost::bind(&EgoCircleROS::odomCB, this, _1));
    tf_filter_->setTolerance(ros::Duration(0.01));
    
    ego_circle_ = EgoCircle(512);
    
    odom_subscriber_.subscribe(nh_, odom_topic, odom_queue_size);
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>("vis",5);
    
  }
  
  void publishPoints()
  {
    visualization_msgs::Marker msg = getVisualizationMsg();
    vis_pub_.publish(msg);
  }
  
private:
  void odomCB(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
    std_msgs::Header header = odom_msg->header;
    header.frame_id = odom_msg->child_frame_id;
    
    update(header);
  }
  
  visualization_msgs::Marker getVisualizationMsg()
  {
    float scale = .02;
    
    visualization_msgs::Marker marker;
    //marker.header.frame_id = odom_frame_id_;
    //marker.header.stamp = old_header_.stamp;
    marker.header = old_header_;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "all";
//     marker.color.a = .75;
//     marker.color.g = 1;
//     marker.color.r = 1;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    
    int num_cells = ego_circle_.cells_.size();
    
    
    for( EgoCircle::iterator it = ego_circle_.begin(); it != ego_circle_.end(); ++it)
    {
      EgoCircularPoint point = *it;
      geometry_msgs::Point point_msg;
      point_msg.x = point.x;
      point_msg.y = point.y;
      point_msg.z = 0;
      
      int cell_id = it.getCell();
      std_msgs::ColorRGBA color = getConfidenceColor(cell_id, num_cells);
      
      marker.points.push_back(point_msg);
      marker.colors.push_back(color);
    }
    
    //ROS_INFO_STREAM_THROTTLE(1, "Total # points= " << marker.points.size());
    ROS_INFO_STREAM("Publishing " << marker.points.size() << " points");
    
    return marker;
  }
  
  bool update(std_msgs::Header new_header)
  {
    bool success = true;
    
    if(old_header_.stamp > ros::Time(0))
    {
      success = update(old_header_, new_header);
    }
    
    if(success)
    {
      old_header_ = new_header;
      publishPoints();
    }
    return success;
  }
  
  
  bool update(std_msgs::Header old_header, std_msgs::Header new_header)
  {
    try
    {
      ROS_DEBUG("Getting Transformation details");
      geometry_msgs::TransformStamped trans = tf_buffer_.lookupTransform(new_header.frame_id, new_header.stamp,
                                                                      old_header.frame_id, old_header.stamp,
                                                                      odom_frame_id_); 
      ego_circle_.applyTransform(trans);      
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
      return false;
    }
    return true;
  }
  
};


std::vector<EgoCircularPoint> makePoints(int num)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  
  ROS_INFO_STREAM("Making " << num << " points");
  std::vector<EgoCircularPoint> points;
  
  for(int i = 0; i < num; i++)
  {
    //float d = std::fmod(i,4);
    float d = std::generate_canonical<double,10>(gen);
    float angle = 6.0 /num * i;
    float x = std::sin(angle)*d;
    float y = std::cos(angle)*d;
    EgoCircularPoint point(x,y);
    points.push_back(point);
  }
  return points;
}

int main(int argc, char **argv)
{
  std::string name= "ego_circle_tester";
  ros::init(argc, argv, name);
  ros::start();
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  EgoCircleROS circle_wrapper;
  circle_wrapper.init();
  
  EgoCircle& circle = circle_wrapper.ego_circle_;
  
  circle.countPoints();
  //EgoCircle circle(512);
  circle.insertPoints(makePoints(500));
//   circle.printPoints();
  circle.countPoints();
  
  circle_wrapper.publishPoints();
  circle.countPoints();
  
  geometry_msgs::TransformStamped trans;
  trans.transform.rotation.w = 1;
  circle.applyTransform(trans);
  circle_wrapper.publishPoints();
  
  circle_wrapper.publishPoints();
  
  circle.countPoints();
  
  
  circle.updateCells();
  circle.countPoints();
  
  circle_wrapper.publishPoints();
  
  
  circle.applyTransform(trans);
  circle.countPoints();
  
  circle_wrapper.publishPoints();
  
  circle.applyTransform(trans);
  circle.countPoints();
  
  circle_wrapper.publishPoints();
  
  trans.transform.translation.x = 1;
  circle.applyTransform(trans);
  circle_wrapper.publishPoints();
  
  
//   circle.printPoints();
  
//  ros::spin();
}
