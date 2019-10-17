
#include <egocircle/ego_circle.h>
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

#include <type_traits>

namespace ego_circle
{
  
  void EgoCircularCell::insertPoint(EgoCircularPoint point, bool clearing)
  {
    float key = point.getKey();
    
    ROS_DEBUG_STREAM("Inserting point [" << point.x << "," << point.y << "] depth_sq: " << key << ", clearing: " << clearing << ", cur_min: " << current_min_);
    
    if(clearing && !border_)
    {
      if(key >= MAX_DEPTH_SQ)
      {
        reset();
      }
      else
      {
        if(points_.size() == 0)
        {
          points_.push_back(point);
          current_min_ = key;
        }
        else if(key < current_min_)
        {
          current_min_ = key;
          points_[0] = point;
        }
      }
      cleared_ = true;
    }
    else
    {
      if(cleared_)
      {
        if(key > current_min_)
        {
          points_.push_back(point);
        }
      }
      else
      {
        if(key < current_min_)
        {
          current_min_ = key;
        }
        if(key < MAX_DEPTH_SQ)
        {
          points_.push_back(point);
        }
      }
    }
    
    
  }
  
  void EgoCircularCell::reset()
  {
    points_.clear();
    current_min_ = MAX_DEPTH_SQ;
    cleared_ = false;
  }
  
  void EgoCircularCell::reset(float max_depth_sq)
  {
    MAX_DEPTH_SQ = max_depth_sq;
    reset();
  }
  
  void EgoCircularCell::applyTransform(SE2Transform transform)
  {
    for(auto& point : points_)
    {
      ego_circle::applyTransform(point, transform);
    }
  }
  
  void EgoCircularCell::printPoints() const
  {
    for(const auto& point : points_)
    {
      EgoCircularPoint p = point;
      ROS_INFO_STREAM("\t\t[" << p.x << "," << p.y << "]");
    }
  }
  
  void EgoCircle::insertPoints(std::vector<EgoCircularCell>& cells, std::vector<EgoCircularPoint> points, bool clearing)
  {
    for(auto point : points)
    {
      insertPoint(cells, point, clearing);
    }
  }
  
  void EgoCircle::insertPoints(std::vector<EgoCircularPoint> points, bool clearing)
  {
    insertPoints(cells_, points, clearing);
  }
  
  void EgoCircle::insertPoints(EgoCircle& other)
  {
    for(auto point : other)
    {
      insertPoint(cells_, point, false);
    }
  }
  
  void EgoCircle::insertPoints(const sensor_msgs::LaserScan& scan)
  {
    int start_ind = getIndex(scan.angle_min);
    int end_ind = getIndex(scan.angle_max);
    
    cells_[start_ind].border_ = true;
    cells_[end_ind].border_ = true;
    
    LaserScanWrapper it_scan(scan);
    
    for(auto pnt : it_scan)
    {
      if(pnt.r==pnt.r)
      {
        insertPoint(cells_, pnt, clearing_enabled_);
      }
    }
  }
  
  void EgoCircle::insertPoints(PCLPointCloud points)
  {
    for(auto point : points)
    {
      EgoCircularPoint ec_pnt(point.x, point.y);
      insertPoint(cells_, ec_pnt, clearing_enabled_);
    }
  }
  
  
  void EgoCircle::applyTransform(geometry_msgs::TransformStamped transform)
  {
    ros::WallTime start = ros::WallTime::now();
    SE2Transform trans(transform);
    
    for(EgoCircularCell& cell : cells_)
    {
      cell.applyTransform(trans);
    }
    
    ros::WallTime end = ros::WallTime::now();
    
    ROS_DEBUG_STREAM_NAMED("timing", "Applying transform took " <<  (end - start).toSec() * 1e3 << "ms");
    
  }
  
  std::vector<float> EgoCircle::getDepths()
  {
    std::vector<float> depths(cells_.size());
    for(int i = 0; i < cells_.size(); i++)
    {
      const auto& cell = cells_[i];
      float depth = max_depth_;
      if(cell.points_.size() > 0)
      {
        depth = std::sqrt(cell.current_min_);
      }
      depths[i] = depth;
    }
    return depths;
  }
  
  int EgoCircle::getN(float depth)
  {
    return converter_.getN(depth);
  }
  
  std::vector<float> EgoCircle::inflateDepths(const std::vector<float>& depths)
  {
    std::vector<float> inflated_depths = depths; //(depths.size());
    
    std::vector<int> ns(depths.size());
    for(int i = 0; i < depths.size(); i++)
    {
      ns[i] = getN(depths[i]);
      //inflated_depths[i]-=inscribed_radius_;
    }
    int n;
    for(int i = 0; i < depths.size(); i++)
    {
      n = ns[i];
      
      for(int j = i - n; j < i + n; j++)
      {
        int ind = (j <0) ? depths.size() + j : ((j >= depths.size()) ? j - depths.size() : j );
        inflated_depths[ind] = std::min(depths[i] -inscribed_radius_,inflated_depths[ind]);
      }
      
      //       for(int j = i+1; j <= i + n; j++)
      //       {
      //         int ind = (j < depths.size()) ? j : j - depths.size();
      //         inflated_depths[ind] = std::min(depths[ind]-inscribed_radius_,depths[i]-inscribed_radius_);
      //       }
    }
    
    return inflated_depths;
  }
  
  void EgoCircle::printPoints() const
  {
    int id = 0;
    for(const EgoCircularCell& cell : cells_)
    {
      ROS_INFO_STREAM("\tCell #" << id << ":");
      cell.printPoints();
      id++;
    }
  }
  
  void EgoCircle::countPoints() const
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
  
  void EgoCircle::reset()
  {
    for(EgoCircularCell& cell : cells_)
    {
      cell.reset();
    }
  }
  
  void EgoCircle::reset(float max_depth)
  {
    max_depth_ = max_depth;
    float max_depth_sq = max_depth*max_depth;
    
    for(EgoCircularCell& cell : cells_)
    {
      cell.reset(max_depth_sq);
    }
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
  
  
  
  EgoCircleROS::EgoCircleROS(ros::NodeHandle nh, ros::NodeHandle pnh):
    nh_(nh),
    pnh_(pnh),
    tf_listener_(tf_buffer_),
    ego_circle_(512),
    old_ego_circle_(512)
  {
    
  }
  
  bool EgoCircleROS::init()
  {
    int odom_queue_size = 5;
    std::string odom_topic = "odom";
    std::string pointcloud_topic = "pointcloud";
    std::string laserscan_topic = "scan";
    
    //ROS_WARN_STREAM("Swapping is good since object is move constructible: " << std::is_move_constructible<EgoCircle>());
    
    if(!pnh_.param<std::string>("base_frame_id", base_frame_id_, "base_link"))
    {
      pnh_.setParam("base_frame_id", base_frame_id_);
    }
    
    if(!pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom"))
    {
      pnh_.setParam("odom_frame_id", odom_frame_id_);
    }
    
    dsrv_ = std::make_shared<dynamic_reconfigure::Server<egocircle::egocircleConfig> >(pnh_);
    dynamic_reconfigure::Server<egocircle::egocircleConfig>::CallbackType cb = boost::bind(&EgoCircleROS::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    
    pc_tf_filter_ = std::make_shared<PC_TF_Filter>(pc_subscriber_, tf_buffer_, odom_frame_id_, odom_queue_size, nh_); //NOTE: this is the correct form for any message but an odometry message
    pc_tf_filter_->registerCallback(boost::bind(&EgoCircleROS::pointcloudCB, this, _1));
    
    //tf_filter_ = std::make_shared<TF_Filter>(odom_subscriber_, tf_buffer_, "base_footprint", odom_queue_size, nh_);
    //tf_filter_->registerCallback(boost::bind(&EgoCircleROS::odomCB, this, _1));
    pc_tf_filter_->setTolerance(ros::Duration(0.01));
    
    ego_circle_.clearing_enabled_ = true;
    
    ls_tf_filter_ = std::make_shared<LS_TF_Filter>(ls_subscriber_, tf_buffer_, odom_frame_id_, odom_queue_size, nh_); //NOTE: this is the correct form for any message but an odometry message
    ls_tf_filter_->registerCallback(boost::bind(&EgoCircleROS::laserscanCB, this, _1));
    ls_tf_filter_->setTolerance(ros::Duration(0.01));
    
    ls_subscriber_.subscribe(nh_, laserscan_topic, 5);
    
    pc_subscriber_.subscribe(nh_, pointcloud_topic, 5);
    
    //odom_subscriber_.subscribe(nh_, odom_topic, odom_queue_size);
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>("vis",5);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("point_scan",5);
    inflated_scan_pub_ =  nh_.advertise<sensor_msgs::LaserScan>("inflated_point_scan",5);
  }
  
  void EgoCircleROS::reconfigureCB(const egocircle::egocircleConfig& config, uint32_t level)
  {
    {
      //lock mutex
      cur_config_ = config;
    }
    ROS_INFO_STREAM("ReconfigureCB: MaxDepth=" << cur_config_.max_depth);
  }
  
  void EgoCircleROS::publishPoints()
  {
    if(vis_pub_.getNumSubscribers()>0)
    {
      visualization_msgs::Marker msg = getVisualizationMsg();
      vis_pub_.publish(msg);
    }
    
  }
  
  void EgoCircleROS::odomCB(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
    std_msgs::Header header = odom_msg->header;
    header.frame_id = odom_msg->child_frame_id;
    
    update(header);
  }
  
  void EgoCircleROS::prep()
  {
    //swap(ego_circle_, old_ego_circle_);
    std::swap(ego_circle_.cells_, old_ego_circle_.cells_);

    {
      //TODO: lock mutex here!
      if(ego_circle_.max_depth_ == cur_config_.max_depth)
      {
        ego_circle_.reset();
      }
      else
      {
        ROS_INFO_STREAM("Setting new egocircle max depth to " << cur_config_.max_depth);
        ego_circle_.reset(cur_config_.max_depth);
      }
      
    }
    
  }
  
  void EgoCircleROS::pointcloudCB(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
  {
    ros::WallTime starttime = ros::WallTime::now();
    
    prep();
    
    std_msgs::Header header = pointcloud_msg->header;
    header.frame_id = base_frame_id_;
    
    try 
    {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped = tf_buffer_.lookupTransform(base_frame_id_, header.stamp, pointcloud_msg->header.frame_id, header.stamp,
                                                    odom_frame_id_); 
      
      sensor_msgs::PointCloud2 transformed_cloud;
      
      tf2::doTransform(*pointcloud_msg, transformed_cloud, transformStamped);
      
      PCLPointCloud::Ptr cloud (new PCLPointCloud);
      
      pcl::fromROSMsg(transformed_cloud, *cloud);
      
      PCLPointCloud::Ptr new_cloud(new PCLPointCloud);
      std::vector<int> mapping;
      //Is this necessary?
      pcl::removeNaNFromPointCloud(*cloud, *new_cloud, mapping);
      
      
      
      ego_circle_.insertPoints(*new_cloud);
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
    }
    
    update(header);
    
    ROS_DEBUG_STREAM_NAMED("timing", "Point insertion and update took " <<  (ros::WallTime::now() - starttime).toSec() * 1e3 << "ms");
    
    
  }
  
  
  void EgoCircleROS::laserscanCB(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    if(scan)
    {
      ros::WallTime starttime = ros::WallTime::now();
      
      prep();
      
      std_msgs::Header header = scan->header;
      ego_circle_.insertPoints(*scan);
      
      update(header);
      
      ROS_DEBUG_STREAM_NAMED("timing", "Point insertion and update took " <<  (ros::WallTime::now() - starttime).toSec() * 1e3 << "ms");
      
    }
    else
    {
      ROS_WARN("Attempting to insert points from empty LaserScan message!");
    }
  }
  
  visualization_msgs::Marker EgoCircleROS::getVisualizationMsg()
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
    
    ROS_INFO_STREAM_THROTTLE(1, "Total # points= " << marker.points.size());
    ROS_DEBUG_STREAM("Publishing " << marker.points.size() << " points");
    
    return marker;
  }
  
  void EgoCircleROS::publishDepthScans()
  {
    bool publish_scans = scan_pub_.getNumSubscribers()>0;
    bool publish_inflated_scans = inflated_scan_pub_.getNumSubscribers()>0;
    
    if(publish_scans || publish_inflated_scans)
    {
      std::vector<float> depths = ego_circle_.getDepths();
      
      sensor_msgs::LaserScan scan;
      scan.header = old_header_;
      scan.angle_min= -std::acos(-1);
      scan.angle_max= std::acos(-1);
      scan.angle_increment = 1/ego_circle_.indexer_.scale;
      scan.ranges = depths;
      scan.range_min = 0;
      scan.range_max = ego_circle_.max_depth_ + OFFSET;  //TODO: make this .01 and see if still visible. Or even get rid of the increase so that only actual points are shown; maybe make it a parameter
      
      if(publish_scans)
      {
        scan_pub_.publish(scan);
      }
      
      if(publish_inflated_scans)
      {
        scan.ranges = ego_circle_.inflateDepths(depths);
        
        inflated_scan_pub_.publish(scan);
      }
    }
  }
  
  bool EgoCircleROS::update(std_msgs::Header new_header)
  {
    bool success = true;
    
    if(old_header_.stamp > ros::Time(0))
    {
      success = update(old_header_, new_header);
    }
    
    old_header_ = new_header;
    
    if(success)
    {
      publishPoints();
      publishDepthScans();
    }
    return success;
  }
  
  
  bool EgoCircleROS::update(std_msgs::Header old_header, std_msgs::Header new_header)
  {
    try
    {
      ROS_DEBUG("Getting Transformation details");
      geometry_msgs::TransformStamped trans = tf_buffer_.lookupTransform(new_header.frame_id, new_header.stamp,
                                                                         old_header.frame_id, old_header.stamp,
                                                                         odom_frame_id_); 
      old_ego_circle_.applyTransform(trans);
      
      ego_circle_.insertPoints(old_ego_circle_);
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
      return false;
    }
    return true;
  }
  
  
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
  
}

