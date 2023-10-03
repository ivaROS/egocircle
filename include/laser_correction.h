#ifndef LASER_CORRECTION_H
#define LASER_CORRECTION_H

#include <vector>
#include <cmath>

#include <geometry_msgs/TransformStamped.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

namespace laser_correction
{
    constexpr float dNaN=(std::numeric_limits<float>::has_quiet_NaN) ? std::numeric_limits<float>::quiet_NaN() : 0;
    class LaserCorrection
    {
        
        typedef std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
        typedef std::shared_ptr<tf2_ros::TransformListener> transform_listener_ptr;

    public:
        LaserCorrection(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        ~LaserCorrection(){};

        void init();
        void scanCB(const sensor_msgs::LaserScan::ConstPtr& laser_msg);

    private:
        std::string name_= "laser_correction";
        ros::NodeHandle nh_, pnh_;
        tf_buffer_ptr tfBuffer_;
        transform_listener_ptr tf_listener_;
        // std::shared_ptr<tf::TransformListener> tfListener_;

        std::string laser_topic_;

        ros::Publisher laser_pub_;
        ros::Subscriber laser_sub_;

        double static_height_;
        std::string fixed_frame_id_, corrected_laser_id_;
        laser_geometry::LaserProjection projector_;
    };
} // namespace laser_correction



#endif