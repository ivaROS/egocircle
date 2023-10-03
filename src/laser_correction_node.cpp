#include <laser_correction.h>

namespace laser_correction
{
    LaserCorrection::LaserCorrection(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    nh_(nh),
    pnh_(pnh)
    {
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(50)); //optional parameter: ros::Duration(cache time) (default=10) (though it doesn't seem to accept it!)
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        // tfListener_ = std::make_shared<tf::TransformListener>(ros::Duration(200));
    }

    void LaserCorrection::init()
    {
        laser_topic_ = "/scan";
        std::string corrected_laser_topic = "/corrected_scan";
        fixed_frame_id_ = "map";
        corrected_laser_id_ = "corrected_laser";

        pnh_.getParam("fixed_frame_id", fixed_frame_id_ );
        pnh_.setParam("fixed_frame_id", fixed_frame_id_ );

        static_height_ = 0.3;

        pnh_.getParam("static_height", static_height_ );
        pnh_.setParam("static_height", static_height_ );

        laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan> (laser_topic_, 10, &LaserCorrection::scanCB, this);
        laser_pub_ = nh_.advertise<sensor_msgs::LaserScan> (corrected_laser_topic, 1);

    }

    void LaserCorrection::scanCB(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
    {
        sensor_msgs::PointCloud2 cloud;
        sensor_msgs::LaserScan tmp_laser = *laser_msg;
        tmp_laser.header.stamp = ros::Time::now();

        geometry_msgs::TransformStamped tmp_transformStamped;
        try{
            tmp_transformStamped = tfBuffer_->lookupTransform(corrected_laser_id_, laser_msg->header.frame_id,
                                    laser_msg->header.stamp, ros::Duration(2));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            return;
        }
        
        projector_.transformLaserScanToPointCloud(corrected_laser_id_, *laser_msg, cloud, *tfBuffer_);

        // ROS_INFO_STREAM(2);

        ROS_INFO_STREAM("Generating new laser");

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(cloud,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*pt_cloud);

        // Get current sensor height
        // tf::StampedTransform transform;
        geometry_msgs::TransformStamped transformStamped;
        try{
            // tfListener_->waitForTransform(fixed_frame_id_, corrected_laser_id_, ros::Time(0), ros::Duration(10));
            // tfListener_->lookupTransform(fixed_frame_id_, corrected_laser_id_,  
            //                         ros::Time(0), transform);
            transformStamped = tfBuffer_->lookupTransform(fixed_frame_id_, corrected_laser_id_, ros::Time(0));
        }
            catch (tf2::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return;
        }
        // double current_sensor_height = transform.getOrigin()[2];
        double current_sensor_height = transformStamped.transform.translation.z;
        double static_height_in_sensor = static_height_ - current_sensor_height;
        // ROS_INFO_STREAM(static_height_ << " " << current_sensor_height << " " << static_height_in_sensor);

        sensor_msgs::LaserScan out_scan;
        out_scan.header.stamp = laser_msg->header.stamp;
        out_scan.header.frame_id = corrected_laser_id_;
        out_scan.angle_min = laser_msg->angle_min;
        out_scan.angle_max = laser_msg->angle_max;
        out_scan.angle_increment = laser_msg->angle_increment;
        out_scan.time_increment = laser_msg->time_increment;
        out_scan.scan_time = laser_msg->scan_time;
        out_scan.range_min = laser_msg->range_min;
        out_scan.range_max = laser_msg->range_max;
        out_scan.intensities = laser_msg->intensities;
        out_scan.ranges.resize(laser_msg->ranges.size(), dNaN);

        for(size_t i = 0; i < pt_cloud->points.size(); i++)
        {
            float x = pt_cloud->points[i].x;
            float y = pt_cloud->points[i].y;
            float z = pt_cloud->points[i].z;

            if(z >= static_height_in_sensor)
            {
                float range = sqrt(x*x + y*y);
                float angle = atan2(y, x);
                int idx = int(round((angle - out_scan.angle_min) / out_scan.angle_increment));
                out_scan.ranges[idx] = range;
            }
        }

        laser_pub_.publish(out_scan);
    }
}

int main(int argc, char **argv)
{
  std::string name= "laser_correction";
  ros::init(argc, argv, name);
  ros::start();
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  laser_correction::LaserCorrection c(nh, pnh);
  c.init();
  
  //circle.insertPoints(ego_circle::makePoints(500),false);
  ros::spin();
}
