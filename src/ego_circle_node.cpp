#include <egocircle/ego_circle.h>

int main(int argc, char **argv)
{
  std::string name= "egocircle_node";
  ros::init(argc, argv, name);
  ros::start();
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  ego_circle::EgoCircleROS circle_wrapper;
  circle_wrapper.init();
  
  ego_circle::EgoCircle& circle = circle_wrapper.ego_circle_;
  
  //circle.insertPoints(ego_circle::makePoints(500),false);
  ros::spin();
}
