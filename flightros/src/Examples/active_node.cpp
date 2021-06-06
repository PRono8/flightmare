#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "active_node");

  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  ros::Publisher chatter_pub = pnh.advertise<std_msgs::Bool>("active", 1);


  ros::Rate loop_rate(50);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::Bool msg;

    msg.data = true;


    //ROS_INFO(msg);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;

}
