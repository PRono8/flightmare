#include <ros/ros.h>

#include "flightros/Examples/example1.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "example1");
  flightros::FlightPilot pilot(ros::NodeHandle(), ros::NodeHandle("~"));

  // spin the ros
  ros::spin();

  return 0;
}
