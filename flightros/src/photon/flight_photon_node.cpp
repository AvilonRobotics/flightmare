#include <ros/ros.h>

#include "flightros/photon/flight_photon.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "flight_photon");
  flightros::FlightPilot pilot(ros::NodeHandle(), ros::NodeHandle("~"));

  // spin the ros
  ros::spin();

  return 0;
}