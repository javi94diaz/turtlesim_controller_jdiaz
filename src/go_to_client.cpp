#include "ros/ros.h"
#include "practicas/GoTo.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_to_client");
  
  if (argc != 4)
  {
    ROS_INFO("usage help: go_to x y speed");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<practicas::GoTo>("go_to");
  practicas::GoTo srv;
  
  srv.request.x = atoll(argv[1]);
  srv.request.y = atoll(argv[2]);
  srv.request.speed = atoll(argv[3]);
  
  if (client.call(srv)) {
    ROS_INFO("Success: %d", srv.response.success);
  }
  else {
    ROS_ERROR("Failed to call go_to service");
    return 1;
  }
  
  return 0;
}
