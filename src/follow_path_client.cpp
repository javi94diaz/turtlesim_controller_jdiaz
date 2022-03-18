#include "ros/ros.h"
#include "practicas/FollowPath.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_path_client");
  
  // User help in case of wrong number of arguments
  if (argc!=4)
  {
    ROS_INFO("Usage help >> follow_path [x1,x2,x3] [y1,y2,y3] speed");
    return 1;
  }
  
  // Starts the client
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<practicas::FollowPath>("follow_path");
  practicas::FollowPath srv;

  // Sets the request variable values
  std::vector<char> x (argv[1], argv[1] + 3);
  std::vector<char> y (argv[2], argv[2] + 3);

  for (int i = 1; i <= x.size(); i++){
    srv.request.x[i] = x[i] - '0';
    srv.request.y[i] = y[i] - '0';
  }
  
  srv.request.speed = atoll(argv[3]);
  
  // Calls the service
  if (client.call(srv)) {
    ROS_INFO("Success: %d", srv.response.success);
  }
  else {
    ROS_ERROR("Failed to call follow_path service");
    return 1;
  }
  
  return 0;
}
