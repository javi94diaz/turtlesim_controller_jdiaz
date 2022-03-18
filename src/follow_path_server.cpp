#include "ros/ros.h"
#include "practicas/FollowPath.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <iostream>
#include <math.h>

ros::Publisher pub;
geometry_msgs::Twist mensaje;
float curr_x=-1;
float curr_y=-1;
float curr_theta;
std::vector<int> goal_x(3);
std::vector<int> goal_y(3);
float step = 0;
float speed;

void funcionsub (const turtlesim::Pose::ConstPtr& msg)
{
    //ROS_INFO("Saving turtle pose: [%f %f %f]", msg->x, msg->y, msg->theta);
    curr_x = msg->x;
    curr_y = msg->y;
    curr_theta = msg->theta;
}

bool funcionservicio (practicas::FollowPath::Request &req, practicas::FollowPath::Response &res)
{
  // Si la velocidad no es nula, el servicio devuelve true
  if (req.speed != 0)
  {
    ROS_INFO("Requested sequence of points: ");
    for(int i = 0; i < req.x.size(); i++){
      std::cout << req.x[i] << " " << req.y[i] << std::endl;
    }
    
    ROS_INFO("Requested speed: %ld m/s", req.speed);
    res.success = true;
  }
  else
  {
    ROS_INFO("Parameters received incorrectly");
    res.success = false;
    return res.success;
  }

  // Guardamos los objetivos recibidos
  for (int i = 0; i < req.x.size(); i++){
    goal_x[i] = req.x[i];
    goal_y[i] = req.y[i];
  }
  speed = req.speed;

  ROS_INFO("Received goal points and speed: %d %d %f", goal_x[0], goal_y[0], speed);

  return res.success;
}

float errorDist()
{
  float error;
  if(curr_x == -1 && curr_y == -1){
    error = 100;
  }
  else{
    float deltaX = goal_x.at(step) - curr_x;
    float deltaY = goal_y.at(step)  - curr_y;
    error = sqrt( pow(deltaX, 2) + pow(deltaY, 2) );
  }
  return error;
}

float deltaTheta()
{
  float deltaX = goal_x.at(step) - curr_x;
  float deltaY = goal_y.at(step)  - curr_y;
  float goal_theta = atan2(deltaY, deltaX);
  float deltaTheta = goal_theta - curr_theta;
  ROS_INFO("goal_theta=%f, curr_theta=%f, deltaTheta=%f", goal_theta, curr_theta, deltaTheta);
  return deltaTheta;
}

void control()
{
  // Errores
  if (errorDist() > 0.1){
    mensaje.linear.x = speed;
    // Corregir velocidad angular
    if (deltaTheta() > 0){
      mensaje.angular.z = speed;
    } else if (deltaTheta() < 0){
      mensaje.angular.z = -speed;
    }else{
      mensaje.angular.z = 0;
    }
  }
  else{
      mensaje.linear.x = 0;
      mensaje.angular.z = 0;
      ROS_INFO("Path point reached");
      if (step < 3){
        step++;
      }
      else{
        ROS_INFO("Final pose: x=%f, y=%f, theta=%f", curr_x, curr_y, curr_theta);
      }
  }

  pub.publish(mensaje);
  //ROS_INFO("Publicado mensaje");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_path_server");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("turtle1/pose", 1, funcionsub);
  pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

  ros::ServiceServer service = n.advertiseService("follow_path", funcionservicio);
  ROS_INFO("Server ready to move the turtle following a path");
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    ROS_INFO("Currents -> x=%f, y=%f, theta=%f", curr_x, curr_y, curr_theta);
    control();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
