#include "ros/ros.h"
#include "practicas/GoTo.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <iostream>
#include <math.h>

ros::Publisher pub;
geometry_msgs::Twist mensaje;
float curr_x;
float curr_y;
float curr_theta;
float goal_x;
float goal_y;
float speed;

void funcionsub (const turtlesim::Pose::ConstPtr& msg)
{
    //ROS_INFO("Saving turtle pose: [%f %f %f]", msg->x, msg->y, msg->theta);
    curr_x = msg->x;
    curr_y = msg->y;
    curr_theta = msg->theta;
}

bool funcionservicio (practicas::GoTo::Request &req, practicas::GoTo::Response &res)
{
  // Si las variables no son cero, el servicio devuelve true
  if (req.x != 0 && req.y != 0 && req.speed != 0)
  {
    ROS_INFO("Requested position: x=%ld, y=%ld", req.x, req.y);
    ROS_INFO("Requested speed: %ld m/s", req.speed);
    res.success = true;
  }
  else
  {
    ROS_INFO("Parameters received incorrectly");
    res.success = false;
  }

  // Guardamos los objetivos recibidos
  goal_x = req.x;
  goal_y = req.y;
  speed = req.speed;

  return res.success;
}

float errorDist(){
  float deltaX = goal_x - curr_x;
  float deltaY = goal_y - curr_y;
  float error = sqrt( pow(deltaX, 2) + pow(deltaY, 2) );
  return error;
}

float deltaTheta(){
  float deltaX = goal_x - curr_x;
  float deltaY = goal_y - curr_y;
  float goal_theta = atan2(deltaY, deltaX);
  float deltaTheta = goal_theta - curr_theta;
  ROS_INFO("goal_theta=%f, curr_theta=%f, deltaTheta=%f", goal_theta, curr_theta, deltaTheta);
  return deltaTheta;
}

void control(){

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
      ROS_INFO("Posicion final: x=%f, y=%f, theta=%f", curr_x, curr_y, curr_theta);
  }

  // Publicamos
  pub.publish(mensaje);
  //ROS_INFO("Publicado mensaje");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_to_server");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("turtle1/pose", 1, funcionsub);
  
  pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

  ros::ServiceServer service = n.advertiseService("go_to", funcionservicio);
  ROS_INFO("Server ready to move the turtle to a goal point");
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    //ROS_INFO("Currents -> x=%f, y=%f, theta=%f", curr_x, curr_y, curr_theta);
    control();
    loop_rate.sleep();
    ros::spinOnce();
  }

  
  return 0;
}
