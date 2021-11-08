#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

//socket comm
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <iostream>
#include <stdio.h>
#include <vector>
#include <cmath>

// Reminder message
const char* msg = R"(
This code move turtlebot to track people.
)";

ros::Publisher pub;

// Create Twist message
geometry_msgs::Twist twist;

//바운드 좌표 및 크기
double _x1 = -1, _x2, _y1, _y2;
double size = 0.5;

bool check = true; //size와 size2 관련 체크변수

//레이저 센서 앞에 장애물과의 거리
std::vector <float> _ranges;

void found_people()
{
  ros::Rate loop_rate(100);
  ros::spinOnce();

  while(true)
  {
    std::cout << "found_people" << '\n';
    twist.angular.z = 0.15;

    if(_x1 != -1 || _x2 != 0 || _y1 != 0 || _y2 != 0) break;

    // Publish it and resolve any remaining callbacks
    pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();

  }

  twist.linear.x = 0;
  twist.angular.z = 0;
  pub.publish(twist);
}

void track_people()
{
  ros::Rate loop_rate(100);
  ros::spinOnce();

  twist.linear.y = twist.linear.z = twist.angular.x = twist.angular.y = 0;
  //size = fabs(_x2-_x1)*fabs(_y2-_y1);

  //double size2 = size;

  while(true)
  {
    std::cout << "track_people" << '\n';
    if(_x1 == -1) break;

    double x = (_x1+_x2)/2;
    double y = (_y1+_y2)/2;
    double size2 = fabs(_x2-_x1)*fabs(_y2-_y1);

    double linear_speed = size - size2;
    if(linear_speed < 0.01 && linear_speed > -0.01) linear_speed = 0;
    if(linear_speed > 0.05) linear_speed = 0.05;
    if(linear_speed < -0.05) linear_speed = -0.05;

    double angular_speed = x - 0.5;
    if(angular_speed > 0.1) angular_speed = 0.1;
    if(angular_speed < -0.1) angular_speed = -0.1;

    /*
    //장애물 처리
    if(linear_speed > 0 && _ranges[0] < 0.001)
    {
      linear_speed *= (-1);
    }
    else if(linear_speed < 0 && _ranges[180] < 0.001)
    {
      linear_speed *= (-1);
    }
    */

    twist.linear.x = linear_speed;
    twist.angular.z = angular_speed;
   
    //size = size2;
    
    // Publish it and resolve any remaining callbacks
    pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
    
    size2 = fabs(_x2-_x1)*fabs(_y2-_y1);
  }

  twist.linear.x = 0;
  twist.angular.z = 0;
  pub.publish(twist);
}

//바운드 박스 받아오기

void rangesCallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
  //받아오기
  _ranges = laser->ranges;

  //ROS_INFO("I received ranges: [%f, %f]", _ranges[0], _ranges[360]);
}

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "pivo_turtlebot_motion");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // Init odom subsciber
  //ros::Subscriber sub = nh.subscribe("bound_topic", 1000, boundCallback);  //카메라 바운드 좌표 받아오기
  ros::Subscriber sub2 = nh.subscribe("/scan", 1000, rangesCallback); //장애물 센서 거리 받아오기

  printf("%s", msg);

  while(true)
  {
    found_people();
    track_people();

    ros::spinOnce();
  }

  return 0;
}

