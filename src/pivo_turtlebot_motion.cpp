#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <vector>
#include <cmath>

//socket comm
#include <thread>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

//Socket comm
#define BUF_SIZE 28
#define portnum 8000
#define threshold 0.2
#define IP_ADDR "192.168.0.93"

//socket variable
int serv_sock; //server socket
struct sockaddr_in serv_adr, clnt_adr;
socklen_t clnt_adr_sz;//client adress size
int str_len;//received m length
char Connected[BUF_SIZE];//connnect message

//socket variable
unsigned char message_rec[16];
float _x1 = -1, _x2, _y1, _y2;
char message_hand[73];

// Reminder message
const char* msg = R"(
This code move turtlebot to follow people using Pivo tracking.
)";

ros::Publisher pub;

// Create Twist message
geometry_msgs::Twist twist;

//바운드 좌표 및 크기
float size = 0.5;

//레이저 센서 앞에 장애물과의 거리
std::vector <float> _ranges;

//found를 위한 
double t1 = 0, t2 = 0;

//Socket comm
void error_handling(const char * message) {
  fputs(message, stderr);
  fputc('\n', stderr);
  exit(1);
}

//Socket message recieve
void socektReceiveThread() {
  while (ros::ok()) {
    int x1, y1, x2, y2;

    t1 = ros::Time::now().toSec();

    //ROS_INFO("thread -ing");
    //ZeroMemory(&message_rec, 72);
    recvfrom(serv_sock, message_rec, 16, 0, (struct sockaddr *)&clnt_adr, &clnt_adr_sz); //클라이언트로부터 실제 메세지 받는 한 줄

    //hand(받은 메세지를 사용하기 편하게 변수로 바꾸어 저장)
    memcpy(&x1,	&message_rec,		sizeof(int));
    memcpy(&x2,	&message_rec[4],	sizeof(int));
    memcpy(&y1,	&message_rec[8],	sizeof(int));
    memcpy(&y2,	&message_rec[12],	sizeof(int));

    _x1 = (float)x1 / (float)970;
    _x2 = (float)x2 / (float)970;
    _y1 = (float)y1 / (float)720;
    _y2 = (float)y2 / (float)720;

    ROS_INFO("bound box: (%f, %f), (%f, %f)",_x1,_x2,_y1,_y2);
  }
}

void found_people()
{
  ros::Rate loop_rate(100);
  ros::spinOnce();

  while(ros::ok())
  {
    std::cout << "found_people" << '\n';
    twist.angular.z = -0.6;

    if(_x1 != -1) break;

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

  while(ros::ok())
  {
    std::cout << "track_people" << '\n';
  
    t2 = ros::Time::now().toSec();

    if (t2 - t1 > 1) {
      _x1 = -1;
      break;
    }

    float x = (_x1+_x2)/2;
    float y = (_y1+_y2)/2;
    float size2 = fabs(_x2-_x1)*fabs(_y2-_y1);

    float linear_speed = (size - size2) * 0.5;
    if(linear_speed < 0.03 && linear_speed > -0.03) linear_speed = 0;
    if(linear_speed > 0.26) linear_speed = 0.26;
    if(linear_speed < -0.26) linear_speed = -0.26;

    float angular_speed = (x - 0.5) * 0.8;
    if(angular_speed > 0.4) angular_speed = 0.4;
    if(angular_speed < -0.4) angular_speed = -0.4;

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
  // ros::Subscriber sub = nh.subscribe("bound_topic", 1000, boundCallback);  //카메라 바운드 좌표 받아오기
  ros::Subscriber sub2 = nh.subscribe("/scan", 1000, rangesCallback); //장애물 센서 거리 받아오기

  printf("%s", msg);

  // comm: create socket
  serv_sock = socket(PF_INET, SOCK_DGRAM, 0); //소켓 개통(IPv4, UDP 통신)
  if (serv_sock == -1) {
      error_handling ("UDP socket creation error");
  }
  else {
      ROS_INFO("1. success to creating UDP socket");
  }

  // comm: memset
  memset(&serv_adr, 0, sizeof(serv_adr));
  serv_adr.sin_family = AF_INET;
  serv_adr.sin_addr.s_addr = htonl(INADDR_ANY); //IP주소 리드
  serv_adr.sin_port = htons(portnum);

  // comm: binding -> assign ip address and port number
  if (bind(serv_sock, (struct sockaddr *)&serv_adr, sizeof(serv_adr)) == -1) {
      error_handling("bind() error");
  } else {
      ROS_INFO("2. success binding");
  }

  //서버 소켓 ip주소 출력 
  ROS_INFO("server ip: %s", inet_ntoa(serv_adr.sin_addr));

  // comm: Receive connectM from Client
  clnt_adr_sz = sizeof(clnt_adr); //client address size
  // str_len = recvfrom(serv_sock, Connected, BUF_SIZE, 0, (struct sockaddr *)&clnt_adr, &clnt_adr_sz);//receive M(Connected) from client
  // Connected[str_len] = 0;
  // ROS_INFO("3. message from client: %s", Connected);

  std::thread t1(socektReceiveThread); //thread keep receiving socket data from unity while ros ok, thread는 생성한 후 다음 함수로 바로 넘어가

  //ros::ok() 를 써야 ctrl+c 할 때 종료할 수 있음
  while(ros::ok())
  {
    found_people();
    track_people();

    ros::spinOnce();
  }

  t1.detach();
  close(serv_sock);
  return 0;
}

