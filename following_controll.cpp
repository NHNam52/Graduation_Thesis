#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <cstdlib> // Dùng để sinh số ngẫu nhiên
#include <string>
#include <iostream>

#define pi 3.14159
#define L 0.4
#define R 0.0725

int person = 0; 
int16_t r_speed = 0, l_speed = 0;
float alpha = 0.0, distance = 0.0, total = 0.0, v = 0.0, w = 0.0;
char buffer[10];
char n ='t';

serial::Serial serial_port;

void calc_speed() {
  r_speed = (int16_t)((2*v + w*L) * 30 / (pi * R));
  l_speed = (int16_t)((2*v - w*L) * 30 / (pi * R));
}

void auto_controll() 
{
  if (person==1)
  {
    if(-12.0<=alpha && alpha<=12.0) 
    { 
      if (distance <= 2.0)
      {
        v = 0; 
        w = 0; 
        calc_speed();
      }

      else 
      {
        v = 0.3; 
        w = 0; 
        calc_speed();
      }
    }
    else if(12.0<alpha && alpha<=25.0) 
    { 
      if (distance <= 2.2)
      {
        v = 0; 
        w = 0.2;
        calc_speed();
      }
      else 
      {
        v = 0.1; 
        w = 0.2;
        calc_speed();
      }

    } 
    else if (-25.0<=alpha && alpha < -12.0)
    { 
      if (distance <= 2.2)
      {
        v = 0; 
        w = -0.2;
        calc_speed();
      }
      else 
      {
        v = 0.1; 
        w = -0.2;
        calc_speed();
      }
    }
  }
  else
  {
    if(alpha > 12.0)
    {
        v = 0; 
        w = 0.2;
        calc_speed();
    }
    if(alpha < -12.0)
    {
        v = 0;
        w = -0.2;
        calc_speed();
    }
  }
}

void callback1(const std_msgs::Float32::ConstPtr& msg1) {
  ROS_INFO("alpha: %0.2f", msg1->data);
  alpha = msg1->data;
}
void callback2(const std_msgs::Float32::ConstPtr& msg2) {
  ROS_INFO("distance: %0.2f", msg2->data);
  distance = msg2->data;
}
void callback3(const std_msgs::Int32::ConstPtr& msg3) {
  ROS_INFO("is_person: %d", msg3->data);
  person = msg3->data;
  auto_controll();
  ROS_INFO("vel_wheel: %d, %d", r_speed, l_speed);
  
  snprintf(buffer, 10, "%c%04d%04d", n, r_speed, l_speed);
  ROS_INFO("Buffer vel: %s", buffer);  
  serial_port.write(buffer);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "following_controll");
  ros::NodeHandle nh;
  
  // Khởi tạo đối tượng serial
  
  serial_port.setPort("/dev/ttyTHS1");
  serial_port.setBaudrate(19200);
  serial_port.setTimeout(serial::Timeout::max(), 100, 0, 100, 0);

  ros::Subscriber sub_alpha = nh.subscribe("alpha", 100, callback1);
  ros::Subscriber sub_distance = nh.subscribe("distance", 100, callback2);
  ros::Subscriber sub_person = nh.subscribe("is_person", 100, callback3);
  
 

  try {
    serial_port.open();
  }
  catch (serial::IOException &e) {
    ROS_ERROR_STREAM("Cannot connect to UART port, Bug: " << e.what());
    return -1;
  }
  ros::spin();
  // Gửi dữ liệu xuống STM32 và hiển thị lên terminal
  while (ros::ok()) {
    if (serial_port.available()) {
      snprintf(buffer, sizeof(buffer), "%04d%04d", r_speed, l_speed);
      serial_port.write(buffer);
      ROS_INFO("Buffer vel: %s", buffer);  
    }
    ros::Duration(1.0).sleep();
    }
  return 0;
}