// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <array>
#include <chrono>
#include <functional>
#include <math.h>
#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;
        double xRef = 0, yRef = 0;
        double vel_x = 0, vel_y = 0, vel_theta = 0;

class Regulator : public rclcpp::Node{
public:
  Regulator(): Node("turtlebot3_regulator")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, std::bind(&Regulator::vel_callback, this, std::placeholders::_1));
    vel_= this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    //timer_ = this->create_wall_timer(
    //  500ms, std::bind(&Regulator::update_vel, this));
  }

private:

  void vel_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
     if(abs(vel_x)<pow(10,-2) && abs(vel_y)<pow(10,-2)){
        std::cout << "inserire le coordinate di riferimento\n (prima x -nella forma double- press enter e inserire y -forma double-):";
        double x, y;
        std::cin >> x;
        std::cin >> y;
            do{
            std::cout << "errore - inserire nuovamente le coordinate:";
            std::cin >> x;
            std::cin >> y;
                 if(isnan(x)==0 && isnan(y)==0){
                 break;
                 }
}while(isnan(x)==1 || isnan(y)==1);
        xRef = x;
        yRef = y;
        std::cout << " coordinate inserite:\n x =" << xRef;
        std::cout << "y =" << yRef;
        }

    double xE, yE, theta;
    double l=0.1;
    //calcolo di theta attraverso la conversione quaternion -> euler angle   DA ELIMINARE POI
    double siny_cosp = 2*(msg->pose.pose.orientation.w*msg->pose.pose.orientation.z
      +msg->pose.pose.orientation.x*msg->pose.pose.orientation.y);
    double cosy_cosp = 1-2*(msg->pose.pose.orientation.y*msg->pose.pose.orientation.y +
    msg->pose.pose.orientation.z*msg->pose.pose.orientation.z);
    theta = std::atan2(siny_cosp,cosy_cosp);
    xE = msg->pose.pose.position.x + l*cos(theta);
    yE = msg->pose.pose.position.y + l*sin(theta);

    //calcolo dell'errore
    double ex, ey;
    ex = xE - xRef;
    ey = yE - yRef;

    // k1 e k2 (usati anche per y)
    double k1=0.2, k2=0.2;

    double v1, v2, u1, u2;
      v1 = -k1*ex;
      v2 = -k2*ey;

      u1 = cos(theta)*v1 + sin(theta)*v2;
      u2 = -v1*(sin(theta)/l) + v2*(cos(theta)/l);


     vel_x = u1*cos(theta) - l*sin(theta)*u2;
//    printf("%f\n", vel_x);

    vel_y = u1*sin(theta) + l*cos(theta)*u2;
//    printf("%f\n", vel_y);

    vel_theta = u2;
//    printf("%f\n", vel_theta);

    if(abs(vel_x)<pow(10,-2) && abs(vel_y)<pow(10,-2)){
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.y = 0;
      cmd_vel.linear.x = 0;
      cmd_vel.linear.z = 0;
      cmd_vel.angular.z = 0;
      cmd_vel.angular.y = 0;
      cmd_vel.angular.x = 0;
      vel_->publish(cmd_vel);
    }
//    else if(abs(vel_y)>0.22 && abs(vel_x)>0.22){
//      geometry_msgs::msg::Twist cmd_vel;
//      cmd_vel.linear.y = 0.22;
//      cmd_vel.linear.x = 0.22;
//      cmd_vel.linear.z = 0;
//      cmd_vel.angular.z = 2.84;
//      cmd_vel.angular.y = 0;
//      cmd_vel.angular.x = 0;
//      vel_->publish(cmd_vel);
//    }
    else{
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.y = vel_y;
      cmd_vel.linear.x = vel_x;
      cmd_vel.linear.z = 0;

      cmd_vel.angular.z = vel_theta;
      cmd_vel.angular.y = 0;
      cmd_vel.angular.x = 0;
      vel_->publish(cmd_vel);
      RCLCPP_INFO(this->get_logger(), "x: '%f',/n y: '%f',/n theta: '%f',/n",
      xE, yE, msg->pose.pose.position.y );
      }
    }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Regulator>());
  rclcpp::shutdown();
  return 0;
}
