#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include <algorithm>

using std::placeholders::_1;
using namespace std::chrono_literals;

// This node gets the odom topic and controls the robot to a defined location
class control : public rclcpp::Node
{
public:
    control():Node("get_control") // This is initializing the node
    {
      RCLCPP_INFO(this->get_logger(), "Starting Control Node");

      rclcpp::QoS qos = rclcpp::QoS(10).best_effort(); // Quality of Service
      subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom_fixed", qos, std::bind(&control::odom_callback, this, _1)); // _1 means the function will allow one argument
      
      subscription_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", qos, std::bind(&control::scan_callback, this, _1)); // _1 means the function will allow one argument

      publisher_cmdvel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

      timer_ = this->create_wall_timer(20ms, std::bind(&control::timer_callback, this));
    }

private:
    
    // Waypoint flags for state machine
    bool wayPoint1_flag = true;
    bool wayPoint1_rot_flag = false;
    bool wayPoint2_flag = false;
    bool wayPoint2_rot_flag = false;
    bool wayPoint3_flag = false;

    // Define sensor messages
    sensor_msgs::msg::LaserScan recent_scan;
    nav_msgs::msg::Odometry recent_odom;
    bool odom_received = false;
    bool scan_received = false;

    // callback to capture recent scan
    void scan_callback(const sensor_msgs::msg::LaserScan & laser_msg) // message type & variable name
    {
        // RCLCPP_INFO(this->get_logger(), "Received Laser Scan");

        recent_scan = laser_msg;
        scan_received = true;
    }

    // callback to capture recent odom
    void odom_callback(const nav_msgs::msg::Odometry & odom_msg)
    {
        // RCLCPP_INFO(this->get_logger(),  "Received Odom");

        recent_odom = odom_msg;
        odom_received = true;
    }

    void timer_callback()
    {
        // Ensure we have data from sensors
        if (scan_received && odom_received)
        {

          // Get current pose from odom
          auto current_dist_x = recent_odom.pose.pose.position.x;
          auto current_dist_y = recent_odom.pose.pose.position.y;
          double current_theta_z = quaternionToYaw(recent_odom.pose.pose.orientation);
          RCLCPP_INFO(this->get_logger(), "yaw: %f", current_theta_z);

          // Get current scan data in front of robot from odom
          auto current_scan = recent_scan.ranges;
          std::vector<float> current_scan_fov;

          // isolate the front 90 degrees of lidar scan
          current_scan_fov.insert(current_scan_fov.end(), current_scan.begin() + 315, current_scan.end());
          current_scan_fov.insert(current_scan_fov.end(), current_scan.begin(),current_scan.begin() + 45); 

          // Instantiate a Twist for control command and defining desired distance
          auto twist_msg = geometry_msgs::msg::Twist();

          // Define waypoints: should get these from the txt file.
          float desired_x_1 = 1.5; // meters
          float desired_y_1 = 0; // meters
          float desired_theta_1 = 1.5707; // radians
          float desired_x_2 = 1.5; // meters
          float desired_y_2 = 1.4; // meters
          float desired_theta_2 = 3.1416; // radians
          float desired_x_3 = 0; // meters
          float desired_y_3 = 1.4; // meters

          float obstacle_dist = 0.2; // meters to stay from obstacles
          bool wall_follow_flag = false;

          // Set linear and angular speeds
          float lin_vel = 0.1;
          float ang_vel = 0.4;

          // Waypoint errors
          auto dist_2_rot = std::abs(desired_theta_1 - current_theta_z);
          auto dist_3_rot = std::abs(desired_theta_2 - current_theta_z);

          // Current errors based on state machine
          float dist_x;
          float dist_y;

          if (wayPoint1_flag)
          {
            dist_x = desired_x_1 - current_dist_x;
            dist_y = desired_y_1 - current_dist_y;
          }
          else if (wayPoint2_flag)
          {
            dist_x = desired_x_2 - current_dist_x;
            dist_y = desired_y_2 - current_dist_y;
          }
          else if (wayPoint3_flag)
          {
            dist_x = desired_x_3 - current_dist_x;
            dist_y = desired_y_3 - current_dist_y;
          }

          // Desired orientation to the goal
          float desired_orientation = std::atan2(dist_y, dist_x);

          RCLCPP_INFO(this->get_logger(), "y %f, x %f, desired %f, actual %f", dist_y, dist_x, desired_orientation, current_theta_z);
          
          // RCLCPP_INFO(this->get_logger(), "Closest obstacle: %f", *std::min_element(current_scan_fov.begin(), current_scan_fov.end()));

          // STATE MACHINE START
          if (std::abs(dist_x < 0.01)) // At the goal?
          {
            twist_msg.linear.x = 0;
            wayPoint1_rot_flag = true;
            wayPoint1_flag = false;
          } 
          else // Move towards the goal
          {
            // state 1: go to goal
            if (wall_follow_flag == false)
            {
              twist_msg.linear.x = lin_vel; // move forward towards the goal
              // RCLCPP_INFO(this->get_logger(), "yes");

              // rotate the robot towards the goal if it ever gets off
              if (desired_orientation - current_theta_z > 0.01)
              {
                twist_msg.angular.z = ang_vel;
              }
              else if (current_theta_z - desired_orientation > 0.01)
              {
                twist_msg.angular.z = -ang_vel;
              }
            }

            // State 2: wall follow: Check if an obstacle is close - only within the FoV of front of robot
            if (*std::min_element(current_scan_fov.begin(), current_scan_fov.end()) < obstacle_dist)
            {
              // Switch to wall following
              wall_follow_flag = true;
              twist_msg.linear.x = 0;

              // Rotate to go right around the object. Rotate until the close distance is at about degree 270 (left of straight)
              if (current_scan[270] > obstacle_dist){
                twist_msg.angular.z = ang_vel; // This should keep the obstacle to the left of the robot at this obstacle_dist
              }
              else
              {
                twist_msg.angular.z = 0; 
                twist_msg.linear.x = lin_vel; // move forward
              }

              // Check straight line to see if we can go back to state 1
              // if ()
            }
          }

          // // Rotate to face waypoint 2
          // else if (wayPoint1_rot_flag) 
          // {
          //   if (dist_2_rot < 0.01) 
          //   {
          //       twist_msg.angular.z = 0;
          //       wayPoint2_flag = true;
          //       wayPoint1_rot_flag = false;
          //   } 
          //   else
          //   {
          //       twist_msg.angular.z = .25;
          //       RCLCPP_INFO(this->get_logger(), "%f from facing toward waypoint 2", dist_2_rot);
          //   }
          // }

          // // Velocity to waypoint 2
          // else if (wayPoint2_flag) 
          // {
          //   if (dist_x < 0.1) 
          //   {
          //     twist_msg.linear.x = 0;
          //     wayPoint2_rot_flag = true;
          //     wayPoint2_flag = false;
          //   } 
          //   else
          //   {
          //     twist_msg.linear.x = .25;
          //     RCLCPP_INFO(this->get_logger(), "%f from waypoint 2", dist_2);
          //   }
          // }

          // // Rotate to face waypoint 3
          // else if (wayPoint2_rot_flag) 
          // {
          //   if (dist_3_rot < 0.01) 
          //   {
          //       twist_msg.angular.z = 0;
          //       wayPoint3_flag = true;
          //       wayPoint2_rot_flag = false;
          //   } 
          //   else
          //   {
          //       twist_msg.angular.z = .25;
          //       RCLCPP_INFO(this->get_logger(), "%f from facing toward waypoint 3", dist_3_rot);
          //   }
          // }

          // // Velocity to waypoint 3
          // else if (wayPoint3_flag) 
          // {
          //   if (dist_3 < 0.1) 
          //   {
          //     twist_msg.linear.x = 0;
          //     wayPoint3_flag = false;
          //   } 
          //   else
          //   {
          //     twist_msg.linear.x = .25;
          //     RCLCPP_INFO(this->get_logger(), "%f from waypoint 3", dist_3);
          //   }
          // }
          // // Publish the command velocity
          publisher_cmdvel->publish(twist_msg);
      } 
    }

    double quaternionToYaw(const geometry_msgs::msg::Quaternion &orientation)
    {
      tf2::Quaternion tf2_quat;
      tf2::fromMsg(orientation, tf2_quat);

      double roll, pitch, yaw;
      tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);

      return yaw;
    }

    // memory management in cpp
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmdvel;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_scan;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // Setup ros2 system
    rclcpp::spin(std::make_shared<control>());
    rclcpp::shutdown();
    return 0;
};