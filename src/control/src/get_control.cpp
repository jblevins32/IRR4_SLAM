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
    
    // Define current task
    int task = 1;

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
          double current_yaw = quaternionToYaw(recent_odom.pose.pose.orientation);

          // Get current scan data in front of robot from odom
          auto current_scan = recent_scan.ranges;
          std::vector<float> current_scan_fov;

          // isolate the front 90 degrees of lidar scan
          current_scan_fov.insert(current_scan_fov.end(), current_scan.begin() + 300, current_scan.end());
          current_scan_fov.insert(current_scan_fov.end(), current_scan.begin(),current_scan.begin() + 60); 

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
          float obstacle_too_close = 0.1; // back up if too close

          // Set linear and angular speeds
          float lin_vel = 0.1;
          float ang_vel = 0.4;

          // Waypoint errors
          float error_threshold = 0.01;
          float error_threshold_rot = 0.05;

          // Current errors based on state machine, defined by tasks completed thus far
          float dist_x;
          float dist_y;
          float desired_yaw;

          // Task 1: go to waypoint 1
          if (task == 1)
          {
            dist_x = desired_x_1 - current_dist_x;
            dist_y = desired_y_1 - current_dist_y;
            desired_yaw = std::atan2(dist_y, dist_x);
          }
          // Task 2: rotate to waypoint 2
          else if (task == 2)
          {
            desired_yaw = desired_theta_1;
            dist_x = 100;
            dist_y = 100;
          }
          // Task 3: go to waypoint 2
          else if (task == 3)
          {
            dist_x = desired_x_2 - current_dist_x;
            dist_y = desired_y_2 - current_dist_y;
            desired_yaw = std::atan2(dist_y, dist_x);
          }
          // Task 4: rotate to waypoint 3
          else if (task == 4)
          {
            desired_yaw = desired_theta_2;
            dist_x = 100;
            dist_y = 100;
          }
          // Task 5: go to waypoint 3
          else if (task == 5)
          {
            dist_x = desired_x_3 - current_dist_x;
            dist_y = desired_y_3 - current_dist_y;
            desired_yaw = std::atan2(dist_y, dist_x);
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), "All tasks completed!");
            twist_msg.linear.x = 0;
            twist_msg.angular.z = 0;
            publisher_cmdvel->publish(twist_msg);
            return;
          }

          int state = 0;
          
          // STATE MACHINE START
          if (std::abs(dist_x) < error_threshold && std::abs(dist_y) < error_threshold) // At the goal?
          {
            twist_msg.linear.x = 0;
            task += 1; // mark a task completed to switch to the next state
          } 

          else if (task != 2 && task != 4) // Move towards the goal
          {
            // Get min distance object in front of robot
            float scan_min = *std::min_element(current_scan_fov.begin(), current_scan_fov.end());

            // move forward towards the goal unless an object is placed in front of it
            if (scan_min > obstacle_too_close)
            {
              // state 1: go to goal
              state = 1;
              twist_msg.linear.x = lin_vel; // Move forward
            }
            else
            {
              // State 2: back it up, too close to obstacle! (obstacle was probably placed right in front)
              state = 2;
              twist_msg.linear.x = -lin_vel; // Back it up!
            }

            // rotate the robot towards the goal
            float yaw_error = getYawError(current_yaw, desired_yaw);
            if (yaw_error > error_threshold_rot)
            {
              twist_msg.angular.z = ang_vel;
            }
            else if (-yaw_error > error_threshold_rot)
            {
              twist_msg.angular.z = -ang_vel;
            }

            // State 3: wall follow: Check if an obstacle is close - only within the FoV of front of robot
            if (scan_min < obstacle_dist && scan_min > obstacle_too_close)
            {
              // Switch to wall following
              state = 3;

              // Rotate to go right around the object. Rotate until the close distance is at about degree 270 (left of straight)
              if (current_scan[260] > obstacle_dist){
                twist_msg.linear.x = 0;
                twist_msg.angular.z = ang_vel; // This should keep the obstacle to the left of the robot at this obstacle_dist
              }
            }
          }

          // State 4: rotate when task calls for it.
          else 
          {
            state = 4;
            twist_msg.linear.x = 0;
            float yaw_error = getYawError(current_yaw, desired_yaw);

            if (yaw_error > error_threshold_rot)
            {
              twist_msg.angular.z = ang_vel;
            }
            else if (-yaw_error > error_threshold_rot)
            {
              twist_msg.angular.z = -ang_vel;
            }
            else
            {
              task += 1; // mark a task completed to switch to the next state
            }
          }

          // Publish the command velocity
          RCLCPP_INFO(this->get_logger(), "Task %d, State %d, y error %f, x error %f, desired rot %f, actual rot %f", task, state, dist_y, dist_x, desired_yaw, current_yaw);
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

    float getYawError(float current_yaw, float desired_yaw)
    {
      float yaw_error = desired_yaw - current_yaw;
      if (yaw_error > M_PI)
      {
        yaw_error -= 2*M_PI;
      }
      else if (yaw_error < -M_PI)
      {
        yaw_error += 2*M_PI;
      }
      return yaw_error;
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