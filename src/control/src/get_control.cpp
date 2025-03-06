#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include <iostream>

using std::placeholders::_1;


// This node gets the odom topic and controls the robot to a defined location
class control : public rclcpp::Node
{
public:
    control():Node("get_control") // This is initializing the node
    {
        RCLCPP_INFO(this->get_logger(), "Starting Control Node");

        rclcpp::QoS qos = rclcpp::QoS(10).best_effort(); // Quality of Service
        subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_fixed", qos, std::bind(&control::topic_callback, this, _1)); // _1 means the function will allow one argument

        publisher_cmdvel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
        
    bool wayPoint0_flag = true;
    bool wayPoint1_flag = false;
    bool wayPoint2_flag = false;
    bool wayPoint3_flag = false;
    bool wayPoint4_flag = false;
    bool wayPoint5_flag = false;

    void topic_callback(const nav_msgs::msg::Odometry & msg)
    {
        auto current_dist_x = msg.pose.pose.position.x;
        auto current_dist_y = msg.pose.pose.position.y;
        auto current_theta_z = msg.pose.pose.orientation.z;

        // Log the received message IF THIS IS NOT SHOWING, BE SURE THE TOPIC EXISTS IN THE SPECIFIC TERMINAL
        // RCLCPP_INFO(this->get_logger(), "Dist from start: (%f,%f)", current_dist_x, current_dist_y);
        
        // Instantiate a Twist for control command and defining desired distance
        auto twist_msg = geometry_msgs::msg::Twist();

        float desired_x_1 = 1.5; // meters

        float desired_theta_1 = 0.707; // radians

        float desired_y_2 = 1.4; // meters

        float desired_theta_2 = 1; // radians

        float desired_x_3 = 0; // meters
        
        // Calculate proportional control command which is a twist around the z axis
        float Kp_dist = .1;  

        // twist_msg.linear.x = Kp_dist*(desired_x - current_dist_x); 
        // twist_msg.linear.y = Kp_dist*(desired_y - current_dist_y); 

        // RCLCPP_INFO(this->get_logger(), "Twist: (%f,%f)", twist_msg.linear.x,twist_msg.linear.y);

        // Velocity to waypoint 1
        auto dist_1 = std::abs(desired_x_1 - current_dist_x);
        auto dist_2_rot = std::abs(desired_theta_1 - current_theta_z);
        auto dist_2 = std::abs(desired_y_2 - current_dist_y);
        auto dist_3_rot = std::abs(desired_theta_2 - current_theta_z);
        auto dist_3 = std::abs(desired_x_3 - current_dist_x);
        
        if (wayPoint0_flag)
        {
          if (dist_1 < 0.01) 
          {
              twist_msg.linear.x = 0;
              wayPoint1_flag = true;
              wayPoint0_flag = false;
          } 
          else
          {
              twist_msg.linear.x = .25;
              RCLCPP_INFO(this->get_logger(), "%f from waypoint 1", dist_1);
          }
        }

        // Rotate to face waypoint 2
        else if (wayPoint1_flag) 
        {
          if (dist_2_rot < 0.01) 
          {
              twist_msg.angular.z = 0;
              wayPoint2_flag = true;
              wayPoint1_flag = false;
          } 
          else
          {
              twist_msg.angular.z = .25;
              RCLCPP_INFO(this->get_logger(), "%f from facing toward waypoint 2", dist_2_rot);
          }
        }

        // Velocity to waypoint 2
        else if (wayPoint2_flag) 
        {
          if (dist_2 < 0.1) 
          {
            twist_msg.linear.x = 0;
            wayPoint3_flag = true;
            wayPoint2_flag = false;
          } 
          else
          {
            twist_msg.linear.x = .25;
            RCLCPP_INFO(this->get_logger(), "%f from waypoint 2", dist_2);
          }
        }

        // Rotate to face waypoint 3
        else if (wayPoint3_flag) 
        {
          if (dist_3_rot < 0.01) 
          {
              twist_msg.angular.z = 0;
              wayPoint4_flag = true;
              wayPoint3_flag = false;
          } 
          else
          {
              twist_msg.angular.z = .25;
              RCLCPP_INFO(this->get_logger(), "%f from facing toward waypoint 3", dist_3_rot);
          }
        }

        // Velocity to waypoint 3
        else if (wayPoint4_flag) 
        {
          if (dist_3 < 0.1) 
          {
            twist_msg.linear.x = 0;
            wayPoint5_flag = true;
            wayPoint4_flag = false;
          } 
          else
          {
            twist_msg.linear.x = .25;
            RCLCPP_INFO(this->get_logger(), "%f from waypoint 3", dist_3);
          }
        }
        // Publish the command velocity
        publisher_cmdvel->publish(twist_msg);
    } 

    // memory management in cpp
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmdvel;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // Setup ros2 system
    rclcpp::spin(std::make_shared<control>());
    rclcpp::shutdown();
    return 0;
};