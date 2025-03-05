#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

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
            "/odom", qos, std::bind(&control::topic_callback, this, _1)); // _1 means the function will allow one argument

        publisher_cmdvel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void topic_callback(const nav_msgs::msg::Odometry & msg)
    {
        auto current_dist = msg.pose.pose.position.x;

        // Log the received message IF THIS IS NOT SHOWING, BE SURE THE TOPIC EXISTS IN THE SPECIFIC TERMINAL
        RCLCPP_INFO(this->get_logger(), "Dist from start: %f", current_dist);
        
        // Instantiate a Twist for control command and defining desired distance
        auto twist_msg = geometry_msgs::msg::Twist();
        float desired_dist = 1.5; // meters
        float desired_angle = 0; // meters

        // Calculate proportional control command which is a twist around the z axis
        int Kp_angle = 1;
        float Kp_dist = .1;  

        // twist_msg.angular.z = Kp_angle*msg.angular.z; // angle is held in the x object of msg
        twist_msg.linear.x = 0.1*(desired_dist - current_dist); // distance is held in the y object of msg

        // Only give a distance command if an object is detected (non-zero angle) and if the object is within 0.75m
        // if (msg.angular.z == 0 || msg.linear.x > 0.75){
        //     twist_msg.linear.x = 0;
        // }

        RCLCPP_INFO(this->get_logger(), "Twist: %f", twist_msg.linear.x);

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