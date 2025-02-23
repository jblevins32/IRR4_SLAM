#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <functional>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class LocationFinder : public rclcpp::Node
{
public:
    LocationFinder() : Node("get_object_location_pubsub") // This is basically the cpp version of __init__
    {
        RCLCPP_INFO(this->get_logger(), "Starting Location Finder Node");

        recent_scan = sensor_msgs::msg::LaserScan();
        recent_coords = geometry_msgs::msg::Point();

        subscription_1 = this->create_subscription<sensor_msgs::msg::LaserScan>
        (
         "/scan",
         rclcpp::QoS(10).best_effort(),
         std::bind(&LocationFinder::scan_callback,this,_1)
        );

        subscription_2 = this->create_subscription<geometry_msgs::msg::Point>
        (
         "/obj_coords",
         rclcpp::QoS(10).reliable(),
         std::bind(&LocationFinder::coords_callback,this,_1)
        );
        
        timer_ = this->create_wall_timer(20ms, std::bind(&LocationFinder::timer_callback, this));

        publisher_ = this->create_publisher<geometry_msgs::msg::Point>
        ("/obj_location",
         10
        );
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan & laser_msg) // message type & variable name
    {
        // RCLCPP_INFO(this->get_logger(), "Received Laser Scan");

        recent_scan = laser_msg;
    }

    void coords_callback(const geometry_msgs::msg::Point & coords_msg)
    {
        // RCLCPP_INFO(this->get_logger(),  "Received Object Coordinates %f", coords_msg.x);

        recent_coords = coords_msg;
    }

    // Combine the lidar and camera data every timer callback
    void timer_callback()
    {
        // Check to be sure data has been collected from both topics
        if (!recent_scan.ranges.empty())
        {

            // Lidar measures every 1.55 deg with 0 at center... so ~220 measurements
            // Camera 62.2 deg FoV so +- 30 degrees with 0 at center... so about 30/1.55 = 20 lidar measurements on each side of center
            // Recent_coords.x is (-1 to 1)

            // Use the above logic to scale the object x coordinates to indices for the lidar data
            int idx = std::round(recent_coords.x * 20);

            // If the index is negative, wrap it around
            if (idx < 0)
            {
                idx += 220;
            }
   
            // Assign the distance of the object to the y field of the obj_coords since it is unused. This way, we do not need to set up a dual subscriber in the next node
            recent_coords.y = recent_scan.ranges[idx];
            
            if (!std::isnan(recent_coords.y))
            {
                RCLCPP_INFO(this->get_logger(),"Publishing Object Angle %f (normed) and Object Range %f m",recent_coords.x, recent_coords.y);
                publisher_->publish(recent_coords);
            }
        }


    }

    // memory management in cpp
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_1;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_2;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    sensor_msgs::msg::LaserScan recent_scan;
    geometry_msgs::msg::Point recent_coords;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocationFinder>());
    rclcpp::shutdown();
    return 0;
}