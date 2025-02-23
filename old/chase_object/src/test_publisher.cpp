#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

class TestPublisher : public rclcpp::Node
{
public:
    TestPublisher() : Node("test_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/obj_coords", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TestPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        auto msg = geometry_msgs::msg::Point();
        msg.x = 1.0;
        msg.y = 2.0;
        msg.z = 3.0;

        RCLCPP_INFO(this->get_logger(), "Publishing Point: x = %.2f, y = %.2f, z = %.2f",
                    msg.x, msg.y, msg.z);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPublisher>());
    rclcpp::shutdown();
    return 0;
}
