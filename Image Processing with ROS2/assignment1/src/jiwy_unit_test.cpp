#include <cstdio>
#include <chrono>
#include <memory>
#include <cstdlib>
#include <ctime>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

using namespace std::chrono_literals;

class JiwyUnitTest : public rclcpp::Node
{
public:
    JiwyUnitTest() : Node("jiwy_unit_test")
    {
        //subscription_cog_ = this->create_subscription<asdfr_interfaces::msg::Point2>("light_cog", 10, std::bind(&JiwyUnitTest::cog_callback, this, std::placeholders::_1));
        publisher_set_point_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);
        timer_ = this->create_wall_timer(33ms, std::bind(&JiwyUnitTest::timer_callback, this));
        cog_x_ = 0;
        cog_y_ = 0;
    }

private:
    //void cog_callback(const asdfr_interfaces::msg::Point2::SharedPtr msg) // Subscriber call_back function for /light_cog topic
    //{
      //  cog_x_ = msg->x;
      //  cog_y_ = msg->y;
    //}

    void timer_callback() // Publisher call_back function for /setpoint topic
    {
        // Seed the random number generator with the current time
        std::srand(std::time(nullptr));        
        auto message = asdfr_interfaces::msg::Point2();
        message.x = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 1.6f - 0.8f; // Generate a random float between -0.8 and 0.8 (based on x limits)
        message.y = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 1.2f - 0.6f; // Generate a random float between -0.6 and 0.6 (based on the y limits)
        publisher_set_point_->publish(message);
        // print value
        RCLCPP_INFO(this->get_logger(), "X: %f", message.x);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr subscription_cog_;
    rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr publisher_set_point_;
    int cog_x_, cog_y_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JiwyUnitTest>());
    rclcpp::shutdown();
    return 0;
}
