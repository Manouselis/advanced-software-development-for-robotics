#include <cstdio>
#include <chrono>
#include <memory>
#include <cstdlib>
#include <ctime>

#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

using namespace std::chrono_literals;

class JiwySequenceController : public rclcpp::Node
{
public:
    JiwySequenceController() : Node("jiwy_sequence_controller")
    {
        subscription_cog_ = this->create_subscription<asdfr_interfaces::msg::Point2>("light_cog", 10, std::bind(&JiwySequenceController::cog_callback, this, std::placeholders::_1));
        subscription_is_light_on_ = this->create_subscription<std_msgs::msg::Bool>("light_status", 10, std::bind(&JiwySequenceController::light_status_callback, this, std::placeholders::_1));
        // Create subscription to image to get width
        webcam_input_ = this->create_subscription<sensor_msgs::msg::Image>("image", 10, std::bind(&JiwySequenceController::webcam_topic_callback, this, std::placeholders::_1));
         
        publisher_set_point_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);
        timer_ = this->create_wall_timer(33ms, std::bind(&JiwySequenceController::timer_callback, this));
        
        cog_x_ = 0;
        cog_y_ = 0;
        
        x_limit_ = 0.8;
        y_limit_ = 0.6;
        
        // Initialize image_width and image_length to non-zero values
        image_width = 300;
        image_length = 300;
    }

private:
    void cog_callback(const asdfr_interfaces::msg::Point2::SharedPtr msg) // Subscriber call_back function for /light_cog topic
    {
        cog_x_ = msg->x;
        cog_y_ = msg->y;       
        //cog_x = (int)floor( (cog_x - (-x_limit_)) / (2*x_limit_) * msg->width);
        //cog_y = (int)floor( (cog_y - (-y_limit_)) / (2*y_limit_) * msg->height);
    }
    void webcam_topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) // Subscriber call_back function for /webcam_input topic
    {
    	image_width = msg->width;
    	image_length = msg->height;
    }
    void light_status_callback(const std_msgs::msg::Bool::SharedPtr msg) // Subscriber call_back function for /light_status topic
    {
    	is_light_on = msg->data;
    }
    void timer_callback() // Publisher call_back function for /setpoint topic
    {        
        auto message = asdfr_interfaces::msg::Point2();
        // If no light is detected, then make camera look straight (i.e. (0,0))
        message.x = 0.0;
        message.y = 0.0;
        if (is_light_on) 
        {//transform variables cog_x and cog_y from pixel locations to pan and tilt radian angles
    	message.x = (cog_x_*2*x_limit_)/(image_width) - x_limit_;
    	message.y = -((cog_y_*2*y_limit_)/(image_length) - y_limit_); // Multiplied by -1 because the direction of y is inverted in jiwy_simulator (see line 145 in jiwy_simulator.cpp)
        }
        publisher_set_point_->publish(message);
        // print value
        RCLCPP_INFO(this->get_logger(), "X: %f", message.x);
        // RCLCPP_INFO(this->get_logger(), "x_limit: %f", x_limit_);
        // RCLCPP_INFO(this->get_logger(), "y_limit: %f", y_limit_);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr publisher_set_point_;
    rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr subscription_cog_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_is_light_on_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr webcam_input_;
    
    int cog_x_, cog_y_;
    // double x_limit_ = 0.8;
    // double y_limit_ = 0.6;
    double x_limit_, y_limit_;
    double image_width, image_length;
    bool is_light_on = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JiwySequenceController>());
    rclcpp::shutdown();
    return 0;
}
