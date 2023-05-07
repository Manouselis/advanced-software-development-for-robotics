#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class BrightnessNode : public rclcpp::Node
{
public:
    BrightnessNode() : Node("brightness_node")
    {
        this->declare_parameter("threshold", 128);          // parameter type is inferred from the default value (128), so it is int
        this->declare_parameter("conversion_mode", "gray"); // parameter to store conversion mode for brightness calculation, default value is "gray"
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>("image", 10, std::bind(&BrightnessNode::image_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("light_status", 10);
        timer_ = this->create_wall_timer(33ms, std::bind(&BrightnessNode::timer_callback, this));
        brightness_ = 0.0;
    }
private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) // Subscriber call_back function
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // convert image to grayscale, or the V layer from its HSV color space
        std::string conversion_mode;
        cv::Mat layer_img;
        conversion_mode = this->get_parameter("conversion_mode").get_parameter_value().get<std::string>();
        if (conversion_mode == "hsv")
        {
            layer_img = convertToHSV(cv_ptr->image);
        }
        else if (conversion_mode == "gray")
        {
            layer_img = convertToGrayscale(cv_ptr->image);
        }
        else
        {
            // default to grayscale conversion if mode is not specified correctly
            layer_img = convertToGrayscale(cv_ptr->image);
        }

        // calculate average pixel value (brightness) from the obtained single layer image
        brightness_ = cv::mean(layer_img)[0];

        // show the single layer image
        // cv::imshow("brightness_node image", cv_ptr->image);
        // cv::imshow("brightness_node image", layer_img);
        // cv::waitKey(1);
    }

    void timer_callback() // Publisher call_back function
    {
        auto message = std_msgs::msg::Bool();
        // RCLCPP_INFO(this->get_logger(), "'%d'", message.data);
        int threshold_parameter = this->get_parameter("threshold").get_parameter_value().get<int>();
        if (brightness_ > threshold_parameter)
        {
            message.data = true;
        }
        else{
            message.data = false;
        }
        // print the brightness value
        RCLCPP_INFO(this->get_logger(), "Brightness: %f", brightness_);

        // print the threshold value
        RCLCPP_INFO(this->get_logger(), "Current set threshold: %d", threshold_parameter);
        publisher_->publish(message);
    }

    cv::Mat convertToHSV(const cv::Mat &bgr_img) 
    {
        // convert image to HSV color space
        cv::Mat hsv_img;
        cv::cvtColor(bgr_img, hsv_img, cv::COLOR_BGR2HSV);

        // extract value channel
        std::vector<cv::Mat> channels;
        cv::split(hsv_img, channels);
        cv::Mat value = channels[2];

        return value;
    }

    cv::Mat convertToGrayscale(const cv::Mat &bgr_img) 
    {
        cv::Mat grayscale_img;
        cv::cvtColor(bgr_img, grayscale_img, cv::COLOR_BGR2GRAY);
        return grayscale_img;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    float brightness_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrightnessNode>());
    rclcpp::shutdown();
    return 0;
}
