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
#include "asdfr_interfaces/msg/point2.hpp"

using namespace std::chrono_literals;

class LightPositionNode : public rclcpp::Node
{
public:
    LightPositionNode() : Node("light_position_node")
    {
        this->declare_parameter("pixel_threshold", 200); // parameter type is inferred from the default value (128), so it is int
        subscription_image_ = this->create_subscription<sensor_msgs::msg::Image>("image", 10, std::bind(&LightPositionNode::image_callback, this, std::placeholders::_1));
        subscription_light_ = this->create_subscription<std_msgs::msg::Bool>("light_status", 10, std::bind(&LightPositionNode::light_status_callback, this, std::placeholders::_1));
        publisher_cog_ = this->create_publisher<asdfr_interfaces::msg::Point2>("light_cog", 10);
        timer_ = this->create_wall_timer(33ms, std::bind(&LightPositionNode::timer_callback, this));
        is_light_on_ = false;
        cog_x_ = 0;
        cog_y_ = 0;
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) // Subscriber call_back function for /webcam_input topic
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

        // convert image to grayscale
        cv::Mat gray_img;
        cv::cvtColor(cv_ptr->image, gray_img, cv::COLOR_BGR2GRAY);

        // show histogram image
        show_histogram(gray_img);

        int pixel_threshold_value = this->get_parameter("pixel_threshold").get_parameter_value().get<int>();
        // only if light spot detected by the brightness_node, then determine COG
        if (is_light_on_)
        {
            // convert image to binary image using the pixel threshold parameter
            cv::Mat binary_image; // output binary image
            cv::threshold(gray_img, binary_image, pixel_threshold_value, 255, cv::THRESH_BINARY);

            // calculate image moments
            cv::Moments moments = cv::moments(binary_image, true);

            // over all pixel positions x,y:
            // m00 = sum(I(x,y))
            // m10 = sum(x* I(x,y))
            // m01 = sum(y* I(x,y))
            // here I(x,y) is either 0 or 1 (binary image), so m00 equals the number of white pixels,
            // m10 equals the summation of the x coordinates of the white pixels,
            // and m01 equals the summation of the y coordinates of the white pixels

            // calculate cog position
            cog_x_ = moments.m10 / moments.m00;
            cog_y_ = moments.m01 / moments.m00;

            // draw a cricle at the cog position
            cv::Point center(cog_x_, cog_y_);
            int radius = 10;
            int thickness = 2;           // thickness of the circle outline
            cv::Scalar color(0, 0, 255); // color of the circle (in BGR)
            cv::circle(cv_ptr->image, center, radius, color, thickness);

            // write the COG position on the top left corner of the image
            cv::putText(cv_ptr->image, "Center: (" + std::to_string(cog_x_) + ", " + std::to_string(cog_y_) + ")",
                        cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }
        else
        {
            cog_x_ = 0;
            cog_y_ = 0;
        }
        // show image
        cv::imshow("COG", cv_ptr->image);
        cv::waitKey(1);
    }

    void light_status_callback(const std_msgs::msg::Bool &msg) // Subscriber call_back function for /light_status topic
    {
        is_light_on_ = msg.data;
        // RCLCPP_INFO(this->get_logger(), "Light: %d", is_light_on_);
    }

    void timer_callback() // Publisher call_back function for /light_cog topic
    {
        auto message = asdfr_interfaces::msg::Point2();
        message.x = cog_x_;
        message.y = cog_y_;
        publisher_cog_->publish(message);
    }

    void show_histogram(const cv::Mat &gray_img)
    {
        // Convert the grayscale image to a histogram
        cv::Mat hist;
        int histSize = 256;       // number of bins in the histogram
        float range[] = {0, 256}; // range of pixel values
        const float *histRange = {range};
        bool uniform = true, accumulate = false;
        cv::calcHist(&gray_img, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

        // Display the histogram
        int histWidth = 1024;
        int histHeight = 800;
        int binWidth = cvRound((double)histWidth / histSize);
        cv::Mat hist_img(histHeight, histWidth, CV_8UC1, cv::Scalar(0, 0, 0));
        cv::normalize(hist, hist, 0, hist_img.rows, cv::NORM_MINMAX, -1, cv::Mat());
        for (int i = 1; i < histSize; i++)
        {
            cv::line(hist_img, cv::Point(binWidth * (i - 1), histHeight - cvRound(hist.at<float>(i - 1))),
                     cv::Point(binWidth * (i), histHeight - cvRound(hist.at<float>(i))),
                     cv::Scalar(255, 255, 255), 2, 8, 0);
            if (i % 8 == 0)
            {
                std::ostringstream ss;
                ss << i;
                cv::putText(hist_img, ss.str(), cv::Point(binWidth * i - 10, histHeight - 10),
                            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            }
        }
        cv::imshow("Histogram", hist_img);
        cv::waitKey(1);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_light_;
    rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr publisher_cog_;
    bool is_light_on_;
    int cog_x_, cog_y_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightPositionNode>());
    rclcpp::shutdown();
    return 0;
}
