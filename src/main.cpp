#include <iostream>
#include <chrono>
#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/msg/marker.hpp>
#include "uv_obstacle_detector/UV_detector.hpp"
#include "uv_obstacle_detector/kalman_filter.hpp"

using namespace cv;
using namespace std;

class MyDetector : public rclcpp::Node
{
public:
    MyDetector() : Node("my_detector")
    {

        std::string image_topic = getParam("image_topic", std::string("/front_top/depth/image_raw"));
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS(), 
                                                                        std::bind(&MyDetector::run, this, std::placeholders::_1));
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);
    }

    template<typename ParamT>
    ParamT getParam(const std::string param_name, ParamT default_value)
    {
        ParamT output{};
        this->declare_parameter(param_name, default_value);
        this->get_parameter(param_name, output);
        return output;
    }

    void run(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        // image conversion
        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat depth = cv_ptr->image;
        // detect
        this->uv_detector.readdata(depth);
        this->uv_detector.detect();
        this->uv_detector.track();
        this->uv_detector.display_depth();
        this->uv_detector.display_U_map();
        this->uv_detector.display_bird_view();
        cout << this->uv_detector.bounding_box_B.size() << endl;
        // rviz visualization
        for (int i = 0; i < this->uv_detector.bounding_box_B.size(); i++)
        {
            Point2f obs_center = Point2f(this->uv_detector.bounding_box_B[i].x + this->uv_detector.bounding_box_B[i].width,
                                         this->uv_detector.bounding_box_B[i].y + this->uv_detector.bounding_box_B[i].height);
            cout << obs_center << endl;
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    UVdetector uv_detector;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create an object of class MyDetector
    auto my_detector_node = std::make_shared<MyDetector>();

    rclcpp::spin(my_detector_node);
    rclcpp::shutdown();

    return 0;
}
