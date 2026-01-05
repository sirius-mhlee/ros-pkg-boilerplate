#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.hpp"

class CustomSubscriber : public rclcpp::Node {
  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    class const_info {
      public:
        static constexpr const char* NODE_NAME{"custom_subscriber"};
        
        static constexpr const char* IMAGE_TOPIC{"/front_stereo_camera/left/image_raw"};
        static constexpr const char* SCAN_TOPIC{"/scan"};

        static constexpr const char* WINDOW_NAME{"Stereo Left Image Raw"};
    };
    
  public:
    CustomSubscriber()
    : rclcpp::Node(const_info::NODE_NAME) {
      auto qos = rclcpp::SensorDataQoS();

      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        const_info::IMAGE_TOPIC, qos,
        std::bind(&CustomSubscriber::onImage, this, std::placeholders::_1));

      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        const_info::SCAN_TOPIC, qos,
        std::bind(&CustomSubscriber::onScan, this, std::placeholders::_1));

      cv::namedWindow(const_info::WINDOW_NAME, cv::WINDOW_NORMAL);

      RCLCPP_INFO(get_logger(),
      "Subscribed topics:\n  image_topic='%s'\n  scan_topic='%s'",
      const_info::IMAGE_TOPIC, const_info::SCAN_TOPIC);
    }

    ~CustomSubscriber() override {
        cv::destroyWindow(const_info::WINDOW_NAME);
    }

  private:
    void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
      cv_bridge::CvImageConstPtr cv_ptr_bgr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      cv::imshow(const_info::WINDOW_NAME, cv_ptr_bgr->image);
      cv::waitKey(1);

      RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Image: stamp=%.3f, size=%ux%u, encoding=%s",
      msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9,
      msg->width, msg->height, msg->encoding.c_str());
    }

    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Scan: stamp=%.3f, rays=%zu, angle=[%.3f, %.3f], inc=%.5f, range=[%.2f, %.2f]",
      msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9,
      msg->ranges.size(),
      msg->angle_min, msg->angle_max, msg->angle_increment,
      msg->range_min, msg->range_max);
    }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomSubscriber>());
  rclcpp::shutdown();
  return 0;
}
