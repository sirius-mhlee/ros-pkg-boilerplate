#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "cv_bridge/cv_bridge.hpp"

class CustomSubscriber : public rclcpp::Node {
  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_{};
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_{};
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_{};

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_{};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{};
    rclcpp::TimerBase::SharedPtr map_render_timer_{};

    nav_msgs::msg::MapMetaData map_info_{};
    cv::Mat map_image_{};

    class ConstInfo {
      public:
        static constexpr int INFO_LOG_PERIOD_MS{2000};
        static constexpr int WARN_LOG_PERIOD_MS{2000};

        static constexpr const char* NODE_NAME{"custom_subscriber"};
        
        static constexpr const char* IMAGE_TOPIC{"/front_stereo_camera/left/image_raw"};
        static constexpr const char* SCAN_TOPIC{"/scan"};
        static constexpr const char* MAP_TOPIC{"/map"};

        static constexpr const char* IMAGE_WINDOW_NAME{"Stereo Left Image Raw"};
        static constexpr const char* MAP_WINDOW_NAME{"Occupancy Grid Map"};

        static constexpr const char* MAP_FRAME_NAME{"map"};
        static constexpr const char* ROBOT_FRAME_NAME{"base_link"};

        static constexpr int MAP_RENDER_PERIOD_MS{100};
    };
    
  public:
    CustomSubscriber()
    : rclcpp::Node(ConstInfo::NODE_NAME) {
      auto sensor_qos = rclcpp::SensorDataQoS();
      auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        ConstInfo::IMAGE_TOPIC, sensor_qos,
        std::bind(&CustomSubscriber::onImage, this, std::placeholders::_1));

      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        ConstInfo::SCAN_TOPIC, sensor_qos,
        std::bind(&CustomSubscriber::onScan, this, std::placeholders::_1));

      map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        ConstInfo::MAP_TOPIC, map_qos,
        std::bind(&CustomSubscriber::onMap, this, std::placeholders::_1));

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      map_render_timer_ = create_wall_timer(
        std::chrono::milliseconds(ConstInfo::MAP_RENDER_PERIOD_MS),
        std::bind(&CustomSubscriber::renderMap, this));

      cv::namedWindow(ConstInfo::IMAGE_WINDOW_NAME, cv::WINDOW_NORMAL);
      cv::namedWindow(ConstInfo::MAP_WINDOW_NAME, cv::WINDOW_NORMAL);

      RCLCPP_INFO(get_logger(),
      "Subscribed topics:\n  image_topic='%s'\n  scan_topic='%s'\n  map_topic='%s'",
      ConstInfo::IMAGE_TOPIC, ConstInfo::SCAN_TOPIC, ConstInfo::MAP_TOPIC);
    }

    ~CustomSubscriber() override {
        cv::destroyWindow(ConstInfo::IMAGE_WINDOW_NAME);
        cv::destroyWindow(ConstInfo::MAP_WINDOW_NAME);
    }

  private:
    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
      cv_bridge::CvImageConstPtr cv_ptr_bgr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      cv::imshow(ConstInfo::IMAGE_WINDOW_NAME, cv_ptr_bgr->image);
      cv::waitKey(1);

      RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), ConstInfo::INFO_LOG_PERIOD_MS,
      "Image: stamp=%.3f, size=%ux%u, encoding=%s",
      rclcpp::Time(msg->header.stamp).seconds(),
      msg->width, msg->height, msg->encoding.c_str());
    }

    void onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
      RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), ConstInfo::INFO_LOG_PERIOD_MS,
      "Scan: stamp=%.3f, rays=%zu, angle=[%.3f, %.3f], inc=%.5f, range=[%.2f, %.2f]",
      rclcpp::Time(msg->header.stamp).seconds(),
      msg->ranges.size(),
      msg->angle_min, msg->angle_max, msg->angle_increment,
      msg->range_min, msg->range_max);
    }

    void onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
      const auto width = msg->info.width;
      const auto height = msg->info.height;

      cv::Mat map_image(height, width, CV_8UC1);
      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          const auto occupancy = msg->data[y * width + x];
          auto& pixel = map_image.at<uint8_t>(height - 1 - y, x);

          if (occupancy < 0) {
            pixel = 127;
          } else {
            const auto clamped_occupancy = std::clamp<int>(occupancy, 0, 100);
            pixel = static_cast<uint8_t>(255 - (clamped_occupancy * 255 / 100));
          }
        }
      }

      map_info_ = msg->info;
      map_image_ = map_image;

      RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), ConstInfo::INFO_LOG_PERIOD_MS,
      "Map: stamp=%.3f, size=%ux%u, resolution=%.3f, origin=[%.3f, %.3f]",
      rclcpp::Time(msg->header.stamp).seconds(),
      width, height, msg->info.resolution,
      msg->info.origin.position.x, msg->info.origin.position.y);
    }

    double getYaw(const geometry_msgs::msg::Quaternion& quaternion) const {
      const auto siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
      const auto cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
      return std::atan2(siny_cosp, cosy_cosp);
    }

    void renderMap() {
      if (map_image_.empty()) {
        return;
      }

      cv::Mat display_image;
      cv::cvtColor(map_image_, display_image, cv::COLOR_GRAY2BGR);

      try {
        const auto map_to_robot = tf_buffer_->lookupTransform(
          ConstInfo::MAP_FRAME_NAME, ConstInfo::ROBOT_FRAME_NAME, tf2::TimePointZero);

        const auto& translation = map_to_robot.transform.translation;
        const auto origin_yaw = getYaw(map_info_.origin.orientation);
        const auto dx = translation.x - map_info_.origin.position.x;
        const auto dy = translation.y - map_info_.origin.position.y;
        const auto cos_yaw = std::cos(-origin_yaw);
        const auto sin_yaw = std::sin(-origin_yaw);
        const auto map_x = (cos_yaw * dx - sin_yaw * dy) / map_info_.resolution;
        const auto map_y = (sin_yaw * dx + cos_yaw * dy) / map_info_.resolution;
        const auto pixel_x = static_cast<int>(std::lround(map_x));
        const auto pixel_y = static_cast<int>(map_info_.height - 1) - static_cast<int>(std::lround(map_y));

        if (pixel_x < 0 || pixel_x >= static_cast<int>(map_info_.width)
          || pixel_y < 0 || pixel_y >= static_cast<int>(map_info_.height)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), ConstInfo::WARN_LOG_PERIOD_MS,
            "TF: robot is outside the map, position=[%.3f, %.3f]",
            translation.x, translation.y);
        } else {
          const cv::Point robot_center(pixel_x, pixel_y);
          const auto robot_yaw = getYaw(map_to_robot.transform.rotation) - origin_yaw;
          const cv::Point robot_direction(
            pixel_x + static_cast<int>(std::lround(std::cos(robot_yaw) * 10.0)),
            pixel_y - static_cast<int>(std::lround(std::sin(robot_yaw) * 10.0)));

          cv::circle(display_image, robot_center, 6, cv::Scalar(0, 0, 255), 2);
          cv::line(display_image, robot_center, robot_direction, cv::Scalar(0, 0, 255), 2);
        }
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), ConstInfo::WARN_LOG_PERIOD_MS,
          "TF lookup failed: %s",
          ex.what());
      }

      cv::imshow(ConstInfo::MAP_WINDOW_NAME, display_image);
      cv::waitKey(1);
    }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomSubscriber>());
  rclcpp::shutdown();
  return 0;
}
