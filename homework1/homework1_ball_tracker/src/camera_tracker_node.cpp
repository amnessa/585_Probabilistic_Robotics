#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <algorithm>
#include <limits>
#include <cstdint>

using sensor_msgs::msg::Image;
using sensor_msgs::msg::CameraInfo;
using geometry_msgs::msg::Point;

class CameraTrackerNode : public rclcpp::Node {
public:
  CameraTrackerNode() : Node("camera_tracker_node") {
    // Params (HSV tuned for blue; adjust in params.yaml)
    hsv_lower_ = declare_parameter<std::vector<int64_t>>("hsv_lower", {90, 80, 40});
    hsv_upper_ = declare_parameter<std::vector<int64_t>>("hsv_upper", {130, 255, 255});
    min_area_  = declare_parameter<int>("min_area", 200);
    ema_alpha_ = declare_parameter<double>("ema_alpha", 0.35);
    depth_win_ = declare_parameter<int>("depth_median_window", 5);
    depth_default_ = declare_parameter<double>("depth_default", 0.8);
    depth_scale_   = declare_parameter<double>("depth_scale", 0.001);

    // Subscriptions (sync only image + depth)
    img_sub_.subscribe(this, "/camera/image", rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, "/camera/depth_image", rmw_qos_profile_sensor_data);
    // info_sub_.subscribe(this, "/camera/camera_info", rmw_qos_profile_sensor_data); // not needed

    sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), img_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&CameraTrackerNode::cb, this,
                                      std::placeholders::_1, std::placeholders::_2));

    pub_ = create_publisher<Point>("/target_pixel_coords", 10);
  }

private:
  // Updated callback: remove CameraInfo arg (unused)
  void cb(const Image::ConstSharedPtr &img_msg,
          const Image::ConstSharedPtr &depth_msg) {
    cv::Mat bgr;
    try { bgr = cv_bridge::toCvShare(img_msg, "bgr8")->image; }
    catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv img: %s", e.what());
      return;
    }

    cv::Mat depth;
    try {
      if (depth_msg->encoding == "16UC1") depth = cv_bridge::toCvShare(depth_msg, "16UC1")->image;
      else depth = cv_bridge::toCvShare(depth_msg, "32FC1")->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv depth: %s", e.what());
      return;
    }

    cv::Mat hsv, mask;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    const cv::Scalar lo((int)hsv_lower_[0], (int)hsv_lower_[1], (int)hsv_lower_[2]);
    const cv::Scalar hi((int)hsv_upper_[0], (int)hsv_upper_[1], (int)hsv_upper_[2]);
    cv::inRange(hsv, lo, hi, mask);
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) { have_px_ = false; return; }

    size_t best_i = 0; double best_area = 0.0;
    for (size_t i = 0; i < contours.size(); ++i) {
      double a = cv::contourArea(contours[i]);
      if (a > best_area) { best_area = a; best_i = i; }
    }
    if (best_area < (double)min_area_) { have_px_ = false; return; }

    cv::Moments m = cv::moments(contours[best_i]);
    double cx = (m.m00 > 1e-6) ? (m.m10 / m.m00) : (bgr.cols * 0.5);
    double cy = (m.m00 > 1e-6) ? (m.m01 / m.m00) : (bgr.rows * 0.5);

    const int w = std::max(1, depth_win_);
    const int x0 = std::clamp((int)std::round(cx) - w/2, 0, depth.cols - 1);
    const int y0 = std::clamp((int)std::round(cy) - w/2, 0, depth.rows - 1);
    const int x1 = std::clamp(x0 + w, 0, depth.cols);
    const int y1 = std::clamp(y0 + w, 0, depth.rows);

    std::vector<float> vals; vals.reserve(w*w);
    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        float d_m = std::numeric_limits<float>::quiet_NaN();
        if (depth_msg->encoding == "16UC1") {
          uint16_t raw = depth.at<uint16_t>(y, x);
          if (raw > 0) d_m = (float)raw * (float)depth_scale_;
        } else {
          float raw = depth.at<float>(y, x);
          if (std::isfinite(raw) && raw > 0.0f) d_m = raw;
        }
        if (std::isfinite(d_m) && d_m > 0.05f) vals.push_back(d_m);
      }
    }
    double d = depth_default_;
    if (!vals.empty()) {
      std::nth_element(vals.begin(), vals.begin() + vals.size()/2, vals.end());
      d = vals[vals.size()/2];
      if (have_px_) d = ema_alpha_ * d + (1.0 - ema_alpha_) * last_depth_;
    }

    last_depth_ = d; have_px_ = true;
    Point out; out.x = cx; out.y = cy; out.z = d;
    pub_->publish(out);
  }

  // params and members
  std::vector<int64_t> hsv_lower_, hsv_upper_;
  int min_area_{200}, depth_win_{5};
  double ema_alpha_{0.35}, depth_default_{0.8}, depth_scale_{0.001};

  message_filters::Subscriber<Image> img_sub_, depth_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Synchronizer> sync_;

  rclcpp::Publisher<Point>::SharedPtr pub_;
  bool have_px_{false};
  double last_depth_{0.8};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraTrackerNode>());
  rclcpp::shutdown();
  return 0;
}