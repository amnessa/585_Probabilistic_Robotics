#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>

class GreenFollower : public rclcpp::Node {
public:
  GreenFollower() : Node("green_follower") {
    image_width_  = this->declare_parameter<int>("image_width", 640);
    goal_dist_    = this->declare_parameter<double>("goal_dist", 1.5);
    k_ang_        = this->declare_parameter<double>("k_ang", 1.2);   // rad/s for full-width error
    k_lin_        = this->declare_parameter<double>("k_lin", 0.8);
    max_lin_      = this->declare_parameter<double>("max_lin", 0.8);
    max_ang_      = this->declare_parameter<double>("max_ang", 1.2);
    search_omega_ = this->declare_parameter<double>("search_omega", 0.6);
    lost_timeout_ = this->declare_parameter<double>("lost_timeout", 0.5);

    sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/target_pixel_coords", 10,
      [this](const geometry_msgs::msg::Point & p){
        last_px_ = p;
        last_stamp_ = this->now();
        have_target_ = true;
      });

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_out", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&GreenFollower::onTimer, this));
  }

private:
  void onTimer() {
    geometry_msgs::msg::Twist cmd;

    const bool fresh = have_target_ && (this->now() - last_stamp_).seconds() <= lost_timeout_;

    if (!fresh) {
      // Fail policy: rotate in place to search
      cmd.angular.z = search_omega_;
      cmd.linear.x = 0.0;
    } else {
      // Heading control: center pixel x to image center
      const double cx = 0.5 * image_width_;
      const double ex = (last_px_.x - cx) / cx;     // [-1, 1] if inside image
      double wz = -k_ang_ * ex;
      if (wz >  max_ang_) wz =  max_ang_;
      if (wz < -max_ang_) wz = -max_ang_;

      // Range control using depth (z field carries depth in meters)
      double vx = k_lin_ * (last_px_.z - goal_dist_);
      if (vx < 0.0) vx = 0.0;                       // don’t drive backwards
      if (vx > max_lin_) vx = max_lin_;

      cmd.linear.x = vx;
      cmd.angular.z = wz;
    }

    pub_->publish(cmd);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int image_width_;
  double goal_dist_, k_ang_, k_lin_, max_lin_, max_ang_, search_omega_, lost_timeout_;
  geometry_msgs::msg::Point last_px_;
  rclcpp::Time last_stamp_{0,0,RCL_ROS_TIME};
  bool have_target_{false};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GreenFollower>());
  rclcpp::shutdown();
  return 0;
}