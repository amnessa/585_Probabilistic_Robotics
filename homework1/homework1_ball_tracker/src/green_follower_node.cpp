#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>

class GreenFollower : public rclcpp::Node {
public:
  GreenFollower() : Node("green_follower") {
    image_width_  = declare_parameter<int>("image_width", 640);
    goal_dist_    = declare_parameter<double>("goal_dist", 1.5);
    k_ang_        = declare_parameter<double>("k_ang", 1.2);
    k_lin_        = declare_parameter<double>("k_lin", 0.8);
    max_lin_      = declare_parameter<double>("max_lin", 0.8);
    max_ang_      = declare_parameter<double>("max_ang", 1.2);
    search_omega_ = declare_parameter<double>("search_omega", 0.6);
    lost_timeout_ = declare_parameter<double>("lost_timeout", 0.5);

    sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/target_pixel_coords", 10,
      [this](const geometry_msgs::msg::Point & p){
        last_px_ = p;
        last_stamp_ = now();
        have_target_ = true;
      });

    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_out", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
              std::bind(&GreenFollower::onTimer, this));
  }

private:
  void onTimer() {
    geometry_msgs::msg::Twist cmd;
    const bool fresh = have_target_ && (now() - last_stamp_).seconds() <= lost_timeout_;

    if (!fresh) {
      cmd.angular.z = search_omega_;  // rotate to search
      cmd.linear.x = 0.0;
      pub_->publish(cmd);
      return;
    }

    const double cx = 0.5 * image_width_;
    const double ex = (last_px_.x - cx) / cx; // [-1,1]
    double wz = -k_ang_ * ex;
    if (wz >  max_ang_) wz =  max_ang_;
    if (wz < -max_ang_) wz = -max_ang_;

    double vx = k_lin_ * (last_px_.z - goal_dist_);
    if (vx < 0.0) vx = 0.0;
    if (vx > max_lin_) vx = max_lin_;

    cmd.linear.x = vx;
    cmd.angular.z = wz;
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