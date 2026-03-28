#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <memory>
#include <vector>

using std::placeholders::_1;

class RedBallTracker : public rclcpp::Node
{
public:
  RedBallTracker()
  : Node("red_ball_tracker_cpp"),
    lower_red_h_(0), lower_red_s_(120), lower_red_v_(70),
    upper_red_h_(10), upper_red_s_(255), upper_red_v_(255),
    min_radius_(10), max_radius_(100), blur_kernel_(9),
    loss_timeout_(2.0),
    kf_(4, 2),
    state_(4, 1, CV_32F),
    meas_(2, 1, CV_32F),
    kalman_initialized_(false),
    last_r_(0),
    reset_sent_(false)
  {
    // Parameters
    this->declare_parameter("lower_red_h", lower_red_h_);
    this->declare_parameter("lower_red_s", lower_red_s_);
    this->declare_parameter("lower_red_v", lower_red_v_);
    this->declare_parameter("upper_red_h", upper_red_h_);
    this->declare_parameter("upper_red_s", upper_red_s_);
    this->declare_parameter("upper_red_v", upper_red_v_);
    this->declare_parameter("min_radius", min_radius_);
    this->declare_parameter("max_radius", max_radius_);
    this->declare_parameter("blur_kernel", blur_kernel_);
    this->declare_parameter("loss_timeout", loss_timeout_);

    this->get_parameter("lower_red_h", lower_red_h_);
    this->get_parameter("lower_red_s", lower_red_s_);
    this->get_parameter("lower_red_v", lower_red_v_);
    this->get_parameter("upper_red_h", upper_red_h_);
    this->get_parameter("upper_red_s", upper_red_s_);
    this->get_parameter("upper_red_v", upper_red_v_);
    this->get_parameter("min_radius", min_radius_);
    this->get_parameter("max_radius", max_radius_);
    this->get_parameter("blur_kernel", blur_kernel_);
    this->get_parameter("loss_timeout", loss_timeout_);

    last_detection_time_ = this->now();

    morph_kernel_ = cv::Mat::ones(5, 5, CV_8U);

    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&RedBallTracker::paramCallback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/inverted_eye_image", rclcpp::QoS(1),
      std::bind(&RedBallTracker::imageCallback, this, _1));

    roi_pub_ = this->create_publisher<sensor_msgs::msg::RegionOfInterest>(
      "/target_roi", rclcpp::QoS(1));

    debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/red_ball_tracking_debug", rclcpp::QoS(1));

    reset_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/reset_head_neutral", rclcpp::QoS(1));

    RCLCPP_INFO(this->get_logger(), "RedBallTracker initialized");
  }

private:
  rcl_interfaces::msg::SetParametersResult
  paramCallback(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &p : params) {
      const auto &name = p.get_name();

      if (name == "lower_red_h") lower_red_h_ = p.as_int();
      else if (name == "lower_red_s") lower_red_s_ = p.as_int();
      else if (name == "lower_red_v") lower_red_v_ = p.as_int();
      else if (name == "upper_red_h") upper_red_h_ = p.as_int();
      else if (name == "upper_red_s") upper_red_s_ = p.as_int();
      else if (name == "upper_red_v") upper_red_v_ = p.as_int();
      else if (name == "min_radius") min_radius_ = p.as_int();
      else if (name == "max_radius") max_radius_ = p.as_int();
      else if (name == "blur_kernel") {
        int bk = p.as_int();
        if (bk % 2 == 0) bk++;
        blur_kernel_ = bk;
      }
      else if (name == "loss_timeout") {
        loss_timeout_ = p.as_double();
      }
    }
    return result;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat img;

    try {
      cv_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");
      img = cv_ptr_->image;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error");
      return;
    }

    int x=0,y=0,r=0;
    bool found = detectRedBall(img, x, y, r);

    sensor_msgs::msg::RegionOfInterest roi;

    if (found) {
      roi.x_offset = x - r;
      roi.y_offset = y - r;
      roi.width = 2*r;
      roi.height = 2*r;
    } else {
      roi.width = 0;
      roi.height = 0;
    }

    roi_pub_->publish(roi);

    cv::Mat dbg = img.clone();
    if (found) drawTracking(dbg, x, y, r);

    cv_bridge::CvImage out;
    out.header = msg->header;
    out.encoding = "bgr8";
    out.image = dbg;

    debug_pub_->publish(*out.toImageMsg());
  }

  bool detectRedBall(const cv::Mat &img, int &out_x, int &out_y, int &out_r)
  {
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv,
      cv::Scalar(lower_red_h_, lower_red_s_, lower_red_v_),
      cv::Scalar(upper_red_h_, upper_red_s_, upper_red_v_),
      mask);

    cv::GaussianBlur(mask, mask, cv::Size(blur_kernel_, blur_kernel_), 0);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return false;

    auto c = *std::max_element(contours.begin(), contours.end(),
      [](auto &a, auto &b) {
        return cv::contourArea(a) < cv::contourArea(b);
      });

    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(c, center, radius);

    if (radius > min_radius_ && radius < max_radius_) {
      out_x = center.x;
      out_y = center.y;
      out_r = radius;
      return true;
    }
    return false;
  }

  void drawTracking(cv::Mat &img, int x, int y, int r)
  {
    cv::circle(img, cv::Point(x,y), r, cv::Scalar(0,255,0), 2);
  }

  // Variables
  int lower_red_h_, lower_red_s_, lower_red_v_;
  int upper_red_h_, upper_red_s_, upper_red_v_;
  int min_radius_, max_radius_, blur_kernel_;
  double loss_timeout_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  cv::Mat morph_kernel_;
  cv_bridge::CvImagePtr cv_ptr_;

  cv::KalmanFilter kf_;
  cv::Mat state_, meas_;

  bool kalman_initialized_;
  int last_r_;
  bool reset_sent_;
  rclcpp::Time last_detection_time_, last_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RedBallTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}