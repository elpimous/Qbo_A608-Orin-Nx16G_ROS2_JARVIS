#include "ball_tracking_cpp/red_ball_tracker.hpp"
#include <opencv2/imgproc.hpp>
#include <chrono>

using std::placeholders::_1;

RedBallTracker::RedBallTracker()
: Node("red_ball_tracker_cpp"),
  lower_red_h_(0), lower_red_s_(120), lower_red_v_(70),
  upper_red_h_(10), upper_red_s_(255), upper_red_v_(255),
  min_radius_(10), max_radius_(100), blur_kernel_(9),
  loss_timeout_(2.0), kf_(4, 2), state_(4, 1, CV_32F), meas_(2, 1, CV_32F),
  kalman_initialized_(false), last_r_(0), reset_sent_(false)
{
  // Déclaration des paramètres avec valeurs par défaut
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

  // Récupération initiale
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

  // Initialiser last_detection_time_ à maintenant
  last_detection_time_ = this->now();

  // Pré-allouer le kernel morphologique (5x5)
  morph_kernel_ = cv::Mat::ones(5, 5, CV_8U);

  // Callback pour mise à jour dynamique des paramètres
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&RedBallTracker::paramCallback, this, std::placeholders::_1));

  // Subscriber image avec QoS
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/inverted_eye_image", rclcpp::QoS(1), std::bind(&RedBallTracker::imageCallback, this, _1));

  // Publisher ROI
  roi_pub_ = this->create_publisher<sensor_msgs::msg::RegionOfInterest>(
    "/target_roi", rclcpp::QoS(1));

  // Publisher debug image en tant que sensor_msgs/Image
  debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/red_ball_tracking_debug", rclcpp::QoS(1));

  // Publisher reset pour repositionner la tête au neutre
  reset_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/reset_head_neutral", rclcpp::QoS(1));

  RCLCPP_INFO(this->get_logger(), "RedBallTracker initialized (loss_timeout=%.2f s)", loss_timeout_);
}

rcl_interfaces::msg::SetParametersResult
RedBallTracker::paramCallback(const std::vector<rclcpp::Parameter> & params)
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
      if (bk % 2 == 0) bk += 1;
      if (bk < 1) bk = 1;
      blur_kernel_ = bk;
    } else if (name == "loss_timeout") {
      loss_timeout_ = p.as_double();
      RCLCPP_INFO(this->get_logger(), "Param %s updated to %.2f", name.c_str(), loss_timeout_);
      // Reset tracking variables
      last_detection_time_ = this->now();
      reset_sent_ = false;
      continue;
    } else {
      continue;
    }
    RCLCPP_INFO(this->get_logger(), "Param %s updated", name.c_str());
  }
  return result;
}

void RedBallTracker::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Timestamp courant
  rclcpp::Time now = this->get_clock()->now();
  double dt = 0.0;
  if (kalman_initialized_) {
    dt = (now - last_time_).seconds();
    if (dt <= 0) dt = 1e-3;
  }
  last_time_ = now;

  // Convertir en cv::Mat BGR
  cv::Mat cv_img;
  try {
    cv_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");
    cv_img = cv_ptr_->image;
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "CvBridge exception: %s", e.what());
    return;
  }

  int x = 0, y = 0, r = 0;
  bool found = detectRedBall(cv::Mat(cv_img), x, y, r);

  // Mettre à jour last_detection_time_ si trouvé
  if (found) {
    last_detection_time_ = now;
    if (reset_sent_) {
      // Si un reset avait été envoyé, indiquer fin de reset
      std_msgs::msg::Bool msg_reset;
      msg_reset.data = false;
      reset_pub_->publish(msg_reset);
      reset_sent_ = false;
    }
  }

  // Vérifier perte prolongée
  if (!found) {
    double since = (now - last_detection_time_).seconds();
    if (since >= loss_timeout_ && !reset_sent_) {
      // Publier commande de reset neutre
      std_msgs::msg::Bool msg_reset;
      msg_reset.data = true;
      reset_pub_->publish(msg_reset);
      RCLCPP_INFO(this->get_logger(), "Ball lost >%.2f s, publishing reset head neutral", loss_timeout_);
      reset_sent_ = true;
    }
  }

  // Traitement Kalman et ROI comme avant
  if (!kalman_initialized_) {
    if (found) {
      kf_.transitionMatrix = cv::Mat::eye(4, 4, CV_32F);
      state_.at<float>(0) = x;
      state_.at<float>(1) = y;
      state_.at<float>(2) = 0;
      state_.at<float>(3) = 0;
      kf_.measurementMatrix = cv::Mat::zeros(2, 4, CV_32F);
      kf_.measurementMatrix.at<float>(0,0) = 1.0f;
      kf_.measurementMatrix.at<float>(1,1) = 1.0f;
      cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-2));
      cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-1));
      cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1));
      kf_.statePost = state_;
      kalman_initialized_ = true;
      last_r_ = r;
    }
  } else {
    kf_.transitionMatrix = cv::Mat::eye(4, 4, CV_32F);
    kf_.transitionMatrix.at<float>(0,2) = dt;
    kf_.transitionMatrix.at<float>(1,3) = dt;
    cv::Mat prediction = kf_.predict();
    float pred_x = prediction.at<float>(0);
    float pred_y = prediction.at<float>(1);

    if (found) {
      meas_.at<float>(0) = x;
      meas_.at<float>(1) = y;
      cv::Mat estimated = kf_.correct(meas_);
      float est_x = estimated.at<float>(0);
      float est_y = estimated.at<float>(1);
      x = static_cast<int>(std::round(est_x));
      y = static_cast<int>(std::round(est_y));
      last_r_ = r;
    } else {
      x = static_cast<int>(std::round(pred_x));
      y = static_cast<int>(std::round(pred_y));
    }
  }

  // Publier ROI
  sensor_msgs::msg::RegionOfInterest roi;
  int img_w = cv_img.cols;
  int img_h = cv_img.rows;
  if (kalman_initialized_ && found) {
    int use_r = last_r_;
    int x0 = std::max(0, x - use_r);
    int y0 = std::max(0, y - use_r);
    int w = 2 * use_r;
    int h = 2 * use_r;
    if (x0 + w > img_w) w = img_w - x0;
    if (y0 + h > img_h) h = img_h - y0;
    roi.x_offset = x0;
    roi.y_offset = y0;
    roi.width = (w > 0 ? w : 0);
    roi.height = (h > 0 ? h : 0);
  } else if (kalman_initialized_ && !found) {
    // Si kalman initialisé mais pas de mesure, on peut publier ROI vide ou rester sur dernière position filtrée
    roi.x_offset = 0;
    roi.y_offset = 0;
    roi.width = 0;
    roi.height = 0;
  } else if (found) {
    int x0 = std::max(0, x - r);
    int y0 = std::max(0, y - r);
    int w = 2 * r;
    int h = 2 * r;
    if (x0 + w > img_w) w = img_w - x0;
    if (y0 + h > img_h) h = img_h - y0;
    roi.x_offset = x0;
    roi.y_offset = y0;
    roi.width = (w > 0 ? w : 0);
    roi.height = (h > 0 ? h : 0);
    last_r_ = r;
  } else {
    roi.x_offset = 0;
    roi.y_offset = 0;
    roi.width = 0;
    roi.height = 0;
  }
  roi.do_rectify = false;
  roi_pub_->publish(roi);

  // Publier image debug
  cv::Mat dbg = cv_img.clone();
  if (kalman_initialized_ && found) {
    drawTracking(dbg, roi.x_offset + last_r_, roi.y_offset + last_r_, last_r_);
  } else if (found) {
    drawTracking(dbg, x, y, r);
  }
  cv_bridge::CvImage out_msg;
  out_msg.header = msg->header;
  out_msg.encoding = "bgr8";
  out_msg.image = dbg;
  debug_pub_->publish(*out_msg.toImageMsg());
}

bool RedBallTracker::detectRedBall(const cv::Mat &img, int &out_x, int &out_y, int &out_r)
{
  cv::Mat hsv;
  cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
  cv::Scalar lower1(lower_red_h_, lower_red_s_, lower_red_v_);
  cv::Scalar upper1(upper_red_h_, upper_red_s_, upper_red_v_);
  cv::Scalar lower2(170, lower_red_s_, lower_red_v_);
  cv::Scalar upper2(180, upper_red_s_, upper_red_v_);
  cv::Mat m1, m2, mask;
  cv::inRange(hsv, lower1, upper1, m1);
  cv::inRange(hsv, lower2, upper2, m2);
  cv::bitwise_or(m1, m2, mask);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, morph_kernel_);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morph_kernel_);
  int bk = blur_kernel_; if (bk % 2 == 0) bk += 1;
  cv::GaussianBlur(mask, mask, cv::Size(bk, bk), 0);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) return false;
  double max_area = 0.0; size_t idx = 0;
  for (size_t i = 0; i < contours.size(); ++i) {
    double a = cv::contourArea(contours[i]); if (a > max_area) { max_area = a; idx = i; }
  }
  cv::Point2f center; float radius;
  cv::minEnclosingCircle(contours[idx], center, radius);
  if (radius >= min_radius_ && radius <= max_radius_) {
    out_x = static_cast<int>(std::round(center.x));
    out_y = static_cast<int>(std::round(center.y));
    out_r = static_cast<int>(std::round(radius));
    return true;
  }
  return false;
}

void RedBallTracker::drawTracking(cv::Mat &img, int x, int y, int r)
{
  if (r > 0) {
    cv::circle(img, cv::Point(x, y), r, cv::Scalar(255, 255, 255), 2);
    cv::circle(img, cv::Point(x, y), 3, cv::Scalar(255, 255, 255), -1);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RedBallTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}