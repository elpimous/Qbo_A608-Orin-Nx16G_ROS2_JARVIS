#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <string>
#include <cmath>
#include <algorithm>
#include <random>

// ─── PID réutilisable ───────────────────────────────────────────────
struct PID
{
    double kp = 0.0, ki = 0.0, kd = 0.0;
    double integral    = 0.0;
    double prev_error  = 0.0;
    double max_integral = 1.0;

    void reset()
    {
        integral   = 0.0;
        prev_error = 0.0;
    }

    double compute(double error, double dt)
    {
        if (dt <= 0.0) dt = 1e-3;
        integral += error * dt;
        integral  = std::clamp(integral, -max_integral, max_integral);
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

// ─── Filtre passe-bas (lissage exponentiel) ─────────────────────────
struct LowPass
{
    double value   = 0.0;
    double alpha   = 0.3;
    bool   initialized = false;

    double filter(double input)
    {
        if (!initialized) {
            value = input;
            initialized = true;
            return value;
        }
        value += alpha * (input - value);
        return value;
    }

    void reset() { initialized = false; }
};

// ═══════════════════════════════════════════════════════════════════
//  HeadTrackerNode
// ═══════════════════════════════════════════════════════════════════
class HeadTrackerNode : public rclcpp::Node
{
public:
    HeadTrackerNode()
    : Node("head_tracker_node"),
      rng_(std::random_device{}())
    {
        // ── Déclaration des paramètres ──────────────────────────────
        this->declare_parameter("image_width",  640);
        this->declare_parameter("image_height", 480);

        this->declare_parameter("pan_joint_name",  std::string("head_pan_joint"));
        this->declare_parameter("tilt_joint_name", std::string("head_tilt_joint"));

        // PID position
        this->declare_parameter("pid_pan_kp",  0.6);
        this->declare_parameter("pid_pan_ki",  0.02);
        this->declare_parameter("pid_pan_kd",  0.08);
        this->declare_parameter("pid_tilt_kp", 0.6);
        this->declare_parameter("pid_tilt_ki", 0.02);
        this->declare_parameter("pid_tilt_kd", 0.08);

        // Vitesse proportionnelle à l'erreur
        this->declare_parameter("vel_gain",      4.0);
        this->declare_parameter("min_vel",       0.3);
        this->declare_parameter("max_pan_vel",   5.0);
        this->declare_parameter("max_tilt_vel",  3.0);

        // Lissage
        this->declare_parameter("smoothing_alpha", 0.20);

        // Seuil de changement minimum
        this->declare_parameter("cmd_threshold_rad", 0.005);

        // Limites angulaires
        this->declare_parameter("min_pan_angle",  -1.22);
        this->declare_parameter("max_pan_angle",   1.22);
        this->declare_parameter("min_tilt_angle", -0.50);
        this->declare_parameter("max_tilt_angle",  0.70);

        // Perte de cible
        this->declare_parameter("loss_timeout",    2.0);
        this->declare_parameter("update_rate_hz", 30.0);

        // Deadzone pixels
        this->declare_parameter("deadzone_px", 12.0);

        // Recherche
        this->declare_parameter("search_pan_vel",     0.3);
        this->declare_parameter("search_tilt_vel",    0.2);
        this->declare_parameter("search_min_pan",     -0.5);
        this->declare_parameter("search_max_pan",      0.5);
        this->declare_parameter("search_min_tilt",    -0.3);
        this->declare_parameter("search_max_tilt",     0.4);
        this->declare_parameter("search_interval_s",   2.0);

        // ── Lecture initiale ────────────────────────────────────────
        loadParams();

        // ── Timestamps ──────────────────────────────────────────────
        last_detection_time_ = this->now();
        last_update_time_    = this->now();
        last_search_time_    = this->now();

        // ── Subscribers ─────────────────────────────────────────────
        roi_sub_ = this->create_subscription<sensor_msgs::msg::RegionOfInterest>(
            "/target_roi", rclcpp::QoS(1),
            std::bind(&HeadTrackerNode::roiCallback, this, std::placeholders::_1));

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(10),
            std::bind(&HeadTrackerNode::jointStateCallback, this, std::placeholders::_1));

        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::QoS(1),
            std::bind(&HeadTrackerNode::cameraInfoCallback, this, std::placeholders::_1));

        // ── Publishers ──────────────────────────────────────────────
        cmd_pub_   = this->create_publisher<sensor_msgs::msg::JointState>("/cmd_joints", 1);
        reset_pub_ = this->create_publisher<std_msgs::msg::Bool>("/reset_head_neutral", 1);

        // ── Timer principal ─────────────────────────────────────────
        double rate_hz = this->get_parameter("update_rate_hz").as_double();
        auto period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = this->create_wall_timer(period,
            std::bind(&HeadTrackerNode::timerCallback, this));

        // ── Callback paramètres dynamiques ──────────────────────────
        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&HeadTrackerNode::paramCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "HeadTrackerNode ready — PID pan(%.2f/%.2f/%.2f) tilt(%.2f/%.2f/%.2f) "
            "vel_gain=%.1f smooth=%.2f @ %.0f Hz",
            pid_pan_.kp, pid_pan_.ki, pid_pan_.kd,
            pid_tilt_.kp, pid_tilt_.ki, pid_tilt_.kd,
            vel_gain_, smoothing_alpha_, rate_hz);
    }

private:
    // ── Subscribers / Publishers ────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr       joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr       cam_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr          cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   reset_pub_;
    rclcpp::TimerBase::SharedPtr                                        timer_;

    // ── Paramètres caméra ───────────────────────────────────────────
    int    image_width_  = 640;
    int    image_height_ = 480;
    double fx_ = 0.0, fy_ = 0.0, cx_cam_ = 0.0, cy_cam_ = 0.0;
    bool   cam_info_received_ = false;

    // ── État courant des joints (lu depuis /joint_states) ───────────
    double current_pan_   = 0.0;
    double current_tilt_  = 0.0;
    bool   joint_received_ = false;   // ✅ true dès le premier /joint_states reçu

    // ── Noms des joints ─────────────────────────────────────────────
    std::string pan_joint_name_  = "head_pan_joint";
    std::string tilt_joint_name_ = "head_tilt_joint";

    // ── PID (corrige la position) ───────────────────────────────────
    PID pid_pan_;
    PID pid_tilt_;

    // ── Vitesse (proportionnelle à l'erreur) ────────────────────────
    double vel_gain_       = 4.0;
    double min_vel_        = 0.3;
    double max_pan_vel_    = 5.0;
    double max_tilt_vel_   = 3.0;

    // ── Lissage ─────────────────────────────────────────────────────
    double  smoothing_alpha_  = 0.20;
    LowPass lp_pan_pos_, lp_tilt_pos_;
    LowPass lp_pan_vel_, lp_tilt_vel_;

    // ── Seuil de commande & dernière position envoyée ───────────────
    double cmd_threshold_rad_ = 0.005;
    double last_sent_pan_     = 0.0;   // ✅ sert aussi de fallback si pas de /joint_states
    double last_sent_tilt_    = 0.0;

    // ── Limites ─────────────────────────────────────────────────────
    double min_pan_angle_  = -1.22;
    double max_pan_angle_  =  1.22;
    double min_tilt_angle_ = -0.5;
    double max_tilt_angle_ =  0.7;

    // ── Deadzone ────────────────────────────────────────────────────
    double deadzone_px_ = 12.0;

    // ── Tracking state ──────────────────────────────────────────────
    rclcpp::Time last_detection_time_;
    double       loss_timeout_   = 2.0;
    bool         target_visible_ = false;
    bool         reset_sent_     = false;

    // ── Search mode ─────────────────────────────────────────────────
    double search_pan_vel_    = 0.3;
    double search_tilt_vel_   = 0.2;
    double search_min_pan_    = -0.5;
    double search_max_pan_    =  0.5;
    double search_min_tilt_   = -0.3;
    double search_max_tilt_   =  0.4;
    double search_interval_s_ = 2.0;
    rclcpp::Time last_search_time_;
    std::mt19937 rng_;

    // ── Dernière ROI ────────────────────────────────────────────────
    double target_cx_ = 0.0;
    double target_cy_ = 0.0;
    bool   roi_fresh_ = false;

    // ── Timer dt ────────────────────────────────────────────────────
    rclcpp::Time last_update_time_;

    // ── Param callback handle ───────────────────────────────────────
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // ═════════════════════════════════════════════════════════════════
    //  Chargement des paramètres
    // ═════════════════════════════════════════════════════════════════
    void loadParams()
    {
        image_width_  = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();

        pan_joint_name_  = this->get_parameter("pan_joint_name").as_string();
        tilt_joint_name_ = this->get_parameter("tilt_joint_name").as_string();

        pid_pan_.kp  = this->get_parameter("pid_pan_kp").as_double();
        pid_pan_.ki  = this->get_parameter("pid_pan_ki").as_double();
        pid_pan_.kd  = this->get_parameter("pid_pan_kd").as_double();
        pid_tilt_.kp = this->get_parameter("pid_tilt_kp").as_double();
        pid_tilt_.ki = this->get_parameter("pid_tilt_ki").as_double();
        pid_tilt_.kd = this->get_parameter("pid_tilt_kd").as_double();

        vel_gain_       = this->get_parameter("vel_gain").as_double();
        min_vel_        = this->get_parameter("min_vel").as_double();
        max_pan_vel_    = this->get_parameter("max_pan_vel").as_double();
        max_tilt_vel_   = this->get_parameter("max_tilt_vel").as_double();

        smoothing_alpha_   = this->get_parameter("smoothing_alpha").as_double();
        cmd_threshold_rad_ = this->get_parameter("cmd_threshold_rad").as_double();

        lp_pan_pos_.alpha  = smoothing_alpha_;
        lp_tilt_pos_.alpha = smoothing_alpha_;
        lp_pan_vel_.alpha  = smoothing_alpha_;
        lp_tilt_vel_.alpha = smoothing_alpha_;

        min_pan_angle_  = this->get_parameter("min_pan_angle").as_double();
        max_pan_angle_  = this->get_parameter("max_pan_angle").as_double();
        min_tilt_angle_ = this->get_parameter("min_tilt_angle").as_double();
        max_tilt_angle_ = this->get_parameter("max_tilt_angle").as_double();

        deadzone_px_     = this->get_parameter("deadzone_px").as_double();
        loss_timeout_    = this->get_parameter("loss_timeout").as_double();

        search_pan_vel_    = this->get_parameter("search_pan_vel").as_double();
        search_tilt_vel_   = this->get_parameter("search_tilt_vel").as_double();
        search_min_pan_    = this->get_parameter("search_min_pan").as_double();
        search_max_pan_    = this->get_parameter("search_max_pan").as_double();
        search_min_tilt_   = this->get_parameter("search_min_tilt").as_double();
        search_max_tilt_   = this->get_parameter("search_max_tilt").as_double();
        search_interval_s_ = this->get_parameter("search_interval_s").as_double();
    }

    // ═════════════════════════════════════════════════════════════════
    //  Callback paramètres dynamiques
    // ═════════════════════════════════════════════════════════════════
    rcl_interfaces::msg::SetParametersResult
    paramCallback(const std::vector<rclcpp::Parameter> & params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & p : params) {
            const auto & n = p.get_name();

            if      (n == "pid_pan_kp")       pid_pan_.kp       = p.as_double();
            else if (n == "pid_pan_ki")     { pid_pan_.ki       = p.as_double(); pid_pan_.reset(); }
            else if (n == "pid_pan_kd")       pid_pan_.kd       = p.as_double();
            else if (n == "pid_tilt_kp")      pid_tilt_.kp      = p.as_double();
            else if (n == "pid_tilt_ki")    { pid_tilt_.ki      = p.as_double(); pid_tilt_.reset(); }
            else if (n == "pid_tilt_kd")      pid_tilt_.kd      = p.as_double();
            else if (n == "vel_gain")         vel_gain_         = p.as_double();
            else if (n == "min_vel")          min_vel_          = p.as_double();
            else if (n == "max_pan_vel")      max_pan_vel_      = p.as_double();
            else if (n == "max_tilt_vel")     max_tilt_vel_     = p.as_double();
            else if (n == "smoothing_alpha") {
                smoothing_alpha_ = p.as_double();
                lp_pan_pos_.alpha = lp_tilt_pos_.alpha = smoothing_alpha_;
                lp_pan_vel_.alpha = lp_tilt_vel_.alpha = smoothing_alpha_;
            }
            else if (n == "cmd_threshold_rad") cmd_threshold_rad_ = p.as_double();
            else if (n == "min_pan_angle")    min_pan_angle_    = p.as_double();
            else if (n == "max_pan_angle")    max_pan_angle_    = p.as_double();
            else if (n == "min_tilt_angle")   min_tilt_angle_   = p.as_double();
            else if (n == "max_tilt_angle")   max_tilt_angle_   = p.as_double();
            else if (n == "deadzone_px")      deadzone_px_      = p.as_double();
            else if (n == "loss_timeout")     loss_timeout_     = p.as_double();
            else if (n == "search_pan_vel")   search_pan_vel_   = p.as_double();
            else if (n == "search_tilt_vel")  search_tilt_vel_  = p.as_double();
            else if (n == "search_min_pan")   search_min_pan_   = p.as_double();
            else if (n == "search_max_pan")   search_max_pan_   = p.as_double();
            else if (n == "search_min_tilt")  search_min_tilt_  = p.as_double();
            else if (n == "search_max_tilt")  search_max_tilt_  = p.as_double();
            else if (n == "search_interval_s") search_interval_s_ = p.as_double();
            else if (n == "image_width")      image_width_      = p.as_int();
            else if (n == "image_height")     image_height_     = p.as_int();
            else continue;

            RCLCPP_INFO(this->get_logger(), "Param %s updated", n.c_str());
        }
        return result;
    }

    // ═════════════════════════════════════════════════════════════════
    //  Callback ROI
    // ═════════════════════════════════════════════════════════════════
    void roiCallback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
    {
        if (msg->width == 0 || msg->height == 0) {
            roi_fresh_ = false;
            return;
        }

        target_cx_ = msg->x_offset + msg->width  / 2.0;
        target_cy_ = msg->y_offset + msg->height / 2.0;
        roi_fresh_ = true;

        last_detection_time_ = this->now();
        target_visible_ = true;

        if (reset_sent_) {
            std_msgs::msg::Bool rst;
            rst.data = false;
            reset_pub_->publish(rst);
            reset_sent_ = false;
            RCLCPP_INFO(this->get_logger(), "Target re-acquired, cancel reset");
        }
    }

    // ═════════════════════════════════════════════════════════════════
    //  Callback joint states
    //  ✅ Met à jour la position réelle lue depuis les servos
    // ═════════════════════════════════════════════════════════════════
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
            if (msg->name[i] == pan_joint_name_) {
                current_pan_ = msg->position[i];
                joint_received_ = true;
            }
            else if (msg->name[i] == tilt_joint_name_) {
                current_tilt_ = msg->position[i];
                joint_received_ = true;
            }
        }
    }

    // ═════════════════════════════════════════════════════════════════
    //  Callback camera info
    // ═════════════════════════════════════════════════════════════════
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (cam_info_received_) return;

        fx_     = msg->k[0];
        fy_     = msg->k[4];
        cx_cam_ = msg->k[2];
        cy_cam_ = msg->k[5];

        image_width_  = static_cast<int>(msg->width);
        image_height_ = static_cast<int>(msg->height);

        cam_info_received_ = true;
        RCLCPP_INFO(this->get_logger(),
            "CameraInfo: %dx%d  fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
            image_width_, image_height_, fx_, fy_, cx_cam_, cy_cam_);
    }

    // ═════════════════════════════════════════════════════════════════
    //  Timer — boucle de contrôle principale
    // ═════════════════════════════════════════════════════════════════
    void timerCallback()
    {
        rclcpp::Time now = this->now();
        double since_last = (now - last_detection_time_).seconds();

        // ✅ Warning si pas de joint_states (une seule fois)
        static bool warned = false;
        if (!joint_received_ && !warned && (now - last_update_time_).seconds() > 2.0) {
            RCLCPP_WARN(this->get_logger(),
                "No /joint_states received — using last commanded position as reference. "
                "Add joint_states publisher to qbo_dynamixel for better tracking.");
            warned = true;
        }

        if (roi_fresh_) {
            trackTarget(target_cx_, target_cy_);
            roi_fresh_ = false;
        }
        else if (since_last >= loss_timeout_) {
            if (!reset_sent_) {
                resetToNeutral();
                reset_sent_ = true;
            } else {
                searchTarget();
            }
            target_visible_ = false;
        }

        last_update_time_ = now;
    }

    // ═════════════════════════════════════════════════════════════════
    //  Tracking : pixels → angle → PID position + vitesse lissée
    //
    //  ✅ FIX : utilise la position réelle si /joint_states est dispo,
    //           sinon la dernière position commandée (last_sent_*)
    // ═════════════════════════════════════════════════════════════════
    void trackTarget(double cx, double cy)
    {
        rclcpp::Time now = this->now();
        double dt = (now - last_update_time_).seconds();
        if (dt <= 0.0) dt = 1e-3;

        // ── Erreur en pixels ──
        double img_cx = image_width_  / 2.0;
        double img_cy = image_height_ / 2.0;
        double err_px = cx - img_cx;
        double err_py = cy - img_cy;

        // ── Deadzone ──
        if (std::abs(err_px) < deadzone_px_ && std::abs(err_py) < deadzone_px_) {
            return;
        }

        // ── Conversion pixel → radians ──
        double err_pan_rad, err_tilt_rad;
        if (cam_info_received_ && fx_ > 0.0 && fy_ > 0.0) {
            err_pan_rad  = std::atan2(err_px, fx_);
            err_tilt_rad = std::atan2(err_py, fy_);
        } else {
            err_pan_rad  = err_px / (image_width_  / 2.0) * 0.52;
            err_tilt_rad = err_py / (image_height_ / 2.0) * 0.40;
        }

        // ── PID → correction de position (radians) ──
        double pan_correction  = pid_pan_.compute(-err_pan_rad, dt);
        double tilt_correction = pid_tilt_.compute(err_tilt_rad, dt);

        // ── ✅ Position de référence : réelle si dispo, sinon dernière commandée ──
        double ref_pan  = joint_received_ ? current_pan_  : last_sent_pan_;
        double ref_tilt = joint_received_ ? current_tilt_ : last_sent_tilt_;

        // ── Position cible = référence + correction PID ──
        double pan_raw  = std::clamp(ref_pan  + pan_correction,
                                     min_pan_angle_,  max_pan_angle_);
        double tilt_raw = std::clamp(ref_tilt + tilt_correction,
                                     min_tilt_angle_, max_tilt_angle_);

        // ── Vitesse proportionnelle à l'erreur ──
        double pan_vel_raw  = std::clamp(std::abs(err_pan_rad)  * vel_gain_,
                                         min_vel_, max_pan_vel_);
        double tilt_vel_raw = std::clamp(std::abs(err_tilt_rad) * vel_gain_,
                                         min_vel_, max_tilt_vel_);

        // ── Lissage passe-bas ──
        double pan_pos  = lp_pan_pos_.filter(pan_raw);
        double tilt_pos = lp_tilt_pos_.filter(tilt_raw);
        double pan_vel  = lp_pan_vel_.filter(pan_vel_raw);
        double tilt_vel = lp_tilt_vel_.filter(tilt_vel_raw);

        // ── N'envoie que si le changement est significatif ──
        double delta_pan  = std::abs(pan_pos - last_sent_pan_);
        double delta_tilt = std::abs(tilt_pos - last_sent_tilt_);

        if (delta_pan > cmd_threshold_rad_ || delta_tilt > cmd_threshold_rad_) {
            sendJointCommand(pan_pos, tilt_pos, pan_vel, tilt_vel);
            last_sent_pan_  = pan_pos;
            last_sent_tilt_ = tilt_pos;
        }

        RCLCPP_DEBUG(this->get_logger(),
            "Track: err(%.0f,%.0f)px → ref(%s: %.3f,%.3f) → cmd(%.3f,%.3f) vel(%.2f,%.2f)",
            err_px, err_py,
            joint_received_ ? "real" : "last_cmd",
            ref_pan, ref_tilt,
            pan_pos, tilt_pos, pan_vel, tilt_vel);
    }

    // ═════════════════════════════════════════════════════════════════
    //  Recherche : une nouvelle position toutes les N secondes
    // ═════════════════════════════════════════════════════════════════
    void searchTarget()
    {
        rclcpp::Time now = this->now();
        if ((now - last_search_time_).seconds() < search_interval_s_) return;
        last_search_time_ = now;

        std::uniform_real_distribution<double> pan_dist(search_min_pan_, search_max_pan_);
        std::uniform_real_distribution<double> tilt_dist(search_min_tilt_, search_max_tilt_);

        double pan  = pan_dist(rng_);
        double tilt = tilt_dist(rng_);

        sendJointCommand(pan, tilt, search_pan_vel_, search_tilt_vel_);
        last_sent_pan_  = pan;
        last_sent_tilt_ = tilt;

        pid_pan_.reset();
        pid_tilt_.reset();
        lp_pan_pos_.reset();
        lp_tilt_pos_.reset();
        lp_pan_vel_.reset();
        lp_tilt_vel_.reset();
    }

    // ═════════════════════════════════════════════════════════════════
    //  Reset neutre
    // ═════════════════════════════════════════════════════════════════
    void resetToNeutral()
    {
        sendJointCommand(0.0, 0.0, 0.3, 0.3);
        last_sent_pan_  = 0.0;
        last_sent_tilt_ = 0.0;

        std_msgs::msg::Bool rst;
        rst.data = true;
        reset_pub_->publish(rst);

        pid_pan_.reset();
        pid_tilt_.reset();
        lp_pan_pos_.reset();
        lp_tilt_pos_.reset();
        lp_pan_vel_.reset();
        lp_tilt_vel_.reset();

        RCLCPP_INFO(this->get_logger(),
            "Target lost > %.1fs → reset to neutral", loss_timeout_);
    }

    // ═════════════════════════════════════════════════════════════════
    //  Envoi commande joints (compatible qbo_dynamixel /cmd_joints)
    // ═════════════════════════════════════════════════════════════════
    void sendJointCommand(double pan_pos, double tilt_pos,
                          double pan_vel, double tilt_vel)
    {
        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = this->now();
        cmd.name     = { pan_joint_name_,  tilt_joint_name_ };
        cmd.position = { pan_pos,          tilt_pos };
        cmd.velocity = { pan_vel,          tilt_vel };
        cmd_pub_->publish(cmd);
    }
};

// ═══════════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════════
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadTrackerNode>());
    rclcpp::shutdown();
    return 0;
}