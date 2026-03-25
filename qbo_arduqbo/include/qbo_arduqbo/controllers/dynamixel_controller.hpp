#ifndef DYNAMIXEL_CONTROLLER_HPP
#define DYNAMIXEL_CONTROLLER_HPP

// =============================================================================
// dynamixel_controller.hpp — Néo / QBo — AX-12A Protocol 1.0 UNIQUEMENT
// =============================================================================
// CORRECTIONS vs version précédente du hpp (alignement sur le cpp réel) :
//
//  ❌ → ✅  setTarget()              → setAngle(float ang, float velocity)
//  ❌ → ✅  syncWritePositions()     → flushSyncWrite()
//  ❌ → ✅  sync_write_handler_index_→ sync_write_handler_idx_
//  ❌ → ✅  auto_torque_off_timeout_ → timeout_sec_
//  ❌ → ✅  state_timer_             → joint_state_timer_
//  ❌ → ✅  manquait steady_clock_   → rclcpp::Clock steady_clock_
//  ❌ → ✅  manquait onParameterChange() dans DynamixelController
//
// RAPPEL ARCHITECTURE Protocol 1.0 (AX-12A) :
//   SYNC WRITE (0x83) ✅ — 1 trame pour tous les servos
//   SYNC READ         ❌ — Protocol 2.0 uniquement
//   BULK READ (0x92)  ⚠️ — théoriquement dispo mais mal supporté par DWB
//   → lectures individuelles itemRead() par servo (≈1ms pour 2 servos à 500kbaud)
//
//   Adresses AX-12A contiguës utilisées pour le Sync Write :
//     Addr 30 (0x1E) : Goal_Position [2 bytes]
//     Addr 32 (0x20) : Moving_Speed  [2 bytes]
//     → addSyncWriteHandler(30, 4) couvre les deux registres en 1 handler
// =============================================================================

#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <qbo_msgs/srv/torque_enable.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.hpp>

inline double radians(double angle) { return angle * M_PI / 180.0; }

// ─── TF configuration ─────────────────────────────────────────────────────────
enum class TfAxis { YAW, PITCH, ROLL };
struct TfConfig {
    std::string parent_frame;
    std::string child_frame;
    double tx, ty, tz;
    TfAxis axis;
};

// ─── DynamixelServo ───────────────────────────────────────────────────────────
class DynamixelServo
{
public:
    DynamixelServo(const std::shared_ptr<rclcpp::Node> & node,
                   const std::string & name,
                   DynamixelWorkbench* wb);
    ~DynamixelServo();

    void setAngle(float ang, float velocity);
    float getAngle() const { return angle_; }
    int getGoalTicks() const { return goal_ticks_; }

    bool servoTorqueEnable(
        const std::shared_ptr<qbo_msgs::srv::TorqueEnable::Request> req,
        std::shared_ptr<qbo_msgs::srv::TorqueEnable::Response> res);

    void setParams(const std::string & motor_key);

    bool isTorqueEnabled() const { return torque_enabled_; }
    void setTorqueEnabled(bool v) { torque_enabled_ = v; }
    std::string getName() const { return name_; }

    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter> & parameters);

    int id_;
    std::string name_;
    std::string joint_name_;
    uint16_t model_number_{0};

    bool invert_;
    int neutral_;
    int ticks_;
    float max_angle_;
    float min_angle_;
    float rad_per_tick_;
    float max_speed_;
    float range_;
    int torque_limit_;

    float angle_{0.0f};
    int last_written_torque_limit_{-1};

    int32_t pending_goal_{512};
    int32_t pending_speed_{300};
    bool pending_dirty_{false};

private:
    std::shared_ptr<rclcpp::Node> node_;
    DynamixelWorkbench* dxl_wb_;
    bool torque_enabled_;
    int goal_ticks_;

    rclcpp::Service<qbo_msgs::srv::TorqueEnable>::SharedPtr servo_torque_enable_srv_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

// ─── DynamixelController ───────────────────────────────────────────────────────
class DynamixelController
{
public:
    explicit DynamixelController(const std::shared_ptr<rclcpp::Node> & node);
    ~DynamixelController();

    DynamixelWorkbench& getWorkbench() { return dxl_wb_; }

private:
    void publishJointStates();
    void jointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void checkInactivity();
    void flushSyncWrite();
    void publishDiagnostics();                    // ← pour les diagnostics

    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter> & parameters);
        
    std::shared_ptr<rclcpp::Node> node_;
    DynamixelWorkbench dxl_wb_;
    std::string usb_port_;
    int baud_rate_;
    double protocol_version_;

    std::vector<std::unique_ptr<DynamixelServo>> servos_;
    std::unordered_map<std::string, DynamixelServo*> servo_map_;

    int sync_write_handler_idx_{-1};
    bool sync_write_available_{false};
    std::vector<uint8_t> sync_ids_;
    std::vector<int32_t> sync_data_;

    sensor_msgs::msg::JointState joint_state_msg_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unordered_map<std::string, TfConfig> tf_configs_;   // ← indispensable

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    rclcpp::TimerBase::SharedPtr inactivity_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;         // ← indispensable

    bool auto_torque_off_;
    double timeout_sec_;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    rclcpp::Time last_cmd_time_;

    // ─── Diagnostics ───────────────────────────────────────────────────────────
    std::shared_ptr<diagnostic_updater::Updater> diagnostics_;
};

#endif // DYNAMIXEL_CONTROLLER_HPP
