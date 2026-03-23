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

inline double radians(double angle) {
    return angle * M_PI / 180.0;
}


// =============================================================================
// 🏗️  TF config — remplace les if/else dans publishJointStates()
// =============================================================================

// vince : axe de rotation du joint pour construire le quaternion TF
enum class TfAxis { YAW, PITCH, ROLL };

// vince : une entrée par joint ayant un TF à publier.
// Peuplée dans le constructeur : tf_configs_["head_pan_joint"] = {...}
// Lookup O(1) dans le hot path publishJointStates() → zéro branche if/else
struct TfConfig {
    std::string parent_frame;
    std::string child_frame;
    double tx, ty, tz;   // translation [m] dans le frame parent
    TfAxis axis;
};


// =============================================================================
// 🔧 DynamixelServo — Un servo AX-12A individuel
// =============================================================================
class DynamixelServo
{
public:
    DynamixelServo(const std::shared_ptr<rclcpp::Node> & node,
                   const std::string & name,
                   DynamixelWorkbench* wb);
    ~DynamixelServo();

    // vince : setAngle() NE WRITE PAS sur le bus.
    // Elle calcule pending_goal_ / pending_speed_, lève pending_dirty_.
    // flushSyncWrite() dans DynamixelController fait le vrai write groupé.
    void setAngle(float ang, float velocity);  // ← nom et signature du cpp

    float getAngle()     const { return angle_; }
    int   getGoalTicks() const { return goal_ticks_; }

    bool servoTorqueEnable(
        const std::shared_ptr<qbo_msgs::srv::TorqueEnable::Request> req,
        std::shared_ptr<qbo_msgs::srv::TorqueEnable::Response> res);

    void setParams(const std::string & motor_key);

    bool isTorqueEnabled()        const { return torque_enabled_; }
    void setTorqueEnabled(bool v)       { torque_enabled_ = v; }
    std::string getName()         const { return name_; }

    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter> & parameters);

    // ─── Identification ───────────────────────────────────────────────────────
    int         id_;
    std::string name_;
    std::string joint_name_;
    uint16_t    model_number_{0};

    // ─── Paramètres mécaniques ────────────────────────────────────────────────
    bool   invert_;
    int    neutral_;        // tick = 0 rad (512 pour AX-12A centre)
    int    ticks_;          // résolution totale (1024 pour AX-12A)
    float  max_angle_;      // [rad]
    float  min_angle_;      // [rad]
    float  rad_per_tick_;   // vince : recalculé UNIQUEMENT si range_ ou ticks_ change
    float  max_speed_;      // [rad/s]
    float  range_;          // [rad] étendue totale
    int    torque_limit_;   // 0–1023

    // ─── État courant ─────────────────────────────────────────────────────────
    float  angle_{0.0f};    // [rad] — dernière position lue via itemRead()

    // ─── Cache Torque_Limit ───────────────────────────────────────────────────
    // vince : évite writeRegister(Torque_Limit) à chaque setAngle() (50Hz inutile)
    // -1 = jamais écrit → force le premier write au démarrage
    int    last_written_torque_limit_{-1};

    // ─── Commande en attente — interface avec flushSyncWrite() ───────────────
    // vince : AX-12A adresses contiguës addr 30–33 :
    //   pending_goal_  → Goal_Position (addr 30, 2 bytes)
    //   pending_speed_ → Moving_Speed  (addr 32, 2 bytes)
    //   Un seul handler addSyncWriteHandler(30, 4) couvre les deux.
    int32_t pending_goal_{512};    // ticks cibles Goal_Position
    int32_t pending_speed_{300};   // Moving_Speed 0–1023
    bool    pending_dirty_{false}; // true = commande à envoyer ce cycle

private:
    std::shared_ptr<rclcpp::Node> node_;
    DynamixelWorkbench* dxl_wb_;
    bool torque_enabled_;
    int goal_ticks_;

    rclcpp::Service<qbo_msgs::srv::TorqueEnable>::SharedPtr servo_torque_enable_srv_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};


// =============================================================================
// 🤖 DynamixelController — Gestion globale du bus Dynamixel
// =============================================================================
class DynamixelController
{
public:
    explicit DynamixelController(const std::shared_ptr<rclcpp::Node> & node);
    ~DynamixelController();

    // vince : déclaration manquante dans le hpp précédent — utilisé dans le cpp
    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter> & parameters);

private:
    void publishJointStates();
    void jointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void checkInactivity();

    // vince : nom exact utilisé dans le cpp (était syncWritePositions() dans le hpp)
    // Collecte les servos dirty → 1 SYNC_WRITE groupé Protocol 1.0
    // Fallback sur itemWrite() individuels si handler non disponible ou count==1
    void flushSyncWrite();  // ← nom du cpp

    // ─── Node & bus ───────────────────────────────────────────────────────────
    std::shared_ptr<rclcpp::Node> node_;
    DynamixelWorkbench            dxl_wb_;
    std::string                   usb_port_;
    int                           baud_rate_;
    // vince : protocol_version_ conservé car lu depuis les params ROS2 au démarrage
    // et passé à dxl_wb_.setPacketHandler() — même si on est toujours en 1.0
    double                        protocol_version_;

    // ─── Servos ───────────────────────────────────────────────────────────────
    std::vector<std::unique_ptr<DynamixelServo>> servos_;        // ownership
    std::unordered_map<std::string, DynamixelServo*> servo_map_; // lookup O(1)

    // ─── Sync Write (Protocol 1.0) ────────────────────────────────────────────
    // vince : idx exact utilisé dans le cpp (était _index_ dans le hpp précédent)
    int  sync_write_handler_idx_{-1};    // ← nom du cpp
    bool sync_write_available_{false};

    // vince : buffers pré-alloués (resize dans le constructeur) — zéro alloc à 50Hz
    //   sync_ids_  : [id0, id1, ...]
    //   sync_data_ : [goal0, speed0, goal1, speed1, ...]   (2 int32 par servo)
    std::vector<uint8_t>  sync_ids_;
    std::vector<int32_t>  sync_data_;

    // ─── Message JointState pré-alloué ───────────────────────────────────────
    // vince : resize() dans le constructeur, noms pré-remplis, zéro push_back à 50Hz
    sensor_msgs::msg::JointState joint_state_msg_;

    // ─── TF ───────────────────────────────────────────────────────────────────
    std::unique_ptr<tf2_ros::TransformBroadcaster>  tf_broadcaster_;
    std::unordered_map<std::string, TfConfig>       tf_configs_;

    // ─── Publishers / Subscribers / Timers ───────────────────────────────────
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;  // ← nom du cpp (était state_timer_)
    rclcpp::TimerBase::SharedPtr inactivity_timer_;

    // ─── Torque auto-off ──────────────────────────────────────────────────────
    bool   auto_torque_off_;
    double timeout_sec_;  // ← nom du cpp (était auto_torque_off_timeout_)

    // vince : steady_clock_ déclaré comme membre car utilisé dans checkInactivity()
    // via steady_clock_.now() — manquait dans le hpp précédent
    rclcpp::Clock                          steady_clock_{RCL_STEADY_TIME};
    rclcpp::Time                           last_cmd_time_;
};

#endif // DYNAMIXEL_CONTROLLER_HPP