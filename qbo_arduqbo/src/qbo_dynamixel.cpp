#include "rclcpp/rclcpp.hpp"
#include "qbo_arduqbo/controllers/dynamixel_hardware.hpp"
#include "qbo_msgs/srv/torque_enable.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <thread>
#include <string>
#include <algorithm>
#include <cmath>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("dynamixel_hardware");
    RCLCPP_INFO(node->get_logger(), "Starting qbo_dynamixel debug node");

    // ✅ Lecture depuis le yaml
    node->declare_parameter("dynamixel.usb_port",         "/dev/ttyDmx");
    node->declare_parameter("dynamixel.baud_rate",        1000000);
    node->declare_parameter("dynamixel.protocol_version", 1.0);
    node->declare_parameter("dynamixel.motor_keys",       std::vector<std::string>{});
    node->declare_parameter("auto_torque_off",         false);
    node->declare_parameter("auto_torque_off_timeout",  20.0);

    std::string port     = node->get_parameter("dynamixel.usb_port").as_string();
    int         baud     = node->get_parameter("dynamixel.baud_rate").as_int();
    double      protocol = node->get_parameter("dynamixel.protocol_version").as_double();
    auto        keys     = node->get_parameter("dynamixel.motor_keys").as_string_array();

    bool   auto_torque_off     = node->get_parameter("auto_torque_off").as_bool();
    double torque_off_timeout  = node->get_parameter("auto_torque_off_timeout").as_double();
    auto   last_cmd_time       = node->get_clock()->now();  // ✅ timestamp dernière commande

    RCLCPP_INFO(node->get_logger(), "Auto torque off: %s (timeout=%.0fs)",
        auto_torque_off ? "ON" : "OFF", torque_off_timeout);

    auto hardware = std::make_shared<DynamixelHardware>();

    hardware_interface::HardwareInfo info;
    info.hardware_parameters["port"]             = port;
    info.hardware_parameters["baud_rate"]        = std::to_string(baud);
    info.hardware_parameters["protocol_version"] = std::to_string(protocol);

    // ✅ Charge chaque motor depuis le yaml
    for (const auto & key : keys) {
        std::string base = "dynamixel.motors." + key;

        node->declare_parameter(base + ".name",              "");
        node->declare_parameter(base + ".id",                0);
        node->declare_parameter(base + ".neutral",           512);
        node->declare_parameter(base + ".ticks",             1024);
        node->declare_parameter(base + ".torque_limit",      200);
        node->declare_parameter(base + ".invert",            false);
        node->declare_parameter(base + ".max_speed",         1.0);
        node->declare_parameter(base + ".min_angle_degrees", -70.0);
        node->declare_parameter(base + ".max_angle_degrees",  70.0);

        hardware_interface::ComponentInfo j;
        j.name                       = node->get_parameter(base + ".name").as_string();
        j.parameters["id"]           = std::to_string(node->get_parameter(base + ".id").as_int());
        j.parameters["neutral"]      = std::to_string(node->get_parameter(base + ".neutral").as_int());
        j.parameters["ticks"]        = std::to_string(node->get_parameter(base + ".ticks").as_int());
        j.parameters["torque_limit"] = std::to_string(node->get_parameter(base + ".torque_limit").as_int());
        j.parameters["invert"]       = node->get_parameter(base + ".invert").as_bool() ? "true" : "false";
        j.parameters["max_speed"]    = std::to_string(node->get_parameter(base + ".max_speed").as_double());
        j.parameters["min_angle"]    = std::to_string(
            node->get_parameter(base + ".min_angle_degrees").as_double() * M_PI / 180.0);
        j.parameters["max_angle"]    = std::to_string(
            node->get_parameter(base + ".max_angle_degrees").as_double() * M_PI / 180.0);

        info.joints.push_back(j);
        RCLCPP_INFO(node->get_logger(),
            "Loaded joint %s (ID %s) min=%.2f max=%.2f rad max_speed=%.1f rad/s from yaml",
            j.name.c_str(), j.parameters["id"].c_str(),
            std::stod(j.parameters["min_angle"]),
            std::stod(j.parameters["max_angle"]),
            std::stod(j.parameters["max_speed"]));
    }

    if (hardware->on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_FATAL(node->get_logger(), "Failed to initialize hardware");
        return 1;
    }

    // ✅ Subscriber /cmd_joints avec clamp position + vitesse
    auto cmd_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/cmd_joints", 10,
        [&hardware, &node, &last_cmd_time](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            auto & wb            = hardware->getWorkbench();
            const auto & servos  = hardware->getServos();

            int32_t goals[10];
            uint8_t ids[10];
            uint8_t count = 0;

            // ✅ Mise à jour du timestamp à chaque commande reçue
            last_cmd_time = node->get_clock()->now();

            // Réactive le torque si il avait été éteint par timeout
            for (auto & s : hardware->getServos()) {
                if (!s.torque_enabled) {
                    hardware->getWorkbench().torqueOn(s.id);
                    s.torque_enabled = true;
                    RCLCPP_INFO(node->get_logger(), "Auto torque ON (commande reçue) : %s", s.name.c_str());
                }
            }

            for (size_t i = 0; i < msg->name.size(); i++) {
                for (const auto & s : servos) {
                    if (s.name == msg->name[i] && i < msg->position.size()) {
                        ids[count]   = static_cast<uint8_t>(s.id);
                        goals[count] = hardware->angleToTicksPublic(s, msg->position[i]);

                        // ✅ Vitesse clampée à max_speed du yaml
                        if (i < msg->velocity.size() && msg->velocity[i] > 0.0) {
                            double vel       = std::min(msg->velocity[i], s.max_speed);
                            int speed_ticks  = static_cast<int>(vel * 85.9);
                            speed_ticks      = std::clamp(speed_ticks, 1, 1023);

                            RCLCPP_INFO(node->get_logger(),
                                "Moving_Speed servo %d -> %.2f rad/s (req=%.1f max=%.1f) -> %d ticks",
                                ids[count], vel, msg->velocity[i], s.max_speed, speed_ticks);

                            const char * log = nullptr;
                            if (!wb.itemWrite(ids[count], "Moving_Speed", speed_ticks, &log)) {
                                RCLCPP_WARN(node->get_logger(),
                                    "Moving_Speed failed servo %d: %s",
                                    ids[count], log ? log : "unknown");
                            }
                        }

                        RCLCPP_INFO(node->get_logger(),
                            "cmd %s -> %.3f rad -> tick %d",
                            s.name.c_str(), msg->position[i], goals[count]);
                        count++;
                        break;
                    }
                }
            }

            if (count == 0) return;

            // ✅ Envoi Goal_Position
            for (uint8_t i = 0; i < count; i++) {
                const char * log = nullptr;
                if (!wb.itemWrite(ids[i], "Goal_Position", goals[i], &log)) {
                    RCLCPP_WARN(node->get_logger(),
                        "itemWrite failed servo %d: %s", ids[i], log ? log : "unknown");
                }
            }
        }
    );

    // ✅ Services torque_enable générés depuis les servos chargés
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    for (const auto & s : hardware->getServos()) {
        auto svc = node->create_service<qbo_msgs::srv::TorqueEnable>(
            "/" + s.name + "/torque_enable",
            [&hardware, id = s.id, name = s.name](
                const std::shared_ptr<qbo_msgs::srv::TorqueEnable::Request> req,
                std::shared_ptr<qbo_msgs::srv::TorqueEnable::Response> res)
            {
                auto & wb = hardware->getWorkbench();
                if (req->torque_enable) {
                    wb.torqueOn(id);
                } else {
                    wb.torqueOff(id);
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
                for (auto & s : hardware->getServos()) {
                    if (s.id == id) { s.torque_enabled = req->torque_enable; break; }
                }
                std::string msg_str = name + " torque -> " + (req->torque_enable ? "ON" : "OFF");
                res->success = true;
                res->message = msg_str;
                RCLCPP_INFO(rclcpp::get_logger("qbo_dynamixel"), "%s", msg_str.c_str());
            }
        );
        services.push_back(svc);
    }

    RCLCPP_INFO(node->get_logger(), "Hardware initialized. Starting read loop...");

    // ✅ Fréquence de lecture depuis le yaml
    node->declare_parameter("dynamixel_state_rate_hz", 1.0);
    double state_rate = node->get_parameter("dynamixel_state_rate_hz").as_double();
    RCLCPP_INFO(node->get_logger(), "Read loop @ %.0f Hz", state_rate);
    rclcpp::WallRate rate(state_rate);

    while (rclcpp::ok()) {
        if (hardware->read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0)) ==
            hardware_interface::return_type::OK)
        {
            // ✅ Auto torque off après timeout d'inactivité
            if (auto_torque_off) {
                double elapsed = (node->get_clock()->now() - last_cmd_time).seconds();
                if (elapsed > torque_off_timeout) {
                    for (auto & s : hardware->getServos()) {
                        if (s.torque_enabled) {
                            hardware->getWorkbench().torqueOff(s.id);
                            s.torque_enabled = false;
                            RCLCPP_WARN(node->get_logger(),
                                "Auto torque OFF : %s (inactif depuis %.0fs)",
                                s.name.c_str(), elapsed);
                        }
                    }
                }
            }

            for (const auto & s : hardware->getServos()) {
                RCLCPP_INFO(node->get_logger(),
                    "Motor %s (ID %d): pos=%.3f rad  temp=%.0f°C  torque=%s",
                    s.name.c_str(), s.id, s.position, s.temperature,
                    s.torque_enabled ? "ON" : "OFF");
            }
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }

    hardware->on_deactivate(rclcpp_lifecycle::State());
    rclcpp::shutdown();
    return 0;
}