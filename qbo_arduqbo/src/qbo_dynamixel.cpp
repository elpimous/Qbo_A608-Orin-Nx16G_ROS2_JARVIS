#include "rclcpp/rclcpp.hpp"
#include "qbo_arduqbo/controllers/dynamixel_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared("qbo_dynamixel", options);
    RCLCPP_INFO(node->get_logger(), "🎬 Starting qbo_dynamixel node");

    try
    {
        std::string usb_port = "";
        int baud_rate = -1;
        double protocol_version = -1.0;

        node->get_parameter("dynamixel.usb_port", usb_port);
        node->get_parameter("dynamixel.baud_rate", baud_rate);
        node->get_parameter("dynamixel.protocol_version", protocol_version);

        if (usb_port.empty()) {
            RCLCPP_FATAL(node->get_logger(), "❌ USB port is not defined");
            return 1;
        }
        if (baud_rate <= 0) {
            RCLCPP_FATAL(node->get_logger(), "❌ Invalid baud rate: %d", baud_rate);
            return 1;
        }
        if (protocol_version != 1.0 && protocol_version != 2.0) {
            RCLCPP_FATAL(node->get_logger(), "❌ Invalid protocol version: %.1f", protocol_version);
            return 1;
        }

        std::vector<std::string> motor_keys;
        node->get_parameter("dynamixel.motor_keys", motor_keys);

        if (motor_keys.empty()) {
            RCLCPP_FATAL(node->get_logger(), "❌ No motors defined");
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "✅ Config OK, starting controller...");
        auto controller = std::make_shared<DynamixelController>(node);

        RCLCPP_INFO(node->get_logger(), "🔍 Starting READ DEBUG LOOP...");

        // 🔴 LOOP DEBUG (2 Hz)
        rclcpp::WallRate rate(2);

        while (rclcpp::ok())
        {
            int32_t ticks = 0;

            if (controller->getWorkbench().itemRead(1, "Present_Position", &ticks))
            {
                RCLCPP_INFO(node->get_logger(), "✅ READ OK: %d", ticks);
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "❌ READ FAIL");
            }

            rclcpp::spin_some(node);
            rate.sleep();
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(node->get_logger(), "🛑 Fatal exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}