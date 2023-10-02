#include <rclcpp/rclcpp.hpp>
#include "drobo_interfaces/msg/solenoid_state_msg.hpp"
#include "drobo_interfaces/msg/sd_lib_msg.hpp"

class MecanumAirRiser : public rclcpp::Node{
    private:
        rclcpp::Publisher<drobo_interfaces::msg::SdLibMsg>::SharedPtr _publisher;
        rclcpp::Subscription<drobo_interfaces::msg::SolenoidStateMsg>::SharedPtr _subscription;
        void _topic_callback(const drobo_interfaces::msg::SolenoidStateMsg::SharedPtr _msg);
    public:
        MecanumAirRiser(
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
        MecanumAirRiser(
            const std::string& name_space,
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
};