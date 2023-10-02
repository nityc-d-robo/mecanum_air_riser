#include "mecanum_air_riser/main.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include "drobo_interfaces/msg/solenoid_state_msg.hpp"
#include "drobo_interfaces/msg/sd_lib_msg.hpp"

void MecanumAirRiser::_topic_callback(const drobo_interfaces::msg::SolenoidStateMsg::SharedPtr _msg){
    RCLCPP_INFO(this->get_logger(), "%uを%d", _msg->axle_position, _msg->state);
    uint16_t solenoind_power = _msg->state ? 999 : 0;

    auto msg = std::make_shared<drobo_interfaces::msg::SdLibMsg>();
    msg->address = _msg->axle_position / 2;
    msg->semi_id = NULL;
    msg->port = _msg->axle_position % 2;
    msg->power1 = solenoind_power;
    msg->power2 = solenoind_power;
    _publisher->publish(*msg);
}

MecanumAirRiser::MecanumAirRiser(
    const rclcpp::NodeOptions& options
): MecanumAirRiser("", options){}

MecanumAirRiser::MecanumAirRiser(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
): Node("mecanum_air_riser", name_space, options){
    _publisher = this->create_publisher<drobo_interfaces::msg::SdLibMsg>("sd_driver_topic", rclcpp::QoS(10));
    _subscription = this->create_subscription<drobo_interfaces::msg::SolenoidStateMsg>(
        "solenoid_order",
        rclcpp::QoS(10),
        std::bind(&MecanumAirRiser::_topic_callback, this, std::placeholders::_1)
    );
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumAirRiser>());
    rclcpp::shutdown();
    return 0;
}