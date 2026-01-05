
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "../../../../common.h"

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("routine_node");

	// Open publisher.
	rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher 
		= node->create_publisher<sensor_msgs::msg::Joy>("joy", 10);

	// Setup message.
	sensor_msgs::msg::Joy joy_msg;
	joy_msg.header.frame_id = "joy";
	joy_msg.buttons.resize(N_BUTTONS, 0);

	// Send routine:
	while(rclcpp::ok()){
		// STOP 1s
		RCLCPP_INFO_STREAM(node->get_logger(), "STOP");
		joy_msg.buttons[BUTTON_STOP] = 1;
		publisher->publish(joy_msg);
		rclcpp::sleep_for(std::chrono::seconds(1));
		joy_msg.buttons[BUTTON_STOP] = 0;

		// CW 1s
		RCLCPP_INFO_STREAM(node->get_logger(), "CW");
		joy_msg.buttons[BUTTON_CW] = 1;
		publisher->publish(joy_msg);
		rclcpp::sleep_for(std::chrono::seconds(1));
		joy_msg.buttons[BUTTON_CW] = 0;

		// TODO 1s STOP
		// TODO 2s CCW
	}

	RCLCPP_INFO_STREAM(node->get_logger(), "exit");
	return 0;
}
