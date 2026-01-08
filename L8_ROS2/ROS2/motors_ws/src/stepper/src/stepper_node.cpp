
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <chrono>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "../../../../common.h"

rclcpp::Node::SharedPtr node;

int gpio_write(int fd, uint8_t pin, uint8_t value) {
	uint8_t pkg[3];
	pkg[0] = 'w';
	pkg[1] = pin;
	pkg[2] = value;
	

	if (write(fd, &pkg, 3) != 3) {
		perror("Failed to write to GPIO");
		return -1;
	}
	return 0;
}

int rot_dir = -1;
bool rot_en = false;
void joy__cb(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
	if(joy_msg->buttons[BUTTON_CW]){
		RCLCPP_INFO_STREAM(node->get_logger(), "CW");
		rot_en = true;
		rot_dir = -1;
	}
	if(joy_msg->buttons[BUTTON_CCW]){
		RCLCPP_INFO_STREAM(node->get_logger(), "CCW");
		rot_en = true;
		rot_dir = +1;
	}
	if(joy_msg->buttons[BUTTON_STOP]){
		RCLCPP_INFO_STREAM(node->get_logger(), "STOP");
		rot_en = false;
	}
}

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	node = std::make_shared<rclcpp::Node>("stepper_node");

	// Subscribe.
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription 
		= node->create_subscription<sensor_msgs::msg::Joy>(
		"joy",
		1,
		&joy__cb
	);

	// Open GPIO device
	int gpio_fd = open("/dev/gpio_stream", O_RDWR);
	if (gpio_fd < 0) {
		perror("Failed to open /dev/gpio_stream");
		return EXIT_FAILURE;
	}

	int step = 0;
	// Rotate motor.
	while(rclcpp::ok()){
		if(rot_en){
			gpio_write(gpio_fd, 17, step == 0);
			gpio_write(gpio_fd, 18, step == 1);
			gpio_write(gpio_fd, 22, step == 2);
			gpio_write(gpio_fd, 23, step == 3);
			rclcpp::sleep_for(std::chrono::milliseconds(3));

			step = (step + rot_dir) % 4;
		}

		rclcpp::spin_some(node);
	}

	return 0;
}
