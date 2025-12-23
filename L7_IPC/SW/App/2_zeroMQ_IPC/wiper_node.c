#include <stdio.h>
#include <zmq.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>

#include "common.h"

// Hostname/IP of publisher (joy_node or routine_node).
#define ZMQ_ENDPOINT "tcp://rpi-controls.local:5555"

#define DEV_FN "/dev/gpio_stream"

int gpio_write(int fd, uint8_t pin, uint8_t value) {
	uint8_t pkg[3];
	pkg[0] = 'w';
	pkg[1] = pin;
	pkg[2] = value;

	if(write(fd, &pkg, 3) != 3){
		perror("Failed to write to GPIO");
		return -1;
	}
	return 0;
}

int main() {
	int gpio_fd = open(DEV_FN, O_RDWR);
	if(gpio_fd < 0){
		perror("Failed to open GPIO driver");
		return 1;
	}

	void* context = zmq_ctx_new();
	if(!context){
		perror("Failed to create ZeroMQ context");
		return 1;
	}
	void* subscriber = zmq_socket(context, ZMQ_SUB);
	if(!subscriber){
		perror("Failed to create ZeroMQ socket");
		zmq_ctx_destroy(context);
		return 1;
	}
	if(zmq_connect(subscriber, ZMQ_ENDPOINT) != 0){
		perror("Failed to connect ZeroMQ socket");
		zmq_close(subscriber);
		zmq_ctx_destroy(context);
		return 1;
	}
	if(zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0) != 0){
		perror("Failed to set ZMQ_SUBSCRIBE");
		zmq_close(subscriber);
		zmq_ctx_destroy(context);
		return 1;
	}

	printf("Connected and listening on %s...\n", ZMQ_ENDPOINT);


	while(1){
		zmq_msg_t msg;
		zmq_msg_init(&msg);
		int bytes = zmq_msg_recv(&msg, subscriber, ZMQ_DONTWAIT); // Non-blocking receive.
		if(bytes == N_BUTTONS){
			memcpy(buttons, zmq_msg_data(&msg), bytes);

			print_buttons("wiper_node recv buttons");

			if(buttons[BUTTON_CCW]){
				printf("CCW\n");
				gpio_write(gpio_fd, 3, 1); // CCW
				gpio_write(gpio_fd, 4, 0); // CCW
				gpio_write(gpio_fd, 2, 1); // EN = 1
			}
			if(buttons[BUTTON_CW]){
				printf("CW\n");
				gpio_write(gpio_fd, 3, 0); // CW
				gpio_write(gpio_fd, 4, 1); // CW
				gpio_write(gpio_fd, 2, 1); // EN = 1
			}
			if(buttons[BUTTON_STOP]){
				printf("STOP\n");
				gpio_write(gpio_fd, 2, 0); // EN = 0
			}
		}
		zmq_msg_close(&msg);

		usleep(10000); // 10 ms
	}

	zmq_close(subscriber);
	zmq_ctx_destroy(context);
	
	close(gpio_fd);

	return 0;
}

