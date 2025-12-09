
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <stdlib.h>

#include "include/gpio_ctrl.h"
#include "gpio.h"

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

int main() {

	// Open GPIO device
	int gpio_fd = open(DEV_STREAM_FN, O_RDWR);
	
	if (gpio_fd < 0) {
		perror("Failed to open /dev/gpio_stream");
		return EXIT_FAILURE;
	}




	uint8_t state = 0;
	while (1) {
		//TODO Spin stepper

		usleep(1000); 
	}

	printf("Exiting...\n");
	
	
	close(gpio_fd);

	return 0;
}
