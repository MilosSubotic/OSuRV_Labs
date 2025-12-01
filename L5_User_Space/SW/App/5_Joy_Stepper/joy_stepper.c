
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <stdlib.h>

#include "include/gpio_ctrl.h"
#include "gpio.h"

struct js_event js_event_data;

volatile int num_of_buttons = 0;
volatile int num_of_axes = 0;
volatile uint8_t* volatile buttons;
volatile uint8_t* volatile axes;
pthread_mutex_t joy_printf_mtx = PTHREAD_MUTEX_INITIALIZER;
sem_t joy_intitialized;

void* js_reader(void* arg) {
	int js_fd;

	// Open the joystick device file in read-only mode
	js_fd = open("/dev/input/js0", O_RDONLY);
	if (js_fd == -1) {
		perror("Error opening joystick device");
		return NULL;
	}

	ioctl(js_fd, JSIOCGBUTTONS, &num_of_buttons);
	ioctl(js_fd, JSIOCGAXES, &num_of_axes);

	// Allocated buttons array
	buttons = (volatile uint8_t*)malloc(num_of_buttons * sizeof(uint8_t));
	if (buttons == NULL) {
		perror("Memory allocation failed");
		return NULL;
	}
	// Initialize buttons to 0
	for (int i = 0; i < num_of_buttons; i++) {
		buttons[i] = 0;
	}
	printf("Joystick initialized with %d buttons\n", num_of_buttons);

	// Allocated axes array
	axes = (volatile uint8_t*)malloc(num_of_axes * sizeof(uint8_t));
	if (axes == NULL) {
		perror("Memory allocation failed");
		return NULL;
	}
	// Initialize axes to 0
	for (int i = 0; i < num_of_axes; i++) {
		axes[i] = 0;
	}
	printf("Joystick initialized with %d axes\n", num_of_axes);

	sem_post(&joy_intitialized);

	while (1) {
		if (read(js_fd, &js_event_data, sizeof(struct js_event)) != sizeof(struct js_event)) {
			perror("Error reading joystick event");
			break;
		}

		if (js_event_data.type & JS_EVENT_BUTTON) {
			pthread_mutex_lock(&joy_printf_mtx);
			printf("Button %d %s (value: %d)\n",
				js_event_data.number,
				(js_event_data.value == 0) ? "released" : "pressed",
				js_event_data.value
			);

			buttons[js_event_data.number] = js_event_data.value;

			pthread_mutex_unlock(&joy_printf_mtx);
		} else if (js_event_data.type & JS_EVENT_AXIS) {
		 	printf("Axis %d moved (value: %d)\n",
		 		js_event_data.number,
		 		js_event_data.value
			);
		}
	}

	close(js_fd);
	return NULL;
}


int gpio_write(int fd, uint8_t pin, uint8_t value) {
	uint8_t pkg[3];
	pkg[0] = GPIO_CTRL__WRITE;
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

	sem_init(&joy_intitialized, 0, 0);

	pthread_t reader;
	pthread_create(&reader, NULL, js_reader, NULL);


	sem_wait(&joy_intitialized);




	while (1) {
		pthread_mutex_lock(&joy_printf_mtx);
		if (buttons[0]) {
			printf("forward\n");
			//TODO
		} else if (buttons[1]) {
			printf("backward\n");
			//TODO
		}

		pthread_mutex_unlock(&joy_printf_mtx);
		usleep(10000); 
	}

	printf("Exiting...\n");
	
	
	pthread_join(reader, NULL);

	if (buttons != NULL) {
		free((void*)buttons);
		buttons = NULL;
	}

	close(gpio_fd);
	pthread_mutex_destroy(&joy_printf_mtx);

	return 0;
}
