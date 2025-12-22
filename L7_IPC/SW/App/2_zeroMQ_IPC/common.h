
#pragma once


#define N_BUTTONS 4
#define BUTTON_CW 0         // Button index for clockwise rotation
#define BUTTON_CCW 1        // Button index for counterclockwise rotation
#define BUTTON_STOP 2       // Button index stopping

static uint8_t buttons[N_BUTTONS];

static void print_buttons(const char* msg) {
	char readable_buttons[N_BUTTONS+1]; // '0' or '1' per button + null terminator
	for(int i = 0; i < N_BUTTONS; i++){
		readable_buttons[i] = buttons[i] ? '1' : '0';
	}
	readable_buttons[N_BUTTONS] = '\0';

	printf("%s: %s\n", msg, readable_buttons);
}