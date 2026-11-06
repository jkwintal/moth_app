#ifndef MOTH_PHOTOTAXIS_H_
#define MOTH_PHOTOTAXIS_H_

#include <stdbool.h>
#include <stdint.h>

#include <moth/light_acquisition.h>

enum moth_motion {
	MOTH_MOTION_STOP = 0,
	MOTH_MOTION_FORWARD,
	MOTH_MOTION_TURN_LEFT,
	MOTH_MOTION_TURN_RIGHT,
};

struct moth_drive_request {
	enum moth_motion motion;
	int16_t left_speed;
	int16_t right_speed;
	bool valid;
};

struct moth_phototaxis_config {
	uint16_t minimum_light;
	uint16_t deadband;
	int16_t base_speed;
	int16_t turn_gain;
	int16_t maximum_speed;
};

void moth_phototaxis_compute(const struct moth_light_frame *frame,
				     const struct moth_phototaxis_config *config,
				     struct moth_drive_request *request);

int moth_phototaxis_start(void);

#endif
