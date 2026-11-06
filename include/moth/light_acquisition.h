#ifndef MOTH_LIGHT_ACQUISITION_H_
#define MOTH_LIGHT_ACQUISITION_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

struct moth_light_frame {
	uint16_t left;
	uint16_t right;
	uint32_t sequence;
	bool valid;
};

extern struct k_msgq moth_light_queue;

int moth_light_acquisition_start(void);

#endif
