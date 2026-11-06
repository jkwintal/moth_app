#ifndef MOTH_LIGHT_ACQUISITION_H_
#define MOTH_LIGHT_ACQUISITION_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

struct moth_light_frame {
	/* Background-subtracted, noise-gated light signals. */
	uint16_t left;
	uint16_t right;
	/* Per-sensor startup background levels, useful for diagnostics. */
	uint16_t ambient_left;
	uint16_t ambient_right;
	uint16_t noise_margin_left;
	uint16_t noise_margin_right;
	uint32_t sequence;
	bool valid;
	bool calibrated;
};

extern struct k_msgq moth_light_queue;

int moth_light_acquisition_start(void);

#endif
