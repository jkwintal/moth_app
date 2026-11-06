#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <moth/light_acquisition.h>
#include <moth/phototaxis.h>

LOG_MODULE_REGISTER(moth_phototaxis, LOG_LEVEL_INF);

#define MOTH_ADC_FULL_SCALE 4095

K_THREAD_STACK_DEFINE(phototaxis_stack, 1024);
static struct k_thread phototaxis_thread;

static int16_t clamp_speed(int32_t speed, int16_t maximum)
{
	if (speed < 0) {
		return 0;
	}
	if (speed > maximum) {
		return maximum;
	}
	return (int16_t)speed;
}

void moth_phototaxis_compute(const struct moth_light_frame *frame,
				     const struct moth_phototaxis_config *config,
				     struct moth_drive_request *request)
{
	if ((frame == NULL) || (config == NULL) || (request == NULL)) {
		return;
	}

	request->valid = frame->valid;
	request->motion = MOTH_MOTION_STOP;
	request->left_speed = 0;
	request->right_speed = 0;

	if (!frame->valid ||
	    ((frame->left < config->minimum_light) &&
	     (frame->right < config->minimum_light))) {
		return;
	}

	int32_t difference = (int32_t)frame->left - (int32_t)frame->right;
	int32_t magnitude = difference < 0 ? -difference : difference;

	if (magnitude <= config->deadband) {
		request->motion = MOTH_MOTION_FORWARD;
		request->left_speed = clamp_speed(config->base_speed,
							 config->maximum_speed);
		request->right_speed = clamp_speed(config->base_speed,
							  config->maximum_speed);
		return;
	}

	int32_t correction = (magnitude * config->turn_gain) / MOTH_ADC_FULL_SCALE;
	if (correction < 1) {
		correction = 1;
	}

	int32_t left_speed = config->base_speed;
	int32_t right_speed = config->base_speed;

	if (difference > 0) {
		/* Left is brighter: slow the left wheel to turn toward the light. */
		request->motion = MOTH_MOTION_TURN_LEFT;
		left_speed -= correction;
		right_speed += correction;
	} else {
		request->motion = MOTH_MOTION_TURN_RIGHT;
		left_speed += correction;
		right_speed -= correction;
	}

	request->left_speed = clamp_speed(left_speed, config->maximum_speed);
	request->right_speed = clamp_speed(right_speed, config->maximum_speed);
}

static const char *motion_name(enum moth_motion motion)
{
	switch (motion) {
	case MOTH_MOTION_FORWARD:
		return "forward";
	case MOTH_MOTION_TURN_LEFT:
		return "left";
	case MOTH_MOTION_TURN_RIGHT:
		return "right";
	case MOTH_MOTION_STOP:
	default:
		return "stop";
	}
}

static void phototaxis_thread_entry(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	const struct moth_phototaxis_config config = {
		.minimum_light = CONFIG_MOTH_PHOTOTAXIS_MINIMUM_LIGHT,
		.deadband = CONFIG_MOTH_PHOTOTAXIS_DEADBAND,
		.base_speed = CONFIG_MOTH_PHOTOTAXIS_BASE_SPEED,
		.turn_gain = CONFIG_MOTH_PHOTOTAXIS_TURN_GAIN,
		.maximum_speed = CONFIG_MOTH_PHOTOTAXIS_MAXIMUM_SPEED,
	};

	struct moth_light_frame frame;
	struct moth_drive_request request;
	enum moth_motion previous_motion = MOTH_MOTION_STOP;

	while (true) {
		if (k_msgq_get(&moth_light_queue, &frame, K_FOREVER) != 0) {
			continue;
		}

		moth_phototaxis_compute(&frame, &config, &request);

		if (request.motion != previous_motion) {
			LOG_INF("Phototaxis: %s (L=%d R=%d, valid=%d)",
				motion_name(request.motion), request.left_speed,
				request.right_speed, request.valid);
			previous_motion = request.motion;
		} else {
			LOG_DBG("Phototaxis: %s (L=%d R=%d)",
				motion_name(request.motion), request.left_speed,
				request.right_speed);
		}
	}
}

int moth_phototaxis_start(void)
{
	k_thread_create(&phototaxis_thread, phototaxis_stack,
			K_THREAD_STACK_SIZEOF(phototaxis_stack),
			phototaxis_thread_entry, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
	return 0;
}
