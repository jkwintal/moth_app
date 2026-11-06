#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <moth/adc_reader.h>
#include <moth/light_acquisition.h>

LOG_MODULE_REGISTER(moth_light_acquisition, LOG_LEVEL_INF);

#define MOTH_LIGHT_PERIOD_MS 50
#define MOTH_LIGHT_QUEUE_LENGTH 4
#define MOTH_LIGHT_FILTER_SHIFT 2

K_MSGQ_DEFINE(moth_light_queue,
		      sizeof(struct moth_light_frame),
		      MOTH_LIGHT_QUEUE_LENGTH,
		      sizeof(uint32_t));

K_THREAD_STACK_DEFINE(light_stack, 1024);
static struct k_thread light_thread;

static uint16_t filter_sample(uint16_t previous, uint16_t current,
			      bool initialized)
{
	if (!initialized) {
		return current;
	}

	int32_t delta = (int32_t)current - (int32_t)previous;
	return (uint16_t)((int32_t)previous +
				  (delta >> MOTH_LIGHT_FILTER_SHIFT));
}

static void publish_frame(const struct moth_light_frame *frame)
{
	if (k_msgq_put(&moth_light_queue, frame, K_NO_WAIT) == 0) {
		return;
	}

	/* Keep the controller close to real time if it falls behind. */
	struct moth_light_frame discarded;
	(void)k_msgq_get(&moth_light_queue, &discarded, K_NO_WAIT);
	(void)k_msgq_put(&moth_light_queue, frame, K_NO_WAIT);
}

static void light_thread_entry(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	struct moth_adc_sample sample;
	struct moth_light_frame frame = { 0 };
	bool filter_initialized = false;
	uint32_t sequence = 0;

	int rc = moth_adc_reader_init();
	if (rc != 0) {
		LOG_ERR("ADC initialization failed: %d", rc);
	}

	while (true) {
		frame.sequence = sequence++;
		frame.valid = false;

		if (rc == 0) {
			rc = moth_adc_reader_read(&sample);
			if (rc == 0) {
				frame.left = filter_sample(frame.left, sample.left,
							   filter_initialized);
				frame.right = filter_sample(frame.right, sample.right,
							    filter_initialized);
				filter_initialized = true;
				frame.valid = true;
			} else {
				LOG_ERR("ADC read failed: %d", rc);
			}
		}

		publish_frame(&frame);
		k_sleep(K_MSEC(MOTH_LIGHT_PERIOD_MS));

		/* A transient read error should not permanently disable acquisition. */
		if (rc != 0) {
			rc = moth_adc_reader_init();
		}
	}
}

int moth_light_acquisition_start(void)
{
	k_thread_create(&light_thread, light_stack, K_THREAD_STACK_SIZEOF(light_stack),
			light_thread_entry, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
	return 0;
}
