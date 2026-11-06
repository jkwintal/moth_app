#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <moth/adc_reader.h>
#include <moth/light_acquisition.h>

LOG_MODULE_REGISTER(moth_light_acquisition, LOG_LEVEL_INF);

#define MOTH_LIGHT_PERIOD_MS 50
#define MOTH_LIGHT_QUEUE_LENGTH 4
#define MOTH_LIGHT_FILTER_SHIFT 2
#define MOTH_CALIBRATION_MAX_SAMPLES 64

BUILD_ASSERT(CONFIG_MOTH_LIGHT_CALIBRATION_SAMPLES <=
		     MOTH_CALIBRATION_MAX_SAMPLES,
		     "Calibration sample count exceeds local storage");

K_MSGQ_DEFINE(moth_light_queue,
		      sizeof(struct moth_light_frame),
		      MOTH_LIGHT_QUEUE_LENGTH,
		      sizeof(uint32_t));

K_THREAD_STACK_DEFINE(light_stack, 1024);
static struct k_thread light_thread;

struct moth_light_calibration {
	uint16_t ambient_left;
	uint16_t ambient_right;
	uint16_t margin_left;
	uint16_t margin_right;
	bool ready;
};

static uint16_t calibration_left[MOTH_CALIBRATION_MAX_SAMPLES];
static uint16_t calibration_right[MOTH_CALIBRATION_MAX_SAMPLES];

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

static void sort_samples(uint16_t *samples, size_t count)
{
	for (size_t i = 1; i < count; ++i) {
		uint16_t value = samples[i];
		size_t j = i;

		while ((j > 0) && (samples[j - 1] > value)) {
			samples[j] = samples[j - 1];
			--j;
		}
		samples[j] = value;
	}
}

static uint16_t absolute_difference(uint16_t first, uint16_t second)
{
	return first >= second ? first - second : second - first;
}

static uint16_t calculate_noise_margin(uint16_t *samples, size_t count,
					       uint16_t ambient)
{
	/* Convert the sample array into absolute deviations from the median. */
	for (size_t i = 0; i < count; ++i) {
		samples[i] = absolute_difference(samples[i], ambient);
	}
	sort_samples(samples, count);

	uint32_t margin = (uint32_t)samples[count / 2] *
			  CONFIG_MOTH_LIGHT_NOISE_MULTIPLIER;
	if (margin < CONFIG_MOTH_LIGHT_MIN_NOISE_MARGIN) {
		margin = CONFIG_MOTH_LIGHT_MIN_NOISE_MARGIN;
	}
	if (margin > UINT16_MAX) {
		margin = UINT16_MAX;
	}

	return (uint16_t)margin;
}

static int calibrate_light(struct moth_light_calibration *calibration)
{
	const size_t sample_count = CONFIG_MOTH_LIGHT_CALIBRATION_SAMPLES;
	struct moth_adc_sample sample;

	LOG_INF("Light calibration: keep robot stationary for %d ms",
		CONFIG_MOTH_LIGHT_CALIBRATION_SETTLE_MS);
	k_sleep(K_MSEC(CONFIG_MOTH_LIGHT_CALIBRATION_SETTLE_MS));

	for (size_t i = 0; i < sample_count; ++i) {
		int rc = moth_adc_reader_read(&sample);
		if (rc != 0) {
			LOG_ERR("Light calibration ADC read failed: %d", rc);
			return rc;
		}

		calibration_left[i] = sample.left;
		calibration_right[i] = sample.right;

		if (i + 1 < sample_count) {
			k_sleep(K_MSEC(MOTH_LIGHT_PERIOD_MS));
		}
	}

	sort_samples(calibration_left, sample_count);
	sort_samples(calibration_right, sample_count);

	calibration->ambient_left = calibration_left[sample_count / 2];
	calibration->ambient_right = calibration_right[sample_count / 2];
	calibration->margin_left = calculate_noise_margin(
		calibration_left, sample_count, calibration->ambient_left);
	calibration->margin_right = calculate_noise_margin(
		calibration_right, sample_count, calibration->ambient_right);
	calibration->ready = true;

	LOG_INF("Light calibration ready: ambient L=%u R=%u, margin L=%u R=%u",
		calibration->ambient_left, calibration->ambient_right,
		calibration->margin_left, calibration->margin_right);

	return 0;
}

static uint16_t subtract_background(uint16_t sample, uint16_t ambient)
{
	return sample > ambient ? sample - ambient : 0;
}

static uint16_t subtract_noise_margin(uint16_t signal, uint16_t margin)
{
	return signal > margin ? signal - margin : 0;
}

static uint16_t track_background(uint16_t ambient, uint16_t sample)
{
	int32_t delta = (int32_t)sample - (int32_t)ambient;
	return (uint16_t)((int32_t)ambient +
				  (delta >> CONFIG_MOTH_LIGHT_BACKGROUND_TRACK_SHIFT));
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
	struct moth_light_calibration calibration = { 0 };
	uint16_t filtered_left = 0;
	uint16_t filtered_right = 0;
	bool filter_initialized = false;
	uint32_t sequence = 0;

	int rc = moth_adc_reader_init();
	if (rc != 0) {
		LOG_ERR("ADC initialization failed: %d", rc);
	}

	while (true) {
		frame = (struct moth_light_frame){
			.sequence = sequence++,
			.ambient_left = calibration.ambient_left,
			.ambient_right = calibration.ambient_right,
			.noise_margin_left = calibration.margin_left,
			.noise_margin_right = calibration.margin_right,
			.valid = false,
			.calibrated = calibration.ready,
		};

		if ((rc == 0) && !calibration.ready) {
			rc = calibrate_light(&calibration);
			frame.calibrated = calibration.ready;
			frame.ambient_left = calibration.ambient_left;
			frame.ambient_right = calibration.ambient_right;
			frame.noise_margin_left = calibration.margin_left;
			frame.noise_margin_right = calibration.margin_right;
		}

		if ((rc == 0) && calibration.ready) {
			rc = moth_adc_reader_read(&sample);
			if (rc == 0) {
				uint16_t left_signal = subtract_background(
					sample.left, calibration.ambient_left);
				uint16_t right_signal = subtract_background(
					sample.right, calibration.ambient_right);

				/* Track only weak background changes. */
				if ((left_signal <= CONFIG_MOTH_LIGHT_BACKGROUND_TRACK_LIMIT) &&
				    (right_signal <= CONFIG_MOTH_LIGHT_BACKGROUND_TRACK_LIMIT)) {
					calibration.ambient_left = track_background(
						calibration.ambient_left, sample.left);
					calibration.ambient_right = track_background(
						calibration.ambient_right, sample.right);
					left_signal = subtract_background(
						sample.left, calibration.ambient_left);
					right_signal = subtract_background(
						sample.right, calibration.ambient_right);
				}

				left_signal = subtract_noise_margin(
					left_signal, calibration.margin_left);
				right_signal = subtract_noise_margin(
					right_signal, calibration.margin_right);

				filtered_left = filter_sample(filtered_left, left_signal,
							      filter_initialized);
				filtered_right = filter_sample(filtered_right, right_signal,
							       filter_initialized);
				frame.left = filtered_left;
				frame.right = filtered_right;
				filter_initialized = true;
				frame.ambient_left = calibration.ambient_left;
				frame.ambient_right = calibration.ambient_right;
				frame.valid = true;
			} else {
				LOG_ERR("ADC read failed: %d", rc);
			}
		}

		publish_frame(&frame);
		k_sleep(K_MSEC(MOTH_LIGHT_PERIOD_MS));

		/* A transient read/init error should not permanently disable acquisition. */
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
