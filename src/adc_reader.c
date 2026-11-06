#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <moth/adc_reader.h>

LOG_MODULE_REGISTER(moth_adc_reader, LOG_LEVEL_INF);

#define MOTH_USER_NODE DT_PATH(zephyr_user)
#define MOTH_ADC_CHANNEL_COUNT 2

static const struct adc_dt_spec adc_channels[MOTH_ADC_CHANNEL_COUNT] = {
	ADC_DT_SPEC_GET_BY_IDX(MOTH_USER_NODE, 0),
	ADC_DT_SPEC_GET_BY_IDX(MOTH_USER_NODE, 1),
};

int moth_adc_reader_init(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(adc_channels); ++i) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			LOG_ERR("ADC channel %u is not ready", i);
			return -ENODEV;
		}

		int rc = adc_channel_setup_dt(&adc_channels[i]);
		if (rc != 0) {
			LOG_ERR("ADC channel %u setup failed: %d", i, rc);
			return rc;
		}

		LOG_INF("ADC channel %u ready (channel id %d)", i,
			adc_channels[i].channel_id);
	}

	return 0;
}

int moth_adc_reader_read(struct moth_adc_sample *sample)
{
	if (sample == NULL) {
		return -EINVAL;
	}

	for (size_t i = 0; i < ARRAY_SIZE(adc_channels); ++i) {
		struct adc_sequence sequence = { 0 };
		int16_t value = 0;

		int rc = adc_sequence_init_dt(&adc_channels[i], &sequence);
		if (rc != 0) {
			return rc;
		}

		sequence.buffer = &value;
		sequence.buffer_size = sizeof(value);

		rc = adc_read_dt(&adc_channels[i], &sequence);
		if (rc != 0) {
			return rc;
		}

		if (i == 0) {
			sample->left = (uint16_t)value;
		} else {
			sample->right = (uint16_t)value;
		}
	}

	return 0;
}
