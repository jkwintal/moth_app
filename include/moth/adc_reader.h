#ifndef MOTH_ADC_READER_H_
#define MOTH_ADC_READER_H_

#include <stdint.h>

struct moth_adc_sample {
	uint16_t left;
	uint16_t right;
};

int moth_adc_reader_init(void);
int moth_adc_reader_read(struct moth_adc_sample *sample);

#endif
