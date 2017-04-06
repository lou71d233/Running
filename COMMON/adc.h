#ifndef ADC_H_INCLUDED
#define ADC_H_INCLUDED

#include <ax8052f143.h>
#include <libmf.h>
#include <libmfadc.h>
#include <libmftypes.h>
#include <libmfuart0.h>

#include "../COMMON/misc.h"
#include "../COMMON/display_com0.h"

#define ADC_BUFFER_SIZE 64
#define motor_ch 1
#define battery_ch 0

extern uint16_t __xdata adcbuffer[ADC_BUFFER_SIZE];

extern void adc_init(void);
extern uint8_t adc_work(uint8_t channel);

#endif // ADC_H_INCLUDED
