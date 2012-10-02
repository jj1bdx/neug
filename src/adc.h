#define ADC_DATA_AVAILABLE ((eventmask_t)1)
extern Thread *rng_thread;

#define NEUG_SAMPLE_BUFSIZE     512
extern uint16_t adc_samp[NEUG_SAMPLE_BUFSIZE];

void adc_start (void);
void adc_start_conversion (int offset, int size);
void adc_stop (void);
