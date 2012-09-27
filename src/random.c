/*
 * random.c - random number generation
 *
 * Copyright (C) 2011, 2012 Free Software Initiative of Japan
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * This file is a part of NeuG, a Random Number Generator
 * implementation based on quantization error of ADC (for STM32F103).
 *
 * NeuG is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Gnuk is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <string.h>		/* for memcpy */
#include "config.h"

#include "ch.h"
#include "hal.h"

static Thread *rng_thread;
#define ADC_DATA_AVAILABLE ((eventmask_t)1)

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_NUM_CHANNELS   1
 
/* Depth of the conversion buffer, channels are sampled one time each.*/
#define ADC_GRP1_BUF_DEPTH      256

static void adc2_start (void)
{
  rccEnableAPB2 (RCC_APB2ENR_ADC2EN, FALSE);
  ADC2->CR1 = 0;
  ADC2->CR2 = ADC_CR2_ADON;
  ADC2->CR2 = ADC_CR2_ADON | ADC_CR2_RSTCAL;
  while ((ADC2->CR2 & ADC_CR2_RSTCAL) != 0)
    ;
  ADC2->CR2 = ADC_CR2_ADON | ADC_CR2_CAL;
  while ((ADC2->CR2 & ADC_CR2_CAL) != 0)
    ;
  ADC2->CR2 = 0;

  ADC2->CR1 = ADC_CR1_DUALMOD_2 | ADC_CR1_DUALMOD_1 | ADC_CR1_DUALMOD_0;
  ADC2->CR2 = ADC_CR2_DMA | ADC_CR2_CONT | ADC_CR2_ADON;
#ifdef NEUG_NON_DEFAULT_ADC_CHANNEL
  ADC2->SMPR1 = 0;
  ADC2->SMPR2 = ADC_SMPR2_SMP_ANx_B(ADC_SAMPLE_1P5);
#else
  ADC2->SMPR1 = ADC_SMPR1_SMP_AN11(ADC_SAMPLE_1P5);
  ADC2->SMPR2 = 0;
#endif
  ADC2->SQR1 = ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS);
  ADC2->SQR2 = 0;
#ifdef NEUG_NON_DEFAULT_ADC_CHANNEL
  ADC2->SQR3 = ADC_SQR3_SQ1_N(NEUG_ADC_CHANNEL_B);
#else
  ADC2->SQR3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN11);
#endif

  ADC2->CR2 |= ADC_CR2_EXTTRIG | ADC_CR2_SWSTART;
}

static void adc2_stop (void)
{
  rccDisableAPB2 (RCC_APB2ENR_ADC2EN, FALSE);
}

/*
 * ADC samples buffer.
 */
static adcsample_t samp[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH * 2];
 
static void adccb (ADCDriver *adcp, adcsample_t *buffer, size_t n);
static void adccb_err (ADCDriver *adcp, adcerror_t err);

/*
 * ADC conversion group.
 * Mode: Dual fast interleaved mode.
 *   ADC1: master, 16 samples of 1 channels.
 *   ADC2: slave,  16 samples of 1 channels.
 * Channels:
 *   ADC1:
 *     IN10 (1.5 cycles sample time, port configured as push pull output 50MHz)
 *   ADC2:
 *     IN11 (1.5 cycles sample time, port configured as push pull output 50MHz)
 */
static const ADCConversionGroup adcgrpcfg = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  adccb,
  adccb_err,
  ADC_CR1_DUALMOD_2 | ADC_CR1_DUALMOD_1 | ADC_CR1_DUALMOD_0,
  ADC_CR2_EXTTRIG | ADC_CR2_SWSTART | ADC_CR2_EXTSEL,
#ifdef NEUG_NON_DEFAULT_ADC_CHANNEL
  0,
  ADC_SMPR2_SMP_ANx_A(ADC_SAMPLE_1P5),
#else
  ADC_SMPR1_SMP_AN10(ADC_SAMPLE_1P5),
  0,
#endif
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,
#if NEUG_NON_DEFAULT_ADC_CHANNEL
  ADC_SQR3_SQ1_N(NEUG_ADC_CHANNEL_A)
#else
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10)
#endif
};

/*
 * ADC end conversion callback.
 */
static void adccb (ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
  (void) buffer; (void) n;

  chSysLockFromIsr();
  if (adcp->state == ADC_COMPLETE && rng_thread)
    chEvtSignalFlagsI (rng_thread, ADC_DATA_AVAILABLE);
  chSysUnlockFromIsr();
}

static void adccb_err (ADCDriver *adcp, adcerror_t err)
{
  (void)adcp;  (void)err;
}

#include "sha256.h"

static sha256_context sha256_ctx_data;
static uint32_t sha256_output[SHA256_DIGEST_SIZE/sizeof (uint32_t)];

/*
 * We did an experiment of measuring entropy of ADC output with MUST.
 * The entropy of a byte by raw sampling of LSBs has more than 6.0 bit/byte.
 *
 * More tests will be required, but for now we assume min-entropy >= 5.0.
 * 
 * To be a full entropy source, the requirement is to have N samples for
 * output of 256-bit, where:
 *
 *      N = (256 * 2) / <min-entropy of a sample>
 *
 * For min-entropy = 5.0, N should be more than 103.
 *
 * On the other hand, in the section 6.2 "Full Entropy Source Requirements",
 * it says:
 *
 *     At least twice the block size of the underlying cryptographic
 *     primitive shall be provided as input to the conditioning
 *     function to produce full entropy output.
 *
 * For us, cryptographic primitive is SHA-256 and its blocksize is 512-bit
 * (64-byte), N >= 128.
 *
 * We chose N=139, since we love prime number, and we have "additional bits"
 * of 32-byte for last block (feedback from previous output of SHA-256).
 *
 * This corresponds to min-entropy >= 3.68.
 *
 */
#define NUM_NOISE_INPUTS 139

#define EP_ROUND_0 0 /* initial-five-byte and 59-sample-input */
#define EP_ROUND_1 1 /* 64-sample-input */
#define EP_ROUND_2 2 /* 8-sample-input */
#define EP_ROUND_RAW 3 /* 8-sample-input */

#define EP_ROUND_0_INPUTS 59
#define EP_ROUND_1_INPUTS 64
#define EP_ROUND_2_INPUTS 16
#define EP_ROUND_RAW_INPUTS 64

static uint8_t ep_round;

/*
 * Hash_df initial string:
 *
 *  1,          : counter = 1
 *  0, 0, 1, 0  : no_of_bits_returned (in big endian)
 */
static void ep_fill_initial_string (void)
{
  memset (samp, 0, 5 * 8 * sizeof (adcsample_t));
  samp[0] = 1;
  samp[3*8] = 1;
}

static void ep_init (int raw)
{
  chEvtClearFlags (ADC_DATA_AVAILABLE);
  if (raw)
    {
      ep_round = EP_ROUND_RAW;
      adcStartConversion (&ADCD1, &adcgrpcfg, samp, EP_ROUND_RAW_INPUTS*8/2);
    }
  else
    {
      ep_round = EP_ROUND_0;
      ep_fill_initial_string ();
      /*
       * We get two samples for a single transaction of DMA.
       * We take LSBs of each samples.
       * Thus, we need tansactions of: required_number_of_input_in_byte*8/2 
       */
      adcStartConversion (&ADCD1, &adcgrpcfg,
			  &samp[5*8], EP_ROUND_0_INPUTS*8/2);
    }
}

static uint8_t ep_get_byte_from_samples (int i)
{
  return (  ((samp[i*8+0] & 1) << 0) | ((samp[i*8+1] & 1) << 1)
	  | ((samp[i*8+2] & 1) << 2) | ((samp[i*8+3] & 1) << 3)
	  | ((samp[i*8+4] & 1) << 4) | ((samp[i*8+5] & 1) << 5)
	  | ((samp[i*8+6] & 1) << 6) | ((samp[i*8+7] & 1) << 7));
}

static void noise_source_continuous_test (uint8_t noise);

static void ep_fill_wbuf (int i, int flip)
{
  uint8_t b0, b1, b2, b3;

  b0 = ep_get_byte_from_samples (i*4 + 0);
  b1 = ep_get_byte_from_samples (i*4 + 1);
  b2 = ep_get_byte_from_samples (i*4 + 2);
  b3 = ep_get_byte_from_samples (i*4 + 3);
  noise_source_continuous_test (b0);
  noise_source_continuous_test (b1);
  noise_source_continuous_test (b2);
  noise_source_continuous_test (b3);

  if (flip)
    sha256_ctx_data.wbuf[i] = (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
  else
    sha256_ctx_data.wbuf[i] = (b3 << 24) | (b2 << 16) | (b1 << 8) | b0;
}

/* Here assumes little endian architecture.  */
static int ep_process (int raw)
{
  int i, n, flip;

  if (ep_round == EP_ROUND_0 || ep_round == EP_ROUND_1)
    {
      n = 64 / 4;
      flip = 1;
    }
  else if (ep_round == EP_ROUND_2)
    {
      n = EP_ROUND_2_INPUTS / 4;
      flip = 0;
    }
  else /* ep_round == EP_ROUND_RAW */
    {
      n = EP_ROUND_RAW_INPUTS / 4;
      flip = 0;
    }

  for (i = 0; i < n; i++)
    ep_fill_wbuf (i, flip);

  if (raw)
    {
      ep_init (1);
      return n;
    }
  else
    {
      if (ep_round == EP_ROUND_0)
	{
	  adcStartConversion (&ADCD1, &adcgrpcfg, samp, EP_ROUND_1_INPUTS*8/2);
	  sha256_start (&sha256_ctx_data);
	  sha256_process (&sha256_ctx_data);
	  ep_round++;
	  return 0;
	}
      else if (ep_round == EP_ROUND_1)
	{
	  adcStartConversion (&ADCD1, &adcgrpcfg, samp, EP_ROUND_2_INPUTS*8/2);
	  sha256_process (&sha256_ctx_data);
	  ep_round++;
	  return 0;
	}
      else
	{
	  n = SHA256_DIGEST_SIZE;
	  ep_init (0);
	  memcpy (((uint8_t *)sha256_ctx_data.wbuf)+EP_ROUND_2_INPUTS,
		  sha256_output, n);
	  sha256_ctx_data.total[0] = 5 + NUM_NOISE_INPUTS + n;
	  sha256_finish (&sha256_ctx_data, (uint8_t *)sha256_output);
	  return SHA256_DIGEST_SIZE / sizeof (uint32_t);
	}
    }
}


static const uint32_t *ep_output (int raw)
{
  if (raw)
    return sha256_ctx_data.wbuf;
  else
    return sha256_output;
}

#define REPETITION_COUNT           1
#define ADAPTIVE_PROPORTION_64     2
#define ADAPTIVE_PROPORTION_4096   4

uint32_t neug_err_state;

static void noise_source_error_reset (void)
{
  neug_err_state = 0;
}

static void noise_source_error (uint32_t err)
{
  neug_err_state |= err;

#include "board.h"
#if defined(BOARD_STBEE_MINI)
  palClearPad (IOPORT1, GPIOA_LED2);
#endif
}


/* Cuttoff = 10, when min-entropy = 3.7, W= 2^-30 */
/* ceiling of (1+30/3.7) */
#define REPITITION_COUNT_TEST_CUTOFF 10

static uint8_t rct_a;
static uint8_t rct_b;

static void repetition_count_test (uint8_t sample)
{
  if (rct_a == sample)
    {
      rct_b++;
      if (rct_b >= REPITITION_COUNT_TEST_CUTOFF)
	noise_source_error (REPETITION_COUNT);
   }
  else
    {
      rct_a = sample;
      rct_b = 1;
    }
}

/* Cuttoff = 22, when min-entropy = 3.7, W= 2^-30 */
/* With R, qbinom(1-2^-30,64,2^-3.7) */
#define ADAPTIVE_PROPORTION_64_TEST_CUTOFF 22

static uint8_t ap64t_a;
static uint8_t ap64t_b;
static uint8_t ap64t_s;

static void adaptive_proportion_64_test (uint8_t sample)
{
  if (ap64t_s >= 64)
    {
      ap64t_a = sample;
      ap64t_s = 0;
      ap64t_b = 0;
    }
  else
    {
      ap64t_s++;
      if (ap64t_a == sample)
	{
	  ap64t_b++;
	  if (ap64t_b > ADAPTIVE_PROPORTION_64_TEST_CUTOFF)
	    noise_source_error (ADAPTIVE_PROPORTION_64);
	}
    }
}

/* Cuttoff = 422, when min-entropy = 3.7, W= 2^-30 */
/* With R, qbinom(1-2^-30,4096,2^-3.7) */
#define ADAPTIVE_PROPORTION_4096_TEST_CUTOFF 422

static uint8_t ap4096t_a;
static uint16_t ap4096t_b;
static uint16_t ap4096t_s;

static void adaptive_proportion_4096_test (uint8_t sample)
{
  if (ap4096t_s >= 4096)
    {
      ap4096t_a = sample;
      ap4096t_s = 0;
      ap4096t_b = 0;
    }
  else
    {
      ap4096t_s++;
      if (ap4096t_a == sample)
	{
	  ap4096t_b++;
	  if (ap4096t_b > ADAPTIVE_PROPORTION_4096_TEST_CUTOFF)
	    noise_source_error (ADAPTIVE_PROPORTION_4096);
	}
    }
}

static void noise_source_continuous_test (uint8_t noise)
{
  repetition_count_test (noise);
  adaptive_proportion_64_test (noise);
  adaptive_proportion_4096_test (noise);
}

/*
 * Ring buffer, filled by generator, consumed by neug_get routine.
 */
struct rng_rb {
  uint32_t *buf;
  Mutex m;
  CondVar data_available;
  CondVar space_available;
  uint8_t head, tail;
  uint8_t size;
  unsigned int full :1;
  unsigned int empty :1;
};

static void rb_init (struct rng_rb *rb, uint32_t *p, uint8_t size)
{
  rb->buf = p;
  rb->size = size;
  chMtxInit (&rb->m);
  chCondInit (&rb->data_available);
  chCondInit (&rb->space_available);
  rb->head = rb->tail = 0;
  rb->full = 0;
  rb->empty = 1;
}

static void rb_add (struct rng_rb *rb, uint32_t v)
{
  rb->buf[rb->tail++] = v;
  if (rb->tail == rb->size)
    rb->tail = 0;
  if (rb->tail == rb->head)
    rb->full = 1;
  rb->empty = 0;
}

static uint32_t rb_del (struct rng_rb *rb)
{
  uint32_t v = rb->buf[rb->head++];

  if (rb->head == rb->size)
    rb->head = 0;
  if (rb->head == rb->tail)
    rb->empty = 1;
  rb->full = 0;

  return v;
}

static uint8_t neug_raw;

/**
 * @brief  Random number generation from ADC sampling.
 * @param  RB: Pointer to ring buffer structure
 * @return -1 when failure, 0 otherwise.
 * @note   Called holding the mutex, with RB->full == 0.
 *         Keep generating until RB->full == 1.
 */
static int rng_gen (struct rng_rb *rb)
{
  int n;
  int raw = neug_raw;

  while (1)
    {
      chEvtWaitOne (ADC_DATA_AVAILABLE); /* Got a series of ADC sampling.  */

      if ((n = ep_process (raw)))
	{
	  int i;
	  const uint32_t *vp;

	  vp = ep_output (raw);
	  for (i = 0; i < n; i++)
	    {
	      rb_add (rb, *vp);
	      vp++;
	      if (rb->full)
		return 0;
	    }
	}
    }

  return 0;			/* success */
}

/**
 * @brief Random number generation thread.
 */
static msg_t rng (void *arg)
{
  struct rng_rb *rb = (struct rng_rb *)arg;

  rng_thread = chThdSelf ();

  adcStart (&ADCD1, NULL);
  /* Override DMA settings. */
  ADCD1.dmamode = STM32_DMA_CR_PL(STM32_ADC_ADC1_DMA_PRIORITY)
    | STM32_DMA_CR_MSIZE_WORD | STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MINC
    | STM32_DMA_CR_TCIE       | STM32_DMA_CR_TEIE       | STM32_DMA_CR_EN;
  /* Enable ADC2 */
  adc2_start ();
  ep_init (0);

  while (!chThdShouldTerminate ())
    {
      chMtxLock (&rb->m);
      while (rb->full)
	chCondWait (&rb->space_available);
      while (1)
	{
	  rng_gen (rb);
	  if (neug_err_state != 0)
	    {
	      if (!neug_raw)
		while (!rb->empty)
		  (void)rb_del (rb);
	      noise_source_error_reset ();
	    }
	  else
	    break;
	}
      chCondSignal (&rb->data_available);
      chMtxUnlock ();
    }

  adc2_stop ();
  adcStop (&ADCD1);

  return 0;
}

static struct rng_rb the_ring_buffer;
static WORKING_AREA(wa_rng, 960);

/**
 * @brief Initialize NeuG.
 */
void
neug_init (uint32_t *buf, uint8_t size)
{
  struct rng_rb *rb = &the_ring_buffer;

  neug_raw = 0;
  rb_init (rb, buf, size);
  chThdCreateStatic (wa_rng, sizeof (wa_rng), NORMALPRIO, rng, rb);
}

/**
 * @breif Flush random bytes.
 */
void
neug_flush (void)
{
  struct rng_rb *rb = &the_ring_buffer;

  chMtxLock (&rb->m);
  while (!rb->empty)
    (void)rb_del (rb);
  chCondSignal (&rb->space_available);
  chMtxUnlock ();
}


/**
 * @brief  Wakes up RNG thread to generate random numbers.
 */
void
neug_kick_filling (void)
{
  struct rng_rb *rb = &the_ring_buffer;

  chMtxLock (&rb->m);
  if (!rb->full)
    chCondSignal (&rb->space_available);
  chMtxUnlock ();
}

/**
 * @brief  Get random word (32-bit) from NeuG.
 * @detail With NEUG_KICK_FILLING, it wakes up RNG thread.
 *         With NEUG_NO_KICK, it doesn't wake up RNG thread automatically,
 *         it is needed to call neug_kick_filling later.
 */
uint32_t
neug_get (int kick)
{
  struct rng_rb *rb = &the_ring_buffer;
  uint32_t v;

  chMtxLock (&rb->m);
  while (rb->empty)
    chCondWait (&rb->data_available);
  v = rb_del (rb);
  if (kick)
    chCondSignal (&rb->space_available);
  chMtxUnlock ();

  return v;
}

void
neug_wait_full (void)
{
  struct rng_rb *rb = &the_ring_buffer;

  chMtxLock (&rb->m);
  while (!rb->full)
    chCondWait (&rb->data_available);
  chMtxUnlock ();
}

void
neug_fini (void)
{
  if (rng_thread)
    {
      chThdTerminate (rng_thread);
      neug_get (1);
      chThdWait (rng_thread);
      rng_thread = NULL;
    }
}

void
neug_select (uint8_t raw)
{
  neug_wait_full ();
  if (neug_raw != raw)
    ep_init (raw);
  neug_raw = raw;
  neug_flush ();
}
