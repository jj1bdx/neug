/*
 * random.c - random number generation
 *
 * Copyright (C) 2011, 2012 Free Software Initiative of Japan
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * This file is a part of NeuG, a Random Number Generator
 * implementation (for STM32F103).
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
#define ADC_GRP1_NUM_CHANNELS   2
 
/* Depth of the conversion buffer, channels are sampled one time each.*/
#define ADC_GRP1_BUF_DEPTH      4
 
/*
 * ADC samples buffer.
 */
static adcsample_t samp[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
 
static void adccb (ADCDriver *adcp, adcsample_t *buffer, size_t n);
static void adccb_err (ADCDriver *adcp, adcerror_t err);

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 4 samples of 2 channels, SW triggered.
 * Channels:    Vref   (1.5 cycles sample time, violating the spec.)
 *              Sensor (1.5 cycles sample time, violating the spec.)
 */
static const ADCConversionGroup adcgrpcfg = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  adccb,
  adccb_err,
  0,
  ADC_CR2_TSVREFE,
  ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_1P5) | ADC_SMPR1_SMP_VREF(ADC_SAMPLE_1P5),
  0,
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,
  ADC_SQR3_SQ2_N(ADC_CHANNEL_SENSOR) | ADC_SQR3_SQ1_N(ADC_CHANNEL_VREFINT)
};

/*
 * ADC end conversion callback.
 */
static void adccb (ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
  (void) buffer; (void) n;

  chSysLockFromIsr();
  if (adcp->state == ADC_COMPLETE)
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
 * We chose N=131, since we love prime number, and we have "additional bits"
 * of 32-byte for last block (feedback from previous output of SHA-256).
 *
 * This corresponds to min-entropy >= 3.91.
 *
 */
#define NUM_NOISE_INPUTS 131

static const uint8_t hash_df_initial_string[5] = {
  1,          /* counter = 1 */
  0, 0, 1, 0  /* no_of_bits_returned (big endian) */
};

static void ep_init (void)
{
  sha256_start (&sha256_ctx_data);
  sha256_update (&sha256_ctx_data, hash_df_initial_string, 5);
}

static void ep_add (uint8_t entropy_bits)
{
  sha256_update (&sha256_ctx_data, &entropy_bits, 1);
}

#define PROBABILITY_50_BY_TICK() ((SysTick->VAL & 0x02) != 0)

static const uint32_t *ep_output (void)
{
#if ((SHA256_BLOCK_SIZE - 9) - ((5 + NUM_NOISE_INPUTS) % SHA256_BLOCK_SIZE)) \
    > SHA256_DIGEST_SIZE
  int n = SHA256_DIGEST_SIZE;
#else
  int n = (SHA256_BLOCK_SIZE - 9)
    - ((5 + NUM_NOISE_INPUTS) % SHA256_BLOCK_SIZE);
#endif

  if (PROBABILITY_50_BY_TICK ())
    n = n - 3;

  sha256_update (&sha256_ctx_data, (uint8_t *)sha256_output, n);
  sha256_finish (&sha256_ctx_data, (uint8_t *)sha256_output);
  ep_init ();
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
}


/* Cuttoff = 9, when min-entropy = 4.0, W= 2^-30 */
/* ceiling of (1+30/4.0) */
#define REPITITION_COUNT_TEST_CUTOFF 9

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

/* Cuttoff = 20, when min-entropy = 4.0, W= 2^-30 */
/* With R, qbinom(1-2^-30,64,2^-4.0) */
#define ADAPTIVE_PROPORTION_64_TEST_CUTOFF 20

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

/* Cuttoff = 354, when min-entropy = 4.0, W= 2^-30 */
/* With R, qbinom(1-2^-30,4096,2^-4.0) */
#define ADAPTIVE_PROPORTION_4096_TEST_CUTOFF 354

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
  uint8_t round = 0;
  uint32_t v = 0;

  while (1)
    {
      uint8_t b;

      chEvtWaitOne (ADC_DATA_AVAILABLE);

      /* Got an ADC sampling data */
      b = (((samp[0] & 0x01) << 0) | ((samp[1] & 0x01) << 1)
	   | ((samp[2] & 0x01) << 2) | ((samp[3] & 0x01) << 3)
	   | ((samp[4] & 0x01) << 4) | ((samp[5] & 0x01) << 5)
	   | ((samp[6] & 0x01) << 6) | ((samp[7] & 0x01) << 7));

      adcStartConversion (&ADCD1, &adcgrpcfg, samp, ADC_GRP1_BUF_DEPTH);

      noise_source_continuous_test (b);
      if (neug_raw)
	{
	  v |= (b << (round * 8));
	  round++;
	  if (round >= 4)
	    {
	      rb_add (rb, v);
	      if (rb->full)
		return 0;
	      v = 0;
	      round = 0;
	    }
	}
      else
	{
	  /*
	   * Put a random byte to entropy pool.
	   */
	  ep_add (b);
	  round++;
	  if (round >= NUM_NOISE_INPUTS)
	    {
	      /*
	       * We have enough entropy in the pool.
	       * Thus, we pull the random bits from the pool.
	       */
	      int i;
	      const uint32_t *vp = ep_output ();

	      /* We get the random bits, add it to the ring buffer.  */
	      for (i = 0; i < SHA256_DIGEST_SIZE / 4; i++)
		{
		  rb_add (rb, *vp);
		  vp++;
		  if (rb->full)
		    return 0;
		}

	      round = 0;
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
  adcStartConversion (&ADCD1, &adcgrpcfg, samp, ADC_GRP1_BUF_DEPTH);

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

  return 0;
}

static struct rng_rb the_ring_buffer;
static WORKING_AREA(wa_rng, 256);

/**
 * @brief Initialize NeuG.
 */
void
neug_init (uint32_t *buf, uint8_t size)
{
  struct rng_rb *rb = &the_ring_buffer;

  neug_raw = 0;
  ep_init ();
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
  neug_raw = raw;
  neug_flush ();
}
