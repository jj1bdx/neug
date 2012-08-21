/*
 * random.c - random number generation
 *
 * Copyright (C) 2011 Free Software Initiative of Japan
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


#define EPOOL_SIZE (64 / sizeof (uint32_t))
#define EP_INDEX(cnt, i)  ((cnt + i) & (EPOOL_SIZE - 1))
#define EP_INCREMENT(cnt) ((cnt + 15) & (EPOOL_SIZE - 1))

static uint8_t ep_count;
static uint32_t epool[EPOOL_SIZE];


/*
 * We did an experiment of measuring entropy of ADC output with MUST.
 * The entropy of a byte by raw sampling of LSBs has more than 6.0 bit/byte.
 * So, it is considered OK to get 4-byte from 6-byte (6x6 = 36 > 32).
 */
#define NUM_NOISE_INPUTS 6

static void well512a_step (void)
{
  uint32_t v = epool[ep_count];	/* V0 */
  uint32_t z1, z2;

  z1  = (v ^ (v << 16));
  v = epool[EP_INDEX (ep_count, 13)]; /* VM1 */
  z1 ^= (v ^ (v << 15));
  v = epool[EP_INDEX (ep_count, 9)]; /* VM2 */
  z2 = v ^ (v >> 11);
  v = z1 ^ z2;
  
  /* newV1 */
  epool[ep_count] = v;

  v = (v ^ ((v<<5) & 0xda442d24U));
  z1 = (z1 ^ (z1 << 18));
  z2 = (z2 << 28);
  ep_count = EP_INCREMENT (ep_count);

  /* newV0 */
  epool[ep_count] ^= (epool[ep_count] << 2) ^ z1 ^ z2 ^ v;
}

#define FNV_INIT	2166136261U
#define FNV_PRIME	16777619

static void ep_add (uint8_t entropy_bits, uint8_t another_random_bit,
		    uint32_t round)
{
  uint32_t v = FNV_INIT;

  v ^= entropy_bits;
  v *= FNV_PRIME;
  v ^= ((round >> 19) & 0xff) ^ another_random_bit;
  v *= FNV_PRIME;

  epool[ep_count] ^= v;
  if ((round % 5) == 0 && another_random_bit)
    well512a_step ();
}

static uint32_t ep_output (void)
{
  well512a_step ();
  return epool[ep_count];
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

#define PROBABILITY_50_BY_TICK() ((SysTick->VAL & 0x02) != 0)

/**
 * @brief  Random number generation from ADC sampling.
 * @param  RB: Pointer to ring buffer structure
 * @return -1 when failure, 0 otherwise.
 * @note   Called holding the mutex, with RB->full == 0.
 *         Keep generating until RB->full == 1.
 */
static int rng_gen (struct rng_rb *rb)
{
  static uint32_t round = 0;
  uint8_t b;

  while (1)
    {
      chEvtWaitOne (ADC_DATA_AVAILABLE);

      /* Got, ADC sampling data */
      b = (((samp[0] & 0x01) << 0) | ((samp[1] & 0x01) << 1)
	   | ((samp[2] & 0x01) << 2) | ((samp[3] & 0x01) << 3)
	   | ((samp[4] & 0x01) << 4) | ((samp[5] & 0x01) << 5)
	   | ((samp[6] & 0x01) << 6) | ((samp[7] & 0x01) << 7));

      adcStartConversion (&ADCD1, &adcgrpcfg, samp, ADC_GRP1_BUF_DEPTH);

      /*
       * Put a random byte to entropy pool.
       */
      ep_add (b, PROBABILITY_50_BY_TICK (), round);
      round++;
      if ((round % NUM_NOISE_INPUTS) == 0)
	{		            /* We have enough entropy in the pool.  */
	  uint32_t v = ep_output (); /* Get the random bits from the pool.  */

	  /* We got the final random bits, add it to the ring buffer.  */
	  rb_add (rb, v);
	  if (rb->full)
	    /* fully generated */
	    break;
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

  while (1)
    {
      chMtxLock (&rb->m);
      while (rb->full)
	chCondWait (&rb->space_available);
      rng_gen (rb);
      chCondSignal (&rb->data_available);
      chMtxUnlock ();
    }

  return 0;
}

static struct rng_rb the_ring_buffer;
static WORKING_AREA(wa_rng, 128);

/**
 * @brief Initialize NeuG.
 */
void
neug_init (uint32_t *buf, uint8_t size)
{
  struct rng_rb *rb = &the_ring_buffer;

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
