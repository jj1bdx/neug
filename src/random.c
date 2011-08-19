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
  0,
  ADC_CR2_EXTSEL_SWSTART | ADC_CR2_TSVREFE | ADC_CR2_CONT,
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

/*
 * TinyMT routines.
 *
 * See
 * "Tiny Mersenne Twister (TinyMT): A small-sized variant of Mersenne Twister"
 * by Mutsuo Saito and Makoto Matsumoto
 *     http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/TINYMT/
 */

/* Use the first example of TinyMT */
#define TMT_MAT1 0x8f7011ee
#define TMT_MAT2 0xfc78ff1f
#define TMT_TMAT 0x3793fdff

static uint32_t tmt[4] = {
  /* Initial state of seed=1, precomputed */
  0x0cca24d8, 0x11ba5ad5, 0xf2dad045, 0xd95dd7b2
};

/**
 * @brief  TinyMT one step function, call this every time before tmt_value.
 */
static void tmt_one_step (void)
{
  uint32_t x, y;

  y = tmt[3];
  x = (tmt[0] & 0x7fffffff) ^ tmt[1] ^ tmt[2];
  x ^= (x << 1);
  y ^= (y >> 1) ^ x;
  tmt[0] = tmt[1];
  tmt[1] = tmt[2];
  tmt[2] = x ^ (y << 10);
  tmt[3] = y;
  if ((y & 1))
    {
      tmt[1] ^= TMT_MAT1;
      tmt[2] ^= TMT_MAT2;
    }
}

/**
 * @brief  Get a random word (32-bit).
 */
static uint32_t tmt_value (void)
{
  uint32_t t0, t1;

  t0 = tmt[3];
  t1 = tmt[0] + (tmt[2] >> 8);
  t0 ^= t1;
  if ((t1 & 1))
    t0 ^= TMT_TMAT;
  return t0;
}

/* 8 parallel CRC-16 shift registers */
static uint8_t epool[16];	/* Big-endian */
static uint8_t ep_count;

static void ep_add (uint8_t bits)
{
  uint8_t v = epool[ep_count];

  /* CRC-16-CCITT's Polynomial is: x^16 + x^12 + x^5 + 1 */
  epool[ep_count] ^= bits;
  epool[(ep_count - 5)& 0x0f] ^= v;
  epool[(ep_count - 12)& 0x0f] ^= v;

  ep_count = (ep_count + 1) & 0x0f;
}

static uint32_t ep_value (void)
{
  uint32_t v = (epool[ep_count] << 24)
		    | (epool[(ep_count + 1) & 0x0f] << 16)
		    | (epool[(ep_count + 2) & 0x0f] << 8)
		    | epool[(ep_count + 3) & 0x0f];
  return v;
}

/* CRC-32 shift register */
static uint32_t crc32;

/*
 * CRC-32-IEEE's Polynomial is:
 *    x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10
 *    + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1
 */
static int crc32_top_bit (void)
{
  int v = (crc32 & 0x80000000) != 0;

  return v;
}

static void crc32_add_bit (int bit)
{
  int v = crc32_top_bit ();

  crc32 = (crc32 << 1) | (bit?1:0);

  if (v)
    crc32 ^= 0x04c11db7;
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

/*
 * We did an experiment of measuring entropy of ADC output with MUST.
 * The entropy of a byte by raw sampling of LSBs has more than 6.0 bit/byte.
 * So, it is considered OK to get 4-byte from 7-byte (6x7 = 42 > 32).
 */
#define NUM_NOISE_INPUTS 7

/**
 * @brief  Random number generation from ADC sampling.
 * @param  RB: Pointer to ring buffer structure
 * @return -1 when failure, 0 otherwise.
 * @note   Called holding the mutex, with RB->full == 0.
 *         Keep generating until RB->full == 1.
 */
static int rng_gen (struct rng_rb *rb)
{
  static uint8_t round = 0;
  uint8_t b;

  while (1)
    {
      chEvtWaitOne (ADC_DATA_AVAILABLE);

      /* Got, ADC sampling data */
      round++;
      if ((round & 1))
	{
	  b = (((samp[0] & 0x01) << 0) | ((samp[1] & 0x01) << 1)
	       | ((samp[2] & 0x01) << 2) | ((samp[3] & 0x01) << 3)
	       | ((samp[4] & 0x01) << 4) | ((samp[5] & 0x01) << 5)
	       | ((samp[6] & 0x01) << 6) | ((samp[7] & 0x01) << 7));
	}
      else
	{
	  b = (((samp[0] & 0x01) << 1) | ((samp[1] & 0x01) << 0)
	       | ((samp[2] & 0x01) << 3) | ((samp[3] & 0x01) << 2)
	       | ((samp[4] & 0x01) << 5) | ((samp[5] & 0x01) << 4)
	       | ((samp[6] & 0x01) << 7) | ((samp[7] & 0x01) << 6));

	  /*
	   * Take second LSB of SysTick->VAL, and put to CRC32 shift register.
	   * It seems that LSB is not good entropy source.
	   */
	  crc32_add_bit ((SysTick->VAL >> 1) & 0x01);
	}

      adcStartConversion (&ADCD1, &adcgrpcfg, samp, ADC_GRP1_BUF_DEPTH);

      /*
       * Put a random byte to entropy pool.
       */
      ep_add (b);

      if ((round % NUM_NOISE_INPUTS) == 0)
	{		            /* We have enough entropy in the pool.  */
	  uint32_t v = ep_value (); /* Get the random bits from the pool.  */

	  /* Mix the random bits from the pool with the output of PRNG.  */
	  tmt_one_step ();
	  if (crc32_top_bit ())	/* We shake tmt's progress by 50%.  */
	    tmt_one_step ();
	  v ^= tmt_value ();

	  /* We got the final random bits, add it to the ring buffer.  */
	  rb_add (rb, v);
	  round = 0;
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
 *         With NEUG_NO_KICK, it doesn't wake up automatically,
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
