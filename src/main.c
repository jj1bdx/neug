/*
 * main.c - main routine of neug
 *
 * Copyright (C) 2011 Free Software Initiative of Japan
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * This file is a part of NeuG, a Random Number Generator
 * implementation.
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
#include "board.h"
#include "usb_cdc.h"

/*
 * We are trying to avoid dependency to C library. 
 * GCC built-in functions are declared here.
 */
void *memcpy(void *dest, const void *src, size_t n);

/*
 * USB Driver structure.
 */
static SerialUSBDriver SDU1;

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0200,        /* bcdUSB (2.0).                    */
                         0x02,          /* bDeviceClass (CDC).              */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         0x40,          /* bMaxPacketSize.                  */
                         0x234b,        /* idVendor (FSIJ).                   */
                         0x0001,        /* idProduct.                       */
                         0x0100,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[67] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(67,            /* wTotalLength.                    */
                         0x02,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0x80,          /* bmAttributes (bus powered).      */
                         50),           /* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x01,          /* bNumEndpoints.                   */
                         0x02,          /* bInterfaceClass (Communications
                                           Interface Class, CDC section
                                           4.2).                            */
                         0x02,          /* bInterfaceSubClass (Abstract
                                         Control Model, CDC section 4.3).   */
                         0x01,          /* bInterfaceProtocol (AT commands,
                                           CDC section 4.4).                */
                         0),            /* iInterface.                      */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  USB_DESC_BYTE         (5),            /* bLength.                         */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x00),         /* bDescriptorSubtype (Header
                                           Functional Descriptor.           */
  USB_DESC_BCD          (0x0110),       /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x01),         /* bDescriptorSubtype (Call Management
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x03),         /* bmCapabilities (D0+D1).          */
  USB_DESC_BYTE         (0x01),         /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x02),         /* bDescriptorSubtype (Abstract
                                           Control Management Descriptor).  */
  USB_DESC_BYTE         (0x02),         /* bmCapabilities.                  */
  /* Union Functional Descriptor.*/
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x06),         /* bDescriptorSubtype (Union
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bMasterInterface (Communication
                                           Class Interface).                */
  USB_DESC_BYTE         (0x01),         /* bSlaveInterface0 (Data Class
                                           Interface).                      */
  /* Endpoint 2 Descriptor.*/
  USB_DESC_ENDPOINT     (INTERRUPT_REQUEST_EP|0x80, /* bEndpointAddress.    */
                         0x03,          /* bmAttributes (Interrupt).        */
                         0x0008,        /* wMaxPacketSize.                  */
                         0xFF),         /* bInterval.                       */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x02,          /* bNumEndpoints.                   */
                         0x0A,          /* bInterfaceClass (Data Class
                                           Interface, CDC section 4.5).     */
                         0x00,          /* bInterfaceSubClass (CDC section
                                           4.6).                            */
                         0x00,          /* bInterfaceProtocol (CDC section
                                           4.7).                            */
                         0x00),         /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  USB_DESC_ENDPOINT     (DATA_AVAILABLE_EP,         /* bEndpointAddress.    */
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00),         /* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (DATA_REQUEST_EP|0x80,      /* bEndpointAddress.    */
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00)          /* bInterval.                       */
};


/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] = {
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[] = {
  USB_DESC_BYTE(68),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  /* Manufacturer: "Free Software Initiative of Japan" */
  'F', 0, 'r', 0, 'e', 0, 'e', 0, ' ', 0, 'S', 0, 'o', 0, 'f', 0,
  't', 0, 'w', 0, 'a', 0, 'r', 0, 'e', 0, ' ', 0, 'I', 0, 'n', 0,
  'i', 0, 't', 0, 'i', 0, 'a', 0, 't', 0, 'i', 0, 'v', 0, 'e', 0,
  ' ', 0, 'o', 0, 'f', 0, ' ', 0, 'J', 0, 'a', 0, 'p', 0, 'a', 0,
  'n', 0
};

/*
 * Device Description string.
 */
static const uint8_t vcom_string2[] = {
  USB_DESC_BYTE(18),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  /* Product name: "NeuG RNG" */
  'N', 0, 'e', 0, 'u', 0, 'G', 0, ' ', 0, 'R', 0, 'N', 0, 'G', 0,
};

/*
 * Serial Number string.
 */
static const uint8_t vcom_string3[] = {
  USB_DESC_BYTE(8),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  '0' , 0,  '.' , 0,  '1' , 0
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {

  (void)usbp;
  (void)lang;
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4)
      return &vcom_strings[dindex];
  }
  return NULL;
}

/**
 * @brief   EP1 initialization structure (IN only).
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_BULK | USB_EP_MODE_PACKET,
  NULL,
  sduDataTransmitted,
  NULL,
  0x0040,
  0x0000,
  NULL,
  NULL
};

/**
 * @brief   EP2 initialization structure (IN only).
 */
static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_INTR | USB_EP_MODE_PACKET,
  NULL,
  sduInterruptTransmitted,
  NULL,
  0x0010,
  0x0000,
  NULL,
  NULL
};

/**
 * @brief   EP3 initialization structure (OUT only).
 */
static const USBEndpointConfig ep3config = {
  USB_EP_MODE_TYPE_BULK | USB_EP_MODE_PACKET,
  NULL,
  NULL,
  sduDataReceived,
  0x0000,
  0x0040,
  NULL,
  NULL
};

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {

  switch (event) {
  case USB_EVENT_RESET:
    return;
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    /* Enables the endpoints specified into the configuration.
       Note, this callback is invoked from an ISR so I-Class functions
       must be used.*/
    chSysLockFromIsr();
    usbInitEndpointI(usbp, DATA_REQUEST_EP, &ep1config);
    usbInitEndpointI(usbp, INTERRUPT_REQUEST_EP, &ep2config);
    usbInitEndpointI(usbp, DATA_AVAILABLE_EP, &ep3config);
    chSysUnlockFromIsr();
    return;
  case USB_EVENT_SUSPEND:
    return;
  case USB_EVENT_WAKEUP:
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}


/*
 * Serial over USB driver configuration.
 */
static const SerialUSBConfig serusbcfg = {
  &USBD1,
  {
    usb_event,
    get_descriptor,
    sduRequestsHook,
    NULL
  }
};

Thread *main_thread;

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_NUM_CHANNELS   2
 
/* Depth of the conversion buffer, channels are sampled one time each.*/
#define ADC_GRP1_BUF_DEPTH      4
 
/*
 * ADC samples buffer.
 */
static adcsample_t samp[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
 
#define ADC_SAMPLE_1P5          0   /**< @brief 1.5 cycles sampling time.   */
#define ADC_SAMPLE_13P5         2   /**< @brief 13.5 cycles sampling time.  */
#define ADC_SAMPLE_239P5        7   /**< @brief 239.5 cycles sampling time. */

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
    chEvtSignalFlagsI (main_thread, (eventmask_t)1);
  chSysUnlockFromIsr();
}

static volatile uint8_t fatal_code;

#define TMT_MAT1 0x8f7011ee
#define TMT_MAT2 0xfc78ff1f
#define TMT_TMAT 0x3793fdff

static uint32_t tmt[4] = { 0x56781234, TMT_MAT1, TMT_MAT2, TMT_TMAT };

#define TMT_CALC_LASTWORD(y,v) (y^v)

static void tmt_one_step (uint32_t v)
{
  uint32_t x, y;

  y = tmt[3];
  x = (tmt[0] & 0x7fffffff) ^ tmt[1] ^ tmt[2];
  x ^= (x << 1);
  y ^= (y >> 1) ^ x;
  tmt[0] = tmt[1];
  tmt[1] = tmt[2];
  tmt[2] = x ^ (y << 10);
  tmt[3] = TMT_CALC_LASTWORD(y, v);
  if ((y & 1))
    {
      tmt[1] ^= TMT_MAT1;
      tmt[2] ^= TMT_MAT2;
    }
}

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
  unsigned int v = (epool[ep_count] << 24)
		    | (epool[(ep_count + 1) & 0x0f] << 16)
		    | (epool[(ep_count + 2) & 0x0f] << 8)
		    | epool[(ep_count + 3) & 0x0f];
  return v;
}



/*
 * Entry point.
 *
 * NOTE: the main function is already a thread in the system on entry.
 */
int
main (int argc, char **argv)
{
  int count = 0;

  (void)argc;
  (void)argv;

  halInit();
  chSysInit();

  main_thread = chThdSelf ();

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  USB_Cable_Config (ENABLE);

  adcStart (&ADCD1, NULL);
  adcStartConversion (&ADCD1, &adcgrpcfg, samp, ADC_GRP1_BUF_DEPTH);

  while (1)
    {
      eventmask_t m;

      count++;

      m = chEvtWaitOne (ALL_EVENTS);

      if (m == (eventmask_t)1)
	{
	  static uint8_t round;
	  uint8_t b;

	  if ((round & 1))
	    b = (((samp[0] & 0x01) << 0) | ((samp[1] & 0x01) << 1)
		 | ((samp[2] & 0x01) << 2) | ((samp[3] & 0x01) << 3)
		 | ((samp[4] & 0x01) << 4) | ((samp[5] & 0x01) << 5)
		 | ((samp[6] & 0x01) << 6) | ((samp[7] & 0x01) << 7));
	  else
	    b = (((samp[0] & 0x01) << 1) | ((samp[1] & 0x01) << 0)
		 | ((samp[2] & 0x01) << 3) | ((samp[3] & 0x01) << 2)
		 | ((samp[4] & 0x01) << 5) | ((samp[5] & 0x01) << 4)
		 | ((samp[6] & 0x01) << 7) | ((samp[7] & 0x01) << 6));

	  adcStartConversion (&ADCD1, &adcgrpcfg, samp,
			      ADC_GRP1_BUF_DEPTH);

	  ep_add (b);
	  if (++round >= 7)
	    {
	      static uint8_t r;
	      char s[32];
	      uint32_t x = ep_value ();

	      tmt_one_step (((SysTick->VAL) & 6) == 0);
	      x ^= tmt_value ();

	      memcpy (s + (r&7)*4, (const char *)&x, 4);
	      r = (r + 1) & 7;
	      if (r == 0 && SDU1.config->usbp->state == USB_ACTIVE)
		{
		  static uint8_t led;

		  led++;
		  set_led ((led & 0x80) == 0);

		  chIQResetI (&(SDU1.iqueue)); /* Ignore input */
		  chIOWriteTimeout (&SDU1, (uint8_t *)s, 32, TIME_INFINITE);
		}

	      round = 0;
	    }
	}
    }

  return 0;
}

void
fatal (uint8_t code)
{
  fatal_code = code;

  set_led (1);
  for (;;);
}
