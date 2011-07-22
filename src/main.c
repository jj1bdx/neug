/*
 * main.c - main routine of neug
 *
 * USB-CDC part:
 * ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,2011 Giovanni Di Sirio.
 *
 * Main routine:
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
#include "neug.h"

/*
 * We are trying to avoid dependency to C library. 
 * GCC built-in function(s) are declared here.
 */
void *memcpy(void *dest, const void *src, size_t n);


/*
 * USB CDC stuff taken from ChibiOS/RT's testhal/STM32/USB_CDC/main.c,
 * and modified for NeuG.
 */

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
                         0x234b,        /* idVendor (FSIJ).                 */
                         0x0001,        /* idProduct (NeoG).                */
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
 * Serial Number string.  NOTE: This does not have CONST qualifier.
 */
static uint8_t vcom_string3[] = {
  USB_DESC_BYTE(28),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  '0', 0,  '.', 0,  '0', 0,  '0', 0,    /* Version number of NeuG.          */
  '-', 0,
  0, 0, 0, 0, 0, 0, 0, 0,	        /* Filled by Unique device ID.      */
  0, 0, 0, 0, 0, 0, 0, 0,
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

static void fill_serial_no_by_unique_id (void)
{
  extern const uint8_t * unique_device_id (void);
  uint8_t *p = &vcom_string3[12];
  const uint8_t *u = unique_device_id ();
  int i;

  for (i = 0; i < 4; i++)
    {
      uint8_t b = u[i];
      uint8_t nibble; 

      nibble = (b >> 4);
      nibble += (nibble >= 10 ? ('A' - 10) : '0');
      p[i*4] = nibble;
      nibble = (b & 0x0f);
      nibble += (nibble >= 10 ? ('A' - 10) : '0');
      p[i*4+2] = nibble;
    }
}

static WORKING_AREA(wa_led, 64);

#define LED_ONESHOT ((eventmask_t)1)
static Thread *led_thread;

/*
 * LED blinker: When notified, let LED emit for 100ms.
 */
static msg_t led_blinker (void *arg)
{
  (void)arg;

  led_thread = chThdSelf ();
  set_led (0);

  while (1)
    {
      chEvtWaitOne (LED_ONESHOT);
      set_led (1);
      chThdSleep (MS2ST (100));
      set_led (0);
    }

  return 0;
}

#define RANDOM_BYTES_LENGTH 32
static uint32_t random_word[RANDOM_BYTES_LENGTH/sizeof (uint32_t)];

/*
 * Entry point.
 *
 * NOTE: the main function is already a thread in the system on entry.
 */
int
main (int argc, char **argv)
{
  unsigned int count = 0;

  (void)argc;
  (void)argv;

  fill_serial_no_by_unique_id ();

  halInit();
  chSysInit();

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  USB_Cable_Config (ENABLE);

  chThdCreateStatic (wa_led, sizeof (wa_led), NORMALPRIO, led_blinker, NULL);

  neug_init (random_word, RANDOM_BYTES_LENGTH/sizeof (uint32_t));

  while (1)
    {
      uint32_t v;
      const uint8_t *s = (const uint8_t *)&v;

      while (count++ < NEUG_PRE_LOOP
	     || SDU1.config->usbp->state != USB_ACTIVE)
	{
	  v = neug_get (NEUG_KICK_FILLING);
	  if ((count & 0x000f) == 0)
	    chEvtSignalFlags (led_thread, LED_ONESHOT);
	  chThdSleep (MS2ST (25));
	  count++;
	}

      count = 0;

      while (1)
	{
	  count++;

	  if (SDU1.config->usbp->state != USB_ACTIVE)
	    break;

	  v = neug_get (NEUG_KICK_FILLING);

	  if ((count & 0x07ff) == 0)
	    chEvtSignalFlags (led_thread, LED_ONESHOT);
	  /*
	   * Ignore input, just in case /dev/ttyACM0 echos our output
	   */
	  chIQResetI (&(SDU1.iqueue));
	  chIOWriteTimeout (&SDU1, s, sizeof (v), TIME_INFINITE);
	}
    }

  return 0;
}
