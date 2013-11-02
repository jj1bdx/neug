/*
 * main.c - main routine of neug
 *
 * Main routine:
 * Copyright (C) 2011, 2012, 2013 Free Software Initiative of Japan
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * This file is a part of NeuG, a True Random Number Generator
 * implementation.
 *
 * NeuG is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NeuG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include <stdint.h>
#include <string.h>
#include <chopstx.h>

#include "config.h"
#include "neug.h"
#include "usb_lld.h"
#include "sys.h"
#include "stm32f103.h"
#include "adc.h"

enum {
  FSIJ_DEVICE_RUNNING = 0,
  FSIJ_DEVICE_EXITED,
  FSIJ_DEVICE_EXEC_REQUESTED,
  /**/
  FSIJ_DEVICE_NEUG_EXIT_REQUESTED = 255
}; 

static chopstx_mutex_t usb_mtx;
static chopstx_cond_t cnd_usb;
static uint32_t bDeviceState = UNCONNECTED; /* USB device status */
static uint8_t fsij_device_state = FSIJ_DEVICE_RUNNING;
static uint8_t connected;
static uint32_t wait_usb_connection; /* Timer variable.  */


extern uint8_t __process0_stack_end__;
static chopstx_t main_thd = (uint32_t)(&__process0_stack_end__ - 60);

#define ENDP0_RXADDR        (0x40)
#define ENDP0_TXADDR        (0x80)
#define ENDP1_TXADDR        (0xc0)
#define ENDP2_TXADDR        (0x100)
#define ENDP3_RXADDR        (0x140)

#define USB_CDC_REQ_SET_LINE_CODING             0x20
#define USB_CDC_REQ_GET_LINE_CODING             0x21
#define USB_CDC_REQ_SET_CONTROL_LINE_STATE      0x22
#define USB_CDC_REQ_SEND_BREAK                  0x23

/* USB Device Descriptor */
static const uint8_t vcom_device_desc[18] = {
  18,   /* bLength */
  USB_DEVICE_DESCRIPTOR_TYPE,	/* bDescriptorType */
  0x10, 0x01,			/* bcdUSB = 1.1 */
  0x02,				/* bDeviceClass (CDC).              */
  0x00,				/* bDeviceSubClass.                 */
  0x00,				/* bDeviceProtocol.                 */
  0x40,				/* bMaxPacketSize.                  */
#include "usb-vid-pid-ver.c.inc"
  1,				/* iManufacturer.                   */
  2,				/* iProduct.                        */
  3,				/* iSerialNumber.                   */
  1				/* bNumConfigurations.              */
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_desc[67] = {
  9,
  USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
  /* Configuration Descriptor.*/
  67, 0x00,			/* wTotalLength.                    */
  0x02,				/* bNumInterfaces.                  */
  0x01,				/* bConfigurationValue.             */
  0,				/* iConfiguration.                  */
  0x80,				/* bmAttributes (bus powered).      */
  50,				/* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  9,
  USB_INTERFACE_DESCRIPTOR_TYPE,
  0x00,		   /* bInterfaceNumber.                */
  0x00,		   /* bAlternateSetting.               */
  0x01,		   /* bNumEndpoints.                   */
  0x02,		   /* bInterfaceClass (Communications Interface Class,
		      CDC section 4.2).  */
  0x02,		   /* bInterfaceSubClass (Abstract Control Model, CDC
		      section 4.3).  */
  0x01,		   /* bInterfaceProtocol (AT commands, CDC section
		      4.4).  */
  0,	           /* iInterface.                      */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  5,	      /* bLength.                         */
  0x24,	      /* bDescriptorType (CS_INTERFACE).  */
  0x00,	      /* bDescriptorSubtype (Header Functional Descriptor). */
  0x10, 0x01, /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  5,            /* bFunctionLength.                 */
  0x24,         /* bDescriptorType (CS_INTERFACE).  */
  0x01,         /* bDescriptorSubtype (Call Management Functional
		   Descriptor). */
  0x03,         /* bmCapabilities (D0+D1).          */
  0x01,         /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  4,            /* bFunctionLength.                 */
  0x24,         /* bDescriptorType (CS_INTERFACE).  */
  0x02,         /* bDescriptorSubtype (Abstract Control Management
		   Descriptor).  */
  0x02,         /* bmCapabilities.                  */
  /* Union Functional Descriptor.*/
  5,            /* bFunctionLength.                 */
  0x24,         /* bDescriptorType (CS_INTERFACE).  */
  0x06,         /* bDescriptorSubtype (Union Functional
		   Descriptor).  */
  0x00,         /* bMasterInterface (Communication Class
		   Interface).  */
  0x01,         /* bSlaveInterface0 (Data Class Interface).  */
  /* Endpoint 2 Descriptor.*/
  7,
  USB_ENDPOINT_DESCRIPTOR_TYPE,
  ENDP2|0x80,    /* bEndpointAddress.    */
  0x03,          /* bmAttributes (Interrupt).        */
  0x08, 0x00,	 /* wMaxPacketSize.                  */
  0xFF,		 /* bInterval.                       */
  /* Interface Descriptor.*/
  9,
  USB_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType: */
  0x01,          /* bInterfaceNumber.                */
  0x00,          /* bAlternateSetting.               */
  0x02,          /* bNumEndpoints.                   */
  0x0A,          /* bInterfaceClass (Data Class Interface, CDC section 4.5). */
  0x00,          /* bInterfaceSubClass (CDC section 4.6). */
  0x00,          /* bInterfaceProtocol (CDC section 4.7). */
  0x00,		 /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  7,
  USB_ENDPOINT_DESCRIPTOR_TYPE,	/* bDescriptorType: Endpoint */
  ENDP3,    /* bEndpointAddress. */
  0x02,				/* bmAttributes (Bulk).             */
  0x40, 0x00,			/* wMaxPacketSize.                  */
  0x00,				/* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  7,
  USB_ENDPOINT_DESCRIPTOR_TYPE,	/* bDescriptorType: Endpoint */
  ENDP1|0x80,			/* bEndpointAddress. */
  0x02,				/* bmAttributes (Bulk).             */
  0x40, 0x00,			/* wMaxPacketSize.                  */
  0x00				/* bInterval.                       */
};


/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[4] = {
  4,				/* bLength */
  USB_STRING_DESCRIPTOR_TYPE,
  0x09, 0x04			/* LangID = 0x0409: US-English */
};

#define USB_STRINGS_FOR_NEUG 1
#include "usb-strings.c.inc"

/*
 * Serial Number string.  NOTE: This does not have CONST qualifier.
 */
static uint8_t vcom_string3[28] = {
  28,                               /* bLength.                         */
  USB_STRING_DESCRIPTOR_TYPE,       /* bDescriptorType.                 */
  '0', 0,  '.', 0,  '1', 0, '1', 0, /* Version number of NeuG.          */
  '-', 0,
  0, 0, 0, 0,	/* Filled by Unique device ID.      */
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
};


#define NUM_INTERFACES 2


void
usb_cb_device_reset (void)
{
  /* Set DEVICE as not configured */
  usb_lld_set_configuration (0);

  /* Current Feature initialization */
  usb_lld_set_feature (vcom_configuration_desc[7]);

  usb_lld_reset ();

  /* Initialize Endpoint 0 */
  usb_lld_setup_endpoint (ENDP0, EP_CONTROL, 0, ENDP0_RXADDR, ENDP0_TXADDR, 64);
}

extern uint8_t _regnual_start, __heap_end__;

static const uint8_t *const mem_info[] = { &_regnual_start,  &__heap_end__, };

/* USB vendor requests to control pipe */
#define USB_FSIJ_MEMINFO	  0
#define USB_FSIJ_DOWNLOAD	  1
#define USB_FSIJ_EXEC		  2
#define USB_NEUG_GET_INFO	254
#define USB_NEUG_EXIT		255 /* Ask to exit and to receive reGNUal */


static uint32_t rbit (uint32_t v)
{
  uint32_t r;

  asm ("rbit	%0, %1" : "=r" (r) : "r" (v));
  return r;
}

/* After calling this function, CRC module remain enabled.  */
static int download_check_crc32 (const uint32_t *end_p)
{
  uint32_t crc32 = *end_p;
  const uint32_t *p;

  RCC->AHBENR |= RCC_AHBENR_CRCEN;
  CRC->CR = CRC_CR_RESET;

  for (p = (const uint32_t *)&_regnual_start; p < end_p; p++)
    CRC->DR = rbit (*p);

  if ((rbit (CRC->DR) ^ crc32) == 0xffffffff)
    return USB_SUCCESS;

  return USB_UNSUPPORT;
}

void
usb_cb_ctrl_write_finish (uint8_t req, uint8_t req_no, uint16_t value,
			  uint16_t index, uint16_t len)
{
  uint8_t type_rcp = req & (REQUEST_TYPE|RECIPIENT);

  if (type_rcp == (VENDOR_REQUEST | DEVICE_RECIPIENT)
      && USB_SETUP_SET (req) && len == 0)
    {
      if (req_no == USB_FSIJ_EXEC)
	{
	  chopstx_mutex_lock (&usb_mtx);
	  if (fsij_device_state == FSIJ_DEVICE_EXITED)
	    {
	      usb_lld_prepare_shutdown (); /* No further USB communication */
	      fsij_device_state = FSIJ_DEVICE_EXEC_REQUESTED;
	    }
	  chopstx_mutex_unlock (&usb_mtx);
	}
      else if (req_no == USB_NEUG_EXIT)
	{
	  /* Force exit from the main loop.  */
	  if (wait_usb_connection)
	    {
	      wait_usb_connection = 0;
	      chopstx_wakeup_usec_wait (main_thd);
	    }
	  else
	    {
	      chopstx_mutex_lock (&usb_mtx);
	      chopstx_cond_signal (&cnd_usb);
	      chopstx_mutex_unlock (&usb_mtx);
	    }
	}
    }
  else if (type_rcp == (CLASS_REQUEST | INTERFACE_RECIPIENT)
	   && index == 0 && USB_SETUP_SET (req)
	   && req_no == USB_CDC_REQ_SET_CONTROL_LINE_STATE)
    {
      /* Open/close the connection.  */
      chopstx_mutex_lock (&usb_mtx);
      connected = (value != 0)? 1 : 0;
      if (wait_usb_connection)
	{			/* It is waiting a connection.  */
	  if (connected)	/* It's now connected.  */
	    {
	      wait_usb_connection = 0;
	      chopstx_wakeup_usec_wait (main_thd);
	    }
	}
      else
	chopstx_cond_signal (&cnd_usb);
      chopstx_mutex_unlock (&usb_mtx);
    }
}

struct line_coding
{
  uint32_t bitrate;
  uint8_t format;
  uint8_t paritytype;
  uint8_t datatype;
} __attribute__((packed));

static struct line_coding line_coding = {
  115200, /* baud rate: 115200    */
  0x00,   /* stop bits: 1         */
  0x00,   /* parity:    none      */
  0x08    /* bits:      8         */
};


static int
vcom_port_data_setup (uint8_t req, uint8_t req_no, uint16_t value,
		      uint16_t len)
{
  (void)value;
  if (USB_SETUP_GET (req))
    {
      if (req_no == USB_CDC_REQ_GET_LINE_CODING
	  && len == sizeof (line_coding))
	{
	  usb_lld_set_data_to_send (&line_coding, sizeof (line_coding));
	  return USB_SUCCESS;
	}
    }
  else  /* USB_SETUP_SET (req) */
    {
      if (req_no == USB_CDC_REQ_SET_LINE_CODING
	  && len == sizeof (line_coding))
	{
	  usb_lld_set_data_to_recv (&line_coding, sizeof (line_coding));
	  return USB_SUCCESS;
	}
      else if (req_no == USB_CDC_REQ_SET_CONTROL_LINE_STATE)
	return USB_SUCCESS;
    }

  return USB_UNSUPPORT;
}

int
usb_cb_setup (uint8_t req, uint8_t req_no,
	       uint16_t value, uint16_t index, uint16_t len)
{
  uint8_t type_rcp = req & (REQUEST_TYPE|RECIPIENT);

  if (type_rcp == (VENDOR_REQUEST | DEVICE_RECIPIENT))
    {
      if (USB_SETUP_GET (req))
	{
	  if (req_no == USB_FSIJ_MEMINFO)
	    {
	      usb_lld_set_data_to_send (mem_info, sizeof (mem_info));
	      return USB_SUCCESS;
	    }
	  else if (req_no == USB_NEUG_GET_INFO)
	    {
	      if (index == 0)
		usb_lld_set_data_to_send (&neug_mode, sizeof (uint8_t));
	      else if (index == 1)
		usb_lld_set_data_to_send (&neug_err_cnt, sizeof (uint16_t));
	      else if (index == 2)
		usb_lld_set_data_to_send (&neug_err_cnt_rc, sizeof (uint16_t));
	      else if (index == 3)
		usb_lld_set_data_to_send (&neug_err_cnt_p64, sizeof (uint16_t));
	      else if (index == 4)
		usb_lld_set_data_to_send (&neug_err_cnt_p4k, sizeof (uint16_t));
	      else if (index == 5)
		usb_lld_set_data_to_send (&neug_rc_max, sizeof (uint16_t));
	      else if (index == 6)
		usb_lld_set_data_to_send (&neug_p64_max, sizeof (uint16_t));
	      else if (index == 7)
		usb_lld_set_data_to_send (&neug_p4k_max, sizeof (uint16_t));
	      else
		return USB_UNSUPPORT;

	      return USB_SUCCESS;
	    }
	}
      else /* SETUP_SET */
	{
	  uint8_t *addr = (uint8_t *)(0x20000000 + value * 0x100 + index);

	  if (req_no == USB_FSIJ_DOWNLOAD)
	    {
	      chopstx_mutex_lock (&usb_mtx);
	      if (fsij_device_state != FSIJ_DEVICE_EXITED)
		{
		  chopstx_mutex_unlock (&usb_mtx);
		  return USB_UNSUPPORT;
		}
	      chopstx_mutex_unlock (&usb_mtx);

	      if (addr < &_regnual_start || addr + len > &__heap_end__)
		return USB_UNSUPPORT;

	      if (index + len < 256)
		memset (addr + index + len, 0, 256 - (index + len));

	      usb_lld_set_data_to_recv (addr, len);
	      return USB_SUCCESS;
	    }
	  else if (req_no == USB_FSIJ_EXEC && len == 0)
	    {
	      chopstx_mutex_lock (&usb_mtx);
	      if (fsij_device_state != FSIJ_DEVICE_EXITED)
		{
		  chopstx_mutex_unlock (&usb_mtx);
		  return USB_UNSUPPORT;
		}
	      chopstx_mutex_unlock (&usb_mtx);

	      if (((uint32_t)addr & 0x03))
		return USB_UNSUPPORT;

	      return download_check_crc32 ((uint32_t *)addr);
	    }
	  else if (req_no == USB_NEUG_EXIT && len == 0)
	    {
	      chopstx_mutex_lock (&usb_mtx);
	      if (fsij_device_state != FSIJ_DEVICE_RUNNING)
		{
		  chopstx_mutex_unlock (&usb_mtx);
		  return USB_UNSUPPORT;
		}

	      fsij_device_state = FSIJ_DEVICE_NEUG_EXIT_REQUESTED;
	      chopstx_wakeup_usec_wait (main_thd);
	      chopstx_cond_signal (&cnd_usb);
	      chopstx_mutex_unlock (&usb_mtx);

	      return USB_SUCCESS;
	    }
	}
    }
  else if (type_rcp == (CLASS_REQUEST | INTERFACE_RECIPIENT))
    if (index == 0)
      return vcom_port_data_setup (req, req_no, value, len);

  return USB_UNSUPPORT;
}

int
usb_cb_get_descriptor (uint8_t rcp, uint8_t desc_type, uint8_t desc_index,
		       uint16_t index);
{
  (void)index;
  if (rcp != DEVICE_RECIPIENT)
    return USB_UNSUPPORT;

  if (desc_type == DEVICE_DESCRIPTOR)
    {
      usb_lld_set_data_to_send (vcom_device_desc, sizeof (vcom_device_desc));
      return USB_SUCCESS;
    }
  else if (desc_type == CONFIG_DESCRIPTOR)
    {
      usb_lld_set_data_to_send (vcom_configuration_desc,
				sizeof (vcom_configuration_desc));
      return USB_SUCCESS;
    }
  else if (desc_type == STRING_DESCRIPTOR)
    {
      const uint8_t *str;
      int size;

      switch (desc_index)
	{
	case 0:
	  str = vcom_string0;
	  size = sizeof (vcom_string0);
	  break;
	case 1:
	  str = neug_string_vendor;
	  size = sizeof (neug_string_vendor);
	  break;
	case 2:
	  str = neug_string_product;
	  size = sizeof (neug_string_product);
	  break;
	case 3:
	  str = vcom_string3;
	  size = sizeof (vcom_string3);
	  break;
	case 4:
	  str = neug_revision_detail;
	  size = sizeof (neug_revision_detail);
	  break;
	case 5:
	  str = neug_config_options;
	  size = sizeof (neug_config_options);
	  break;
	case 6:
	  str = sys_version;
	  size = sizeof (sys_version);
	  break;
	default:
	  return USB_UNSUPPORT;
	}

      usb_lld_set_data_to_send (str, size);
      return USB_SUCCESS;
    }

  return USB_UNSUPPORT;
}

static void
neug_setup_endpoints_for_interface (uint16_t interface, int stop)
{
  if (interface == 0)
    {
      if (!stop)
	usb_lld_setup_endpoint (ENDP2, EP_INTERRUPT, 0, 0, ENDP2_TXADDR, 0);
      else
	usb_lld_stall_tx (ENDP2);
    }
  else if (interface == 1)
    {
      if (!stop)
	{
	  usb_lld_setup_endpoint (ENDP1, EP_BULK, 0, 0, ENDP1_TXADDR, 0);
	  usb_lld_setup_endpoint (ENDP3, EP_BULK, 0, ENDP3_RXADDR, 0, 64);
	}
      else
	{
	  usb_lld_stall_tx (ENDP1);
	  usb_lld_stall_rx (ENDP3);
	}
    }
}

int usb_cb_handle_event (uint8_t event_type, uint16_t value)
{
  int i;
  uint8_t current_conf;

  switch (event_type)
    {
    case USB_EVENT_ADDRESS:
      chopstx_mutex_lock (&usb_mtx);
      bDeviceState = ADDRESSED;
      chopstx_mutex_unlock (&usb_mtx);
      return USB_SUCCESS;
    case USB_EVENT_CONFIG:
      current_conf = usb_lld_current_configuration ();
      if (current_conf == 0)
	{
	  if (value != 1)
	    return USB_UNSUPPORT;

	  usb_lld_set_configuration (1);
	  for (i = 0; i < NUM_INTERFACES; i++)
	    neug_setup_endpoints_for_interface (i, 0);
	  chopstx_mutex_lock (&usb_mtx);
	  bDeviceState = CONFIGURED;
	  chopstx_mutex_unlock (&usb_mtx);
	}
      else if (current_conf != value)
	{
	  if (value != 0)
	    return USB_UNSUPPORT;

	  usb_lld_set_configuration (0);
	  for (i = 0; i < NUM_INTERFACES; i++)
	    neug_setup_endpoints_for_interface (i, 1);
	  chopstx_mutex_lock (&usb_mtx);
	  bDeviceState = ADDRESSED;
	  chopstx_mutex_unlock (&usb_mtx);
	}
      /* Do nothing when current_conf == value */
      return USB_SUCCESS;

      return USB_SUCCESS;
    default:
      break;
    }

  return USB_UNSUPPORT;
}


int usb_cb_interface (uint8_t cmd, uint16_t interface, uint16_t alt)
{
  static uint8_t zero = 0;

  if (interface >= NUM_INTERFACES)
    return USB_UNSUPPORT;

  switch (cmd)
    {
    case USB_SET_INTERFACE:
      if (alt != 0)
	return USB_UNSUPPORT;
      else
	{
	  neug_setup_endpoints_for_interface (interface, 0);
	  return USB_SUCCESS;
	}

    case USB_GET_INTERFACE:
      usb_lld_set_data_to_send (&zero, 1);
      return USB_SUCCESS;

    default:
    case USB_QUERY_INTERFACE:
      return USB_SUCCESS;
    }
}

#define INTR_REQ_USB 20
#define PRIO_USB 3

static void *
usb_intr (void *arg)
{
  chopstx_intr_t interrupt;

  (void)arg;
  usb_lld_init (vcom_configuration_desc[7]);
  chopstx_claim_irq (&interrupt, INTR_REQ_USB);
  usb_interrupt_handler ();

  while (1)
    {
      chopstx_intr_wait (&interrupt);

      /* Process interrupt. */
      usb_interrupt_handler ();
    }

  return NULL;
}


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

void
EP1_IN_Callback (void)
{
  chopstx_mutex_lock (&usb_mtx);
  chopstx_cond_signal (&cnd_usb);
  chopstx_mutex_unlock (&usb_mtx);
}

void
EP2_IN_Callback (void)
{
}

void
EP3_OUT_Callback (void)
{
  usb_lld_rx_enable (ENDP3);
}

typedef uint32_t eventmask_t;
#define ALL_EVENTS (~0)

struct event_flag {
  chopstx_mutex_t mutex;
  chopstx_cond_t cond;
  eventmask_t flag;
};

static void event_flag_init (struct event_flag *ev)
{
  ev->flag = 0;
  chopstx_mutex_init (&ev->mutex);
  chopstx_cond_init (&ev->cond);
}


static eventmask_t event_flag_waitone (struct event_flag *ev, eventmask_t m)
{
  int n;

  chopstx_mutex_lock (&ev->mutex);
  while (!(ev->flag & m))
    chopstx_cond_wait (&ev->cond, &ev->mutex);

  n = __builtin_ffs ((ev->flag & m));
  ev->flag &= ~(1 << (n - 1));
  chopstx_mutex_unlock (&ev->mutex);

  return (1 << (n - 1));
}

static void event_flag_signal (struct event_flag *ev, eventmask_t m)
{
  chopstx_mutex_lock (&ev->mutex);
  ev->flag |= m;
  chopstx_cond_signal (&ev->cond);
  chopstx_mutex_unlock (&ev->mutex);
}

extern uint8_t __process1_stack_base__, __process1_stack_size__;
extern uint8_t __process3_stack_base__, __process3_stack_size__;

const uint32_t __stackaddr_led = (uint32_t)&__process1_stack_base__;
const size_t __stacksize_led = (size_t)&__process1_stack_size__;
const uint32_t __stackaddr_usb = (uint32_t)&__process3_stack_base__;
const size_t __stacksize_usb = (size_t)&__process3_stack_size__;


#define PRIO_LED 3
static struct event_flag led_event;

#define LED_ONESHOT_SHORT	((eventmask_t)1)
#define LED_TWOSHOTS		((eventmask_t)2)
#define LED_ONESHOT_LONG	((eventmask_t)4)

/*
 * LED blinker: When notified, let LED emit for 100ms.
 */
static void *
led_blinker (void *arg)
{
  (void)arg;

  set_led (0);

  while (1)
    {
      eventmask_t m;

      m = event_flag_waitone (&led_event, ALL_EVENTS);
      if (fsij_device_state != FSIJ_DEVICE_RUNNING) /* No usb_mtx lock.  */
	break;

      set_led (1);
      if (m == LED_ONESHOT_SHORT)
	chopstx_usec_wait (100*1000);
      else if (m == LED_TWOSHOTS)
	{
	  chopstx_usec_wait (50*1000);
	  set_led (0);
	  chopstx_usec_wait (50*1000);
	  set_led (1);
	  chopstx_usec_wait (50*1000);
	}
      else
	chopstx_usec_wait (250*1000);
      set_led (0);
    }

  return NULL;
}

#define RANDOM_BYTES_LENGTH 64
static uint32_t random_word[RANDOM_BYTES_LENGTH/sizeof (uint32_t)];

static void copy_to_tx (uint32_t v, int i)
{
  usb_lld_txcpy (&v, ENDP1, i * 4, 4);
}

/*
 * In Gnuk 1.0.[12], reGNUal was not relocatable.
 * Now, it's relocatable, but we need to calculate its entry address
 * based on it's pre-defined address.
 */
#define REGNUAL_START_ADDRESS_COMPATIBLE 0x20001400
static uint32_t
calculate_regnual_entry_address (const uint8_t *addr)
{
  const uint8_t *p = addr + 4;
  uint32_t v = p[0] + (p[1] << 8) + (p[2] << 16) + (p[3] << 24);

  v -= REGNUAL_START_ADDRESS_COMPATIBLE;
  v += (uint32_t)addr;
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
  uint32_t entry;
  chopstx_t led_thread, usb_thd;
  unsigned int count;

  (void)argc;
  (void)argv;

  fill_serial_no_by_unique_id ();

  adc_init ();

  event_flag_init (&led_event);
  
  led_thread = chopstx_create (PRIO_LED, __stackaddr_led, __stacksize_led,
			       led_blinker, NULL);

  chopstx_mutex_init (&usb_mtx);
  chopstx_cond_init (&cnd_usb);

  usb_thd = chopstx_create (PRIO_USB, __stackaddr_usb, __stacksize_usb,
			    usb_intr, NULL);

  neug_init (random_word, RANDOM_BYTES_LENGTH/sizeof (uint32_t));

  chopstx_mutex_lock (&usb_mtx);

 not_configured:
  count = 0;
  /* A run-up */
  while (count < NEUG_PRE_LOOP || bDeviceState != CONFIGURED)
    {
      if (fsij_device_state != FSIJ_DEVICE_RUNNING)
	break;

      chopstx_mutex_unlock (&usb_mtx);
      neug_wait_full ();
      neug_flush ();

      if ((count & 0x0007) == 0)
	event_flag_signal (&led_event, LED_ONESHOT_SHORT);
      chopstx_usec_wait (25*1000);
      count++;
      chopstx_mutex_lock (&usb_mtx);
    }

  /* Holding USB_MTX.  */
  while (1)
    {
      int last_was_fullsizepacket = 0;

      if (fsij_device_state != FSIJ_DEVICE_RUNNING)
	break;

      while (1)
	{
	  wait_usb_connection = 5000*1000;
	  if (connected || bDeviceState != CONFIGURED
	      || fsij_device_state != FSIJ_DEVICE_RUNNING)
	    break;

	  chopstx_mutex_unlock (&usb_mtx);
	  neug_flush ();
	  neug_mode_select (line_coding.paritytype);
	  event_flag_signal (&led_event, LED_TWOSHOTS);
	  chopstx_usec_wait_var (&wait_usb_connection);
	  chopstx_mutex_lock (&usb_mtx);
	}

      if (bDeviceState != CONFIGURED)
	goto not_configured;

      /* The connection opened.  */
      count = 0;

      while (1)
	{
	  int i;

	  chopstx_mutex_unlock (&usb_mtx);
	  /*
	   * No parity is standard.  It means providing conditioned output.
	   * When parity enabled, it means to provide raw output
	   * (CRC32 filtered when odd, direct sample of ADC when even).
	   *
	   * line_coding.paritytype:
	   *   0: None, 1: Odd, 2: Even
	   */
	  neug_mode_select (line_coding.paritytype);

	  if ((count & 0x03ff) == 0)
	    event_flag_signal (&led_event, LED_ONESHOT_SHORT);

	  i = neug_consume_random (copy_to_tx);

	  if (i == 0 && !last_was_fullsizepacket)
	    {	 /* Only send ZLP when the last packet was fullsize.  */
	      neug_wait_full ();

	      chopstx_mutex_lock (&usb_mtx);
	      if (bDeviceState != CONFIGURED || !connected
		  || fsij_device_state != FSIJ_DEVICE_RUNNING)
		break;
	    }
	  else
	    {
	      if (i == 64/4)
		last_was_fullsizepacket = 1;
	      else
		last_was_fullsizepacket = 0;

	      chopstx_mutex_lock (&usb_mtx);
	      if (bDeviceState != CONFIGURED || !connected
		  || fsij_device_state != FSIJ_DEVICE_RUNNING)
		break;

	      /* Prepare sending random data.  */
	      usb_lld_tx_enable (ENDP1, i * 4);
	      chopstx_cond_wait (&cnd_usb, &usb_mtx);
	      count++;
	    }
	}
    }

  chopstx_mutex_unlock (&usb_mtx);

  chopstx_cancel (led_thread);
  chopstx_join (led_thread, NULL);

  /*
   * We come here, because of FSIJ_DEVICE_NEUG_EXIT_REQUESTED.
   */
  neug_fini ();

  chopstx_mutex_lock (&usb_mtx);
  fsij_device_state = FSIJ_DEVICE_EXITED;

  while (fsij_device_state == FSIJ_DEVICE_EXITED)
    {
      chopstx_mutex_unlock (&usb_mtx);
      chopstx_usec_wait (500*1000);
      chopstx_mutex_lock (&usb_mtx);
    }
  chopstx_mutex_unlock (&usb_mtx);

  flash_unlock ();		/* Flash unlock should be done here */
  set_led (1);
  usb_lld_shutdown ();

  /* Finish application.  */
  chopstx_cancel (usb_thd);
  chopstx_join (usb_thd, NULL);

  /* Set vector */
  SCB->VTOR = (uint32_t)&_regnual_start;
  entry = calculate_regnual_entry_address (&_regnual_start);
#ifdef DFU_SUPPORT
#define FLASH_SYS_START_ADDR 0x08000000
#define FLASH_SYS_END_ADDR (0x08000000+0x1000)
  {
    extern uint8_t _sys;
    uint32_t addr;
    handler *new_vector = (handler *)FLASH_SYS_START_ADDR;
    void (*func) (void (*)(void)) = (void (*)(void (*)(void)))new_vector[9];

    /* Kill DFU */
    for (addr = FLASH_SYS_START_ADDR; addr < FLASH_SYS_END_ADDR;
	 addr += FLASH_PAGE_SIZE)
      flash_erase_page (addr);

    /* copy system service routines */
    flash_write (FLASH_SYS_START_ADDR, &_sys, 0x1000);

    /* Leave NeuG to exec reGNUal */
    (*func) ((void (*)(void))entry);
    for (;;);
  }
#else
  /* Leave NeuG to exec reGNUal */
  flash_erase_all_and_exec ((void (*)(void))entry);
#endif

  /* Never reached */
  return 0;
}
