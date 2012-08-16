/*
 * main.c - main routine of neug
 *
 * Main routine:
 * Copyright (C) 2011, 2012 Free Software Initiative of Japan
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
#include "neug.h"
#include "usb_lld.h"
#include "sys.h"

/*
 * We are trying to avoid dependency to C library. 
 * GCC built-in function(s) are declared here.
 */
extern void *memcpy(void *dest, const void *src, size_t n);
extern void *memset (void *s, int c, size_t n);


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
  0x4b, 0x23,			/* idVendor (FSIJ).                 */
  0x01, 0x00,			/* idProduct (NeoG).                */
  0x00, 0x01,			/* bcdDevice.                       */
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

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[68] = {
  68,                    /* bLength.                         */
  USB_STRING_DESCRIPTOR_TYPE, /* bDescriptorType.                 */
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
static const uint8_t vcom_string2[18] = {
  18,                    /* bLength.                         */
  USB_STRING_DESCRIPTOR_TYPE, /* bDescriptorType.                 */
  /* Product name: "NeuG RNG" */
  'N', 0, 'e', 0, 'u', 0, 'G', 0, ' ', 0, 'R', 0, 'N', 0, 'G', 0,
};

/*
 * Serial Number string.  NOTE: This does not have CONST qualifier.
 */
static uint8_t vcom_string3[28] = {
  28,                    /* bLength.                         */
  USB_STRING_DESCRIPTOR_TYPE, /* bDescriptorType.                 */
  '0', 0,  '.', 0,  '0', 0, '2', 0,    /* Version number of NeuG.          */
  '-', 0,
  0, 0, 0, 0,	/* Filled by Unique device ID.      */
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
};

static const struct Descriptor device_desc = {
  vcom_device_desc,
  sizeof (vcom_device_desc)
};

static const struct Descriptor config_desc = {
  vcom_configuration_desc,
  sizeof (vcom_configuration_desc)
};

static const struct Descriptor string_descs[] = {
  {vcom_string0, sizeof vcom_string0},
  {vcom_string1, sizeof vcom_string1},
  {vcom_string2, sizeof vcom_string2},
  {vcom_string3, sizeof vcom_string3}
};

#define NUM_STRING_DESC (sizeof (string_descs)/sizeof (struct Descriptor))

#define NUM_INTERFACES 2

uint32_t bDeviceState = UNCONNECTED; /* USB device status */


static void
neug_device_reset (void)
{
  /* Set DEVICE as not configured */
  usb_lld_set_configuration (0);

  /* Current Feature initialization */
  usb_lld_set_feature (config_desc.Descriptor[7]);

  usb_lld_reset ();

  /* Initialize Endpoint 0 */
  usb_lld_setup_endpoint (ENDP0, EP_CONTROL, 0, ENDP0_RXADDR, ENDP0_TXADDR, 64);
}

#define USB_REGNUAL_MEMINFO	0
#define USB_REGNUAL_SEND	1
#define USB_REGNUAL_RESULT	2
#define USB_REGNUAL_FLASH	3
#define USB_REGNUAL_PROTECT	4
#define USB_REGNUAL_FINISH	5

static uint8_t mem[256];
static uint32_t result;

extern uint8_t __flash_start__,  __flash_end__;
static const uint8_t *const mem_info[] = { &__flash_start__,  &__flash_end__ };


static uint32_t rbit (uint32_t v)
{
  uint32_t r;

  asm ("rbit	%0, %1" : "=r" (r) : "r" (v));
  return r;
}

static uint32_t fetch (int i)
{
  uint32_t v;

  v = *(uint32_t *)(&mem[i*4]);
  return rbit (v);
}

static uint32_t calc_crc32 (void)
{
  int i;

  CRC->CR = CRC_CR_RESET;

  for (i = 0; i < 256/4; i++)
    CRC->DR = fetch (i);

  return rbit (CRC->DR);
}


static void neug_ctrl_write_finish (uint8_t req, uint8_t req_no,
				    uint16_t value, uint16_t index,
				    uint16_t len)
{
  uint8_t type_rcp = req & (REQUEST_TYPE|RECIPIENT);

  if (type_rcp == (VENDOR_REQUEST | DEVICE_RECIPIENT) && USB_SETUP_SET (req))
    {
      if (req_no == USB_REGNUAL_SEND && value == 0)
	result = calc_crc32 ();
      else if (req_no == USB_REGNUAL_FLASH && len == 0 && index == 0)
	{
	  uint32_t dst_addr = (0x08000000 + value * 0x100);

	  result = flash_write (dst_addr, mem, 256);
	}
      else if (req_no == USB_REGNUAL_PROTECT && len == 0
	       && value == 0 && index == 0)
	result = flash_protect ();
      else if (req_no == USB_REGNUAL_FINISH && len == 0
	       && value == 0 && index == 0)
	nvic_system_reset ();
    }
}

struct line_coding
{
  uint32_t bitrate;
  uint8_t format;
  uint8_t paritytype;
  uint8_t datatype;
};

static struct line_coding line_coding = {
  115200, /* baud rate: 115200    */
  0x00,   /* stop bits: 1         */
  0x00,   /* parity:    none      */
  0x08    /* bits:      8         */
};

static uint8_t connected;

static int
vcom_port_data_setup (uint8_t req, uint8_t req_no, uint16_t value)
{
  if (USB_SETUP_GET (req))
    {
      if (req_no == USB_CDC_REQ_GET_LINE_CODING)
	{
	  usb_lld_set_data_to_send (&line_coding, sizeof(line_coding));
	  return USB_SUCCESS;
	}
    }
  else  /* USB_SETUP_SET (req) */
    {
      if (req_no == USB_CDC_REQ_SET_LINE_CODING)
	{
	  usb_lld_set_data_to_recv (&line_coding, sizeof(line_coding));
	  return USB_SUCCESS;
	}
      else if (req_no == USB_CDC_REQ_SET_CONTROL_LINE_STATE)
	{
	  if (value != 0)
	    {
	      if ((connected & 1) == 0)
		/* It's Open call */
		connected++;
	    }
	  else
	    /* Close call */
	    connected++;

	  return USB_SUCCESS;
	}
    }

  return USB_UNSUPPORT;
}

static int
neug_setup (uint8_t req, uint8_t req_no,
	       uint16_t value, uint16_t index, uint16_t len)
{
  uint8_t type_rcp = req & (REQUEST_TYPE|RECIPIENT);

  if (type_rcp == (VENDOR_REQUEST | DEVICE_RECIPIENT))
    {
      if (USB_SETUP_GET (req))
	{
	  if (req_no == USB_REGNUAL_MEMINFO)
	    {
	      usb_lld_set_data_to_send (mem_info, sizeof (mem_info));
	      return USB_SUCCESS;
	    }
	  else if (req_no == USB_REGNUAL_RESULT)
	    {
	      usb_lld_set_data_to_send (&result, sizeof (uint32_t));
	      return USB_SUCCESS;
	    }
	}
      else /* SETUP_SET */
	{
	  if (req_no == USB_REGNUAL_SEND)
	    {
	      if (value != 0 || index + len > 256)
		return USB_UNSUPPORT;

	      if (index + len < 256)
		memset (mem + index + len, 0xff, 256 - (index + len));

	      usb_lld_set_data_to_recv (mem + index, len);
	      return USB_SUCCESS;
	    }
	  else if (req_no == USB_REGNUAL_FLASH && len == 0 && index == 0)
	    {
	      uint32_t dst_addr = (0x08000000 + value * 0x100);

	      if (dst_addr + 256 <= (uint32_t)&__flash_end__)
		return USB_SUCCESS;
	    }
	  else if (req_no == USB_REGNUAL_PROTECT && len == 0
		   && value == 0 && index == 0)
	    return USB_SUCCESS;
	  else if (req_no == USB_REGNUAL_FINISH && len == 0
		   && value == 0 && index == 0)
	    return USB_SUCCESS;
	}
    }
  else if (type_rcp == (CLASS_REQUEST | INTERFACE_RECIPIENT))
    if (index == 0)
      return vcom_port_data_setup (req, req_no, value);

  return USB_UNSUPPORT;
}

static int
neug_get_descriptor (uint8_t desc_type, uint16_t index, uint16_t value)
{
  (void)index;
  if (desc_type == DEVICE_DESCRIPTOR)
    {
      usb_lld_set_data_to_send (device_desc.Descriptor,
				device_desc.Descriptor_Size);
      return USB_SUCCESS;
    }
  else if (desc_type == CONFIG_DESCRIPTOR)
    {
      usb_lld_set_data_to_send (config_desc.Descriptor,
				config_desc.Descriptor_Size);
      return USB_SUCCESS;
    }
  else if (desc_type == STRING_DESCRIPTOR)
    {
      uint8_t desc_index = value & 0xff;

      if (desc_index < NUM_STRING_DESC)
	{
	  usb_lld_set_data_to_send (string_descs[desc_index].Descriptor,
				    string_descs[desc_index].Descriptor_Size);
	  return USB_SUCCESS;
	}
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

static int neug_usb_event (uint8_t event_type, uint16_t value)
{
  int i;
  uint8_t current_conf;

  switch (event_type)
    {
    case USB_EVENT_ADDRESS:
      bDeviceState = ADDRESSED;
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
	  bDeviceState = CONFIGURED;
	}
      else if (current_conf != value)
	{
	  if (value != 0)
	    return USB_UNSUPPORT;

	  usb_lld_set_configuration (0);
	  for (i = 0; i < NUM_INTERFACES; i++)
	    neug_setup_endpoints_for_interface (i, 1);
	  bDeviceState = ADDRESSED;
	}
      /* Do nothing when current_conf == value */
      return USB_SUCCESS;

      return USB_SUCCESS;
    default:
      break;
    }

  return USB_UNSUPPORT;
}


static int neug_interface (uint8_t cmd, uint16_t interface, uint16_t alt)
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

const struct usb_device_method Device_Method = {
  neug_device_reset,
  neug_ctrl_write_finish,
  neug_setup,
  neug_get_descriptor,
  neug_usb_event,
  neug_interface,
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

static Thread *main_thread = NULL;

CH_IRQ_HANDLER (Vector90)
{
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();

  usb_interrupt_handler ();

  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}

void
EP1_IN_Callback (void)
{
  if (main_thread != NULL)
    {
      chSysLockFromIsr ();
      main_thread->p_u.rdymsg = RDY_OK;
      chSchReadyI (main_thread);
      chSysUnlockFromIsr ();
    }
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

static WORKING_AREA(wa_led, 64);

#define LED_ONESHOT_SHORT ((eventmask_t)1)
#define LED_ONESHOT_LONG  ((eventmask_t)2)
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
      eventmask_t m;

      m = chEvtWaitOne (ALL_EVENTS);
      set_led (1);
      if (m == LED_ONESHOT_SHORT)
	chThdSleep (MS2ST (100));
      else
	chThdSleep (MS2ST (250));
      set_led (0);
    }

  return 0;
}

#define RANDOM_BYTES_LENGTH 60
static uint32_t random_word[RANDOM_BYTES_LENGTH/sizeof (uint32_t)];

/*
 * Entry point.
 *
 * NOTE: the main function is already a thread in the system on entry.
 */
int
main (int argc, char **argv)
{
  (void)argc;
  (void)argv;

  fill_serial_no_by_unique_id ();

  halInit();
  chSysInit();

  main_thread = chThdSelf ();

  usb_lld_init (config_desc.Descriptor[7]);

  chThdCreateStatic (wa_led, sizeof (wa_led), NORMALPRIO, led_blinker, NULL);

  neug_init (random_word, RANDOM_BYTES_LENGTH/sizeof (uint32_t));

  while (1)
    {
      unsigned int count = 0;

      connected = 0;
      while (count < NEUG_PRE_LOOP || bDeviceState != CONFIGURED)
	{
	  (void)neug_get (NEUG_KICK_FILLING);
	  if ((count & 0x000f) == 0)
	    chEvtSignalFlags (led_thread, LED_ONESHOT_SHORT);
	  chThdSleep (MS2ST (25));
	  count++;
	}

    waiting_connection:
      while ((connected & 1) == 0)
	{
	  neug_flush ();
	  chEvtSignalFlags (led_thread, LED_ONESHOT_LONG);
	  chThdSleep (MS2ST (2500));
	}

      /* The connection opened.  */
      count = 0;

      while (1)
	{
	  if ((connected & 1) == 0)
	    goto waiting_connection;

	  if (bDeviceState != CONFIGURED)
	    break;

	  neug_wait_full ();

	  if ((count & 0x00ff) == 0)
	    chEvtSignalFlags (led_thread, LED_ONESHOT_SHORT);

	  usb_lld_txcpy (random_word, ENDP1, 0, RANDOM_BYTES_LENGTH);
	  neug_flush ();

	  chSysLock ();
	  usb_lld_tx_enable (ENDP1, RANDOM_BYTES_LENGTH);
	  chSchGoSleepS (THD_STATE_SUSPENDED);
	  chSysUnlock();

	  count++;
	}
    }

  return 0;
}
