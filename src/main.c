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

static Thread *main_thread = NULL;


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
  {neug_string_vendor, sizeof (neug_string_vendor)},
  {neug_string_product, sizeof (neug_string_product)},
  {vcom_string3, sizeof (vcom_string3)},
  {neug_revision_detail, sizeof (neug_revision_detail)},
  {neug_config_options, sizeof (neug_config_options)},
  {sys_version, sizeof (sys_version)},
};

#define NUM_STRING_DESC (sizeof (string_descs)/sizeof (struct Descriptor))

#define NUM_INTERFACES 2

uint32_t bDeviceState = UNCONNECTED; /* USB device status */

#define NEUG_WAIT_FOR_TX_READY 1
static uint8_t neug_state;

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

extern uint8_t _regnual_start, __heap_end__;

static const uint8_t *const mem_info[] = { &_regnual_start,  &__heap_end__, };

/* USB vendor requests to control pipe */
#define USB_FSIJ_MEMINFO	  0
#define USB_FSIJ_DOWNLOAD	  1
#define USB_FSIJ_EXEC		  2
#define USB_NEUG_EXIT		255 /* Ask to exit and to receive reGNUal */

enum {
  FSIJ_DEVICE_RUNNING = 0,
  FSIJ_DEVICE_EXITED,
  FSIJ_DEVICE_EXEC_REQUESTED,
  /**/
  FSIJ_DEVICE_NEUG_EXIT_REQUESTED = 255
}; 

static uint8_t fsij_device_state = FSIJ_DEVICE_RUNNING;

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

static void neug_ctrl_write_finish (uint8_t req, uint8_t req_no,
				    uint16_t value, uint16_t index,
				    uint16_t len)
{
  uint8_t type_rcp = req & (REQUEST_TYPE|RECIPIENT);

  if (type_rcp == (VENDOR_REQUEST | DEVICE_RECIPIENT)
      && USB_SETUP_SET (req) && len == 0)
    {
      if (req_no == USB_FSIJ_EXEC)
	{
	  if (fsij_device_state != FSIJ_DEVICE_EXITED)
	    return;

	  (void)value; (void)index;
	  usb_lld_prepare_shutdown (); /* No further USB communication */
	  fsij_device_state = FSIJ_DEVICE_EXEC_REQUESTED;
	}
      else if (req_no == USB_NEUG_EXIT)
	{
	  if (neug_state == NEUG_WAIT_FOR_TX_READY)
	    {
	      chSysLockFromIsr ();
	      main_thread->p_u.rdymsg = RDY_OK;
	      chSchReadyI (main_thread);
	      chSysUnlockFromIsr ();
	    }
	  else
	    chEvtSignalFlagsI (main_thread, 1);
	}
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
	      if (connected == 0)
		/* It's Open call */
		connected++;
	    }
	  else
	    {
	      if (connected)
		{
		  /* Close call */
		  connected = 0;
		  if (neug_state == NEUG_WAIT_FOR_TX_READY)
		    {
		      chSysLockFromIsr ();
		      main_thread->p_u.rdymsg = RDY_OK;
		      chSchReadyI (main_thread);
		      chSysUnlockFromIsr ();
		    }
		}
	    }

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
	  if (req_no == USB_FSIJ_MEMINFO)
	    {
	      usb_lld_set_data_to_send (mem_info, sizeof (mem_info));
	      return USB_SUCCESS;
	    }
	}
      else /* SETUP_SET */
	{
	  uint8_t *addr = (uint8_t *)(0x20000000 + value * 0x100 + index);

	  if (req_no == USB_FSIJ_DOWNLOAD)
	    {
	      if (fsij_device_state != FSIJ_DEVICE_EXITED)
		return USB_UNSUPPORT;

	      if (addr < &_regnual_start || addr + len > &__heap_end__)
		return USB_UNSUPPORT;

	      if (index + len < 256)
		memset (addr + index + len, 0, 256 - (index + len));

	      usb_lld_set_data_to_recv (addr, len);
	      return USB_SUCCESS;
	    }
	  else if (req_no == USB_FSIJ_EXEC && len == 0)
	    {
	      if (fsij_device_state != FSIJ_DEVICE_EXITED)
		return USB_UNSUPPORT;

	      if (((uint32_t)addr & 0x03))
		return USB_UNSUPPORT;

	      return download_check_crc32 ((uint32_t *)addr);
	    }
	  else if (req_no == USB_NEUG_EXIT && len == 0)
	    {
	      if (fsij_device_state != FSIJ_DEVICE_RUNNING)
		return USB_UNSUPPORT;

	      fsij_device_state = FSIJ_DEVICE_NEUG_EXIT_REQUESTED;
	      return USB_SUCCESS;
	    }
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

#define LED_ONESHOT_SHORT	((eventmask_t)1)
#define LED_TWOSHOTS		((eventmask_t)2)
#define LED_ONESHOT_LONG	((eventmask_t)4)
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
      if (fsij_device_state != FSIJ_DEVICE_RUNNING)
	break;

      set_led (1);
      if (m == LED_ONESHOT_SHORT)
	chThdSleep (MS2ST (100));
      else if (m == LED_TWOSHOTS)
	{
	  chThdSleep (MS2ST (50));
	  set_led (0);
	  chThdSleep (MS2ST (50));
	  set_led (1);
	  chThdSleep (MS2ST (50));
	}
      else
	chThdSleep (MS2ST (250));
      set_led (0);
    }

  return 0;
}

#define RANDOM_BYTES_LENGTH 32
static uint32_t random_word[RANDOM_BYTES_LENGTH/sizeof (uint32_t)];

extern void adc2_init (void);

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

  halInit ();
  adc2_init ();
  chSysInit ();

  main_thread = chThdSelf ();

  chThdCreateStatic (wa_led, sizeof (wa_led), NORMALPRIO, led_blinker, NULL);

  neug_init (random_word, RANDOM_BYTES_LENGTH/sizeof (uint32_t));

  usb_lld_init (config_desc.Descriptor[7]);

  while (1)
    {
      unsigned int count = 0;

      if (fsij_device_state != FSIJ_DEVICE_RUNNING)
	break;

      while (count < NEUG_PRE_LOOP || bDeviceState != CONFIGURED)
	{
	  if (fsij_device_state != FSIJ_DEVICE_RUNNING)
	    break;

	  neug_wait_full ();
	  neug_flush ();

	  if ((count & 0x0007) == 0)
	    chEvtSignalFlags (led_thread, LED_ONESHOT_SHORT);
	  chEvtWaitOneTimeout (ALL_EVENTS, MS2ST (25));
	  count++;
	}

    waiting_connection:
      while (connected == 0)
	{
	  if (fsij_device_state != FSIJ_DEVICE_RUNNING)
	    break;

	  neug_flush ();
	  chEvtSignalFlags (led_thread, LED_TWOSHOTS);
	  chEvtWaitOneTimeout (ALL_EVENTS, MS2ST (5000));
	}

      if (fsij_device_state != FSIJ_DEVICE_RUNNING)
	break;

      /* The connection opened.  */
      count = 0;
      /*
       * No parity is standard.  It means to provide conditioned output.
       * When parity enabled, it means to provide raw output.
       */
      neug_select (line_coding.paritytype != 0);

      while (1)
	{
	  if (fsij_device_state != FSIJ_DEVICE_RUNNING)
	    break;

	  if (bDeviceState != CONFIGURED)
	    break;

	  neug_wait_full ();

	  if ((count & 0x03ff) == 0)
	    chEvtSignalFlags (led_thread, LED_ONESHOT_SHORT);

	  usb_lld_txcpy (random_word, ENDP1, 0, RANDOM_BYTES_LENGTH);
	  neug_flush ();

	  chSysLock ();
	  if (connected == 0)
	    {
	      chSysUnlock();
	      goto waiting_connection;
	    }
	  else
	    {
	      neug_state = NEUG_WAIT_FOR_TX_READY;
	      usb_lld_tx_enable (ENDP1, RANDOM_BYTES_LENGTH);
	      chSchGoSleepS (THD_STATE_SUSPENDED);
	      neug_state = 0;
	    }
	  chSysUnlock();

	  count++;
	}
    }

  chEvtSignalFlags (led_thread, LED_ONESHOT_SHORT);
  chThdWait (led_thread);

  /*
   * We come here, because of FSIJ_DEVICE_NEUG_EXIT_REQUESTED.
   */
  neug_fini ();

  fsij_device_state = FSIJ_DEVICE_EXITED;

  while (fsij_device_state == FSIJ_DEVICE_EXITED)
    chThdSleep (MS2ST (500));

  flash_unlock ();		/* Flash unlock should be done here */
  set_led (1);
  usb_lld_shutdown ();
  /* Disable SysTick */
  SysTick->CTRL = 0;
  /* Disable all interrupts */
  port_disable ();
  /* Set vector */
  SCB->VTOR = (uint32_t)&_regnual_start;
#ifdef DFU_SUPPORT
#define FLASH_SYS_START_ADDR 0x08000000
#define FLASH_SYS_END_ADDR (0x08000000+0x1000)
  {
    extern uint8_t _sys;
    uint32_t addr;
    handler *new_vector = (handler *)FLASH_SYS_START_ADDR;
    void (*func) (void (*)(void)) = (void (*)(void (*)(void)))new_vector[10];

    /* Kill DFU */
    for (addr = FLASH_SYS_START_ADDR; addr < FLASH_SYS_END_ADDR;
	 addr += FLASH_PAGE_SIZE)
      flash_erase_page (addr);

    /* copy system service routines */
    flash_write (FLASH_SYS_START_ADDR, &_sys, 0x1000);

    /* Leave NeuG to exec reGNUal */
    (*func) (*((void (**)(void))(&_regnual_start+4)));
    for (;;);
  }
#else
  /* Leave NeuG to exec reGNUal */
  flash_erase_all_and_exec (*((void (**)(void))(&_regnual_start+4)));
#endif

  /* Never reached */
  return 0;
}
