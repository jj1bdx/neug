/*
 * main.c - main routine of neug
 *
 * Main routine:
 * Copyright (C) 2011, 2012, 2013, 2015, 2016, 2017
 *		 Free Software Initiative of Japan
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
#ifndef GNU_LINUX_EMULATION
#include "mcu/stm32f103.h"
#endif
#include "adc.h"

enum {
  FSIJ_DEVICE_RUNNING = 0,
  FSIJ_DEVICE_EXITED,
  FSIJ_DEVICE_EXEC_REQUESTED,
  /**/
  FSIJ_DEVICE_NEUG_FRAUCHEKY_REQUESTED = 254,
  FSIJ_DEVICE_NEUG_EXIT_REQUESTED = 255
};

#ifdef FRAUCHEKY_SUPPORT
static uint8_t running_neug;
extern void EP6_IN_Callback (uint16_t len);
extern void EP6_OUT_Callback (uint16_t len);
#endif

static chopstx_mutex_t usb_mtx;
static chopstx_cond_t usb_cnd;
static uint32_t bDeviceState = UNCONNECTED; /* USB device status */
static uint8_t fsij_device_state = FSIJ_DEVICE_RUNNING;
static uint8_t connected;


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
  DEVICE_DESCRIPTOR,	        /* bDescriptorType */
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

#define VCOM_FEATURE_BUS_POWERED	0x80

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_config_desc[67] = {
  9,
  CONFIG_DESCRIPTOR,            /* bDescriptorType: Configuration   */
  /* Configuration Descriptor.*/
  67, 0x00,			/* wTotalLength.                    */
  0x02,				/* bNumInterfaces.                  */
  0x01,				/* bConfigurationValue.             */
  0,				/* iConfiguration.                  */
  VCOM_FEATURE_BUS_POWERED,	/* bmAttributes.                    */
  50,				/* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  9,
  INTERFACE_DESCRIPTOR,
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
  ENDPOINT_DESCRIPTOR,
  ENDP2|0x80,    /* bEndpointAddress.    */
  0x03,          /* bmAttributes (Interrupt).        */
  0x08, 0x00,	 /* wMaxPacketSize.                  */
  0xFF,		 /* bInterval.                       */
  /* Interface Descriptor.*/
  9,
  INTERFACE_DESCRIPTOR,          /* bDescriptorType: */
  0x01,          /* bInterfaceNumber.                */
  0x00,          /* bAlternateSetting.               */
  0x02,          /* bNumEndpoints.                   */
  0x0A,          /* bInterfaceClass (Data Class Interface, CDC section 4.5). */
  0x00,          /* bInterfaceSubClass (CDC section 4.6). */
  0x00,          /* bInterfaceProtocol (CDC section 4.7). */
  0x00,		 /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  7,
  ENDPOINT_DESCRIPTOR,	        /* bDescriptorType: Endpoint */
  ENDP3,    /* bEndpointAddress. */
  0x02,				/* bmAttributes (Bulk).             */
  0x40, 0x00,			/* wMaxPacketSize.                  */
  0x00,				/* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  7,
  ENDPOINT_DESCRIPTOR,	        /* bDescriptorType: Endpoint */
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
  STRING_DESCRIPTOR,
  0x09, 0x04			/* LangID = 0x0409: US-English */
};

#include "usb-strings.c.inc"

static void neug_setup_endpoints_for_interface (struct usb_dev *dev,
						uint16_t interface, int stop);
#ifdef FRAUCHEKY_SUPPORT
#define MSC_MASS_STORAGE_RESET_COMMAND 0xFF
extern int fraucheky_enabled (void);
extern void fraucheky_main (void);

extern void fraucheky_setup_endpoints_for_interface (int stop);
extern int fraucheky_setup (struct usb_dev *dev);
extern int fraucheky_get_descriptor (struct usb_dev *dev);
#endif

#define NUM_INTERFACES 2

static void
usb_device_reset (struct usb_dev *dev)
{
  int i;

  usb_lld_reset (dev, VCOM_FEATURE_BUS_POWERED);

  /* Initialize Endpoint 0.  */
#ifdef GNU_LINUX_EMULATION
  usb_lld_setup_endp (dev, ENDP0, 1, 1);
#else
  usb_lld_setup_endpoint (ENDP0, EP_CONTROL, 0, ENDP0_RXADDR, ENDP0_TXADDR, 64);
#endif

  /* Stop the interface */
  for (i = 0; i < NUM_INTERFACES; i++)
    neug_setup_endpoints_for_interface (dev, i, 1);

  /* Notify upper layer.  */
  chopstx_mutex_lock (&usb_mtx);
  bDeviceState = ATTACHED;
  connected = 0;
  chopstx_cond_signal (&usb_cnd);
  chopstx_mutex_unlock (&usb_mtx);
}

extern uint8_t _regnual_start, __heap_end__;

static const uint8_t *const mem_info[] = { &_regnual_start,  &__heap_end__, };

/* USB vendor requests to control pipe */
#define USB_FSIJ_MEMINFO	  0
#define USB_FSIJ_DOWNLOAD	  1
#define USB_FSIJ_EXEC		  2
#define USB_NEUG_SET_PASSWD	253
#define USB_NEUG_GET_INFO	254
#define USB_NEUG_EXIT		255 /* Ask to exit and to receive reGNUal */

uint8_t neug_passwd[33] __attribute__ ((section(".passwd"))) = {
  0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};
static uint8_t usbbuf[64];

#define DEFAULT_PASSWD "12345678"
#define DEFAULT_PASSWD_LEN 8

static void set_passwd (void)
{
  flash_unlock ();
  if (neug_passwd[0] != 0xff)
    flash_erase_page ((uintptr_t)neug_passwd);
  if (usbbuf[0] == DEFAULT_PASSWD_LEN
      && !memcmp (usbbuf + 1, DEFAULT_PASSWD, DEFAULT_PASSWD_LEN))
    return;
  flash_write ((uintptr_t)neug_passwd, usbbuf, usbbuf[0] + 1);
}


/* After calling this function, CRC module remain enabled.  */
static int download_check_crc32 (struct usb_dev *dev, const uint32_t *end_p)
{
  uint32_t crc32 = *end_p;
  const uint32_t *p;

  crc32_reset ();

  for (p = (const uint32_t *)&_regnual_start; p < end_p; p++)
    crc32_step (rbit (*p));

  if ((rbit (crc32_get ()) ^ crc32) == 0xffffffff)
    return usb_lld_ctrl_ack (dev);

  return -1;
}


#define NEUG_SPECIAL_BITRATE 110

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

#define CDC_CTRL_DTR            0x0001

static void
usb_ctrl_write_finish (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t type_rcp = arg->type & (REQUEST_TYPE|RECIPIENT);

  if (type_rcp == (VENDOR_REQUEST | DEVICE_RECIPIENT)
      && USB_SETUP_SET (arg->type))
    {
      if (arg->request == USB_FSIJ_EXEC)
	{
	  chopstx_mutex_lock (&usb_mtx);
	  if (fsij_device_state == FSIJ_DEVICE_EXITED)
	    {
	      usb_lld_prepare_shutdown (); /* No further USB communication */
	      fsij_device_state = FSIJ_DEVICE_EXEC_REQUESTED;
	    }
	  chopstx_mutex_unlock (&usb_mtx);
	}
      else if (arg->request == USB_NEUG_SET_PASSWD)
	set_passwd ();
      else if (arg->request == USB_NEUG_EXIT)
	{
	  if ((neug_passwd[0] == 0xff && usbbuf[0] == DEFAULT_PASSWD_LEN
	       && !memcmp (usbbuf + 1, DEFAULT_PASSWD, DEFAULT_PASSWD_LEN))
	      || (neug_passwd[0] == usbbuf[0]
		  && !memcmp (neug_passwd+1, usbbuf+1, neug_passwd[0])))
	    {
	      chopstx_mutex_lock (&usb_mtx);
	      fsij_device_state = FSIJ_DEVICE_NEUG_EXIT_REQUESTED;
	      chopstx_cond_signal (&usb_cnd);
	      chopstx_mutex_unlock (&usb_mtx);
	    }
	}
    }
  else if (type_rcp == (CLASS_REQUEST | INTERFACE_RECIPIENT) && arg->index == 0
	   && USB_SETUP_SET (arg->type))
    {
      if (arg->request == USB_CDC_REQ_SET_CONTROL_LINE_STATE)
	{
	  /* Open/close the connection.  */
	  chopstx_mutex_lock (&usb_mtx);
	  connected = (arg->value & CDC_CTRL_DTR)? 1 : 0;
	  chopstx_cond_signal (&usb_cnd);
	  chopstx_mutex_unlock (&usb_mtx);
	}
#ifdef FRAUCHEKY_SUPPORT
      else if (running_neug && arg->request == USB_CDC_REQ_SET_LINE_CODING)
	{
	  chopstx_mutex_lock (&usb_mtx);
	  if (line_coding.bitrate == NEUG_SPECIAL_BITRATE)
	    {
	      fsij_device_state = FSIJ_DEVICE_NEUG_FRAUCHEKY_REQUESTED;
	      chopstx_cond_signal (&usb_cnd);
	    }
	  else if (fsij_device_state == FSIJ_DEVICE_NEUG_FRAUCHEKY_REQUESTED)
	    fsij_device_state = FSIJ_DEVICE_RUNNING;
	  chopstx_mutex_unlock (&usb_mtx);
	}
      else if (!running_neug && arg->request == MSC_MASS_STORAGE_RESET_COMMAND)
	fraucheky_setup_endpoints_for_interface (0);
#endif
    }
}


static int
vcom_port_data_setup (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;

  if (USB_SETUP_GET (arg->type))
    {
      if (arg->request == USB_CDC_REQ_GET_LINE_CODING)
	return usb_lld_ctrl_send (dev, &line_coding, sizeof(line_coding));
    }
  else  /* USB_SETUP_SET (req) */
    {
      if (arg->request == USB_CDC_REQ_SET_LINE_CODING
	  && arg->len == sizeof (line_coding))
	return usb_lld_ctrl_recv (dev, &line_coding, sizeof (line_coding));
      else if (arg->request == USB_CDC_REQ_SET_CONTROL_LINE_STATE)
	return usb_lld_ctrl_ack (dev);
    }

  return -1;
}

static int
usb_setup (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t type_rcp = arg->type & (REQUEST_TYPE|RECIPIENT);

  if (type_rcp == (VENDOR_REQUEST | DEVICE_RECIPIENT))
    {
      if (USB_SETUP_GET (arg->type))
	{
	  if (arg->request == USB_FSIJ_MEMINFO)
	    {
	      chopstx_mutex_lock (&usb_mtx);
	      if (fsij_device_state != FSIJ_DEVICE_EXITED)
		{
		  chopstx_mutex_unlock (&usb_mtx);
		  return -1;
		}
	      chopstx_mutex_unlock (&usb_mtx);
	      return usb_lld_ctrl_send (dev, mem_info, sizeof (mem_info));
	    }
	  else if (arg->request == USB_NEUG_GET_INFO)
	    {
	      if (arg->index == 0)
		return usb_lld_ctrl_send (dev, &neug_mode, sizeof (uint8_t));
	      else if (arg->index == 1)
		return usb_lld_ctrl_send (dev, &neug_err_cnt, sizeof (uint16_t));
	      else if (arg->index == 2)
		return usb_lld_ctrl_send (dev, &neug_err_cnt_rc, sizeof (uint16_t));
	      else if (arg->index == 3)
		return usb_lld_ctrl_send (dev, &neug_err_cnt_p64, sizeof (uint16_t));
	      else if (arg->index == 4)
		return usb_lld_ctrl_send (dev, &neug_err_cnt_p4k, sizeof (uint16_t));
	      else if (arg->index == 5)
		return usb_lld_ctrl_send (dev, &neug_rc_max, sizeof (uint16_t));
	      else if (arg->index == 6)
		return usb_lld_ctrl_send (dev, &neug_p64_max, sizeof (uint16_t));
	      else if (arg->index == 7)
		return usb_lld_ctrl_send (dev, &neug_p4k_max, sizeof (uint16_t));
	      else
		return -1;
	    }
	}
      else /* SETUP_SET */
	{
	  uint8_t *addr = (uint8_t *)(0x20000000UL + arg->value * 0x100 + arg->index);

	  if (arg->request == USB_FSIJ_DOWNLOAD)
	    {
	      chopstx_mutex_lock (&usb_mtx);
	      if (fsij_device_state != FSIJ_DEVICE_EXITED)
		{
		  chopstx_mutex_unlock (&usb_mtx);
		  return -1;
		}
	      chopstx_mutex_unlock (&usb_mtx);

	      if (addr < &_regnual_start || addr + arg->len > &__heap_end__)
		return -1;

	      if (arg->index + arg->len < 256)
		memset (addr + arg->index + arg->len, 0, 256 - (arg->index + arg->len));

	      return usb_lld_ctrl_recv (dev, addr, arg->len);
	    }
	  else if (arg->request == USB_FSIJ_EXEC && arg->len == 0)
	    {
	      chopstx_mutex_lock (&usb_mtx);
	      if (fsij_device_state != FSIJ_DEVICE_EXITED)
		{
		  chopstx_mutex_unlock (&usb_mtx);
		  return -1;
		}
	      chopstx_mutex_unlock (&usb_mtx);

	      if (((uintptr_t)addr & 0x03))
		return -1;

	      return download_check_crc32 (dev, (uint32_t *)addr);
	    }
	  else if (arg->request == USB_NEUG_SET_PASSWD && arg->len <= 32)
	    {
	      usbbuf[0] = arg->len;
	      return usb_lld_ctrl_recv (dev, usbbuf + 1, arg->len);
	    }
	  else if (arg->request == USB_NEUG_EXIT && arg->len <= 32)
	    {
	      chopstx_mutex_lock (&usb_mtx);
	      if (fsij_device_state != FSIJ_DEVICE_RUNNING)
		{
		  chopstx_mutex_unlock (&usb_mtx);
		  return -1;
		}
	      chopstx_mutex_unlock (&usb_mtx);

	      usbbuf[0] = arg->len;
	      return usb_lld_ctrl_recv (dev, usbbuf + 1, arg->len);
	    }
	}
    }
  else if (type_rcp == (CLASS_REQUEST | INTERFACE_RECIPIENT)
	   && arg->index == 0)
    {
#ifdef FRAUCHEKY_SUPPORT
      if (running_neug)
	return vcom_port_data_setup (dev);
      else
	return fraucheky_setup (dev);
#else
      return vcom_port_data_setup (dev);
#endif
    }

  return -1;
}

static int
usb_get_descriptor (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;
  uint8_t desc_type = (arg->value >> 8);
  uint8_t desc_index = (arg->value & 0xff);

#ifdef FRAUCHEKY_SUPPORT
  if (!running_neug)
    return fraucheky_get_descriptor (dev);
#endif

  if (rcp != DEVICE_RECIPIENT)
    return -1;

  if (desc_type == DEVICE_DESCRIPTOR)
    return usb_lld_ctrl_send (dev,
			      vcom_device_desc, sizeof (vcom_device_desc));
  else if (desc_type == CONFIG_DESCRIPTOR)
    return usb_lld_ctrl_send (dev,
			      vcom_config_desc, sizeof (vcom_config_desc));
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
	  str = neug_string_serial;
	  size = sizeof (neug_string_serial);
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
#ifdef USE_SYS3
	case 7:
	  {
	    int i;
	    str = usbbuf;
	    for (i = 0; i < (int)sizeof (usbbuf)/2 - 2; i++)
	      {
		if (sys_board_name[i] == 0)
		  break;

		usbbuf[i*2+2] = sys_board_name[i];
		usbbuf[i*2+3] = 0;
	      }
	    usbbuf[0] = i*2 + 2;
	    usbbuf[1] = STRING_DESCRIPTOR;
	    size = i*2 + 2;
	  }
	  break;
#endif
	default:
	  return -1;
	}

      return usb_lld_ctrl_send (dev, str, size);
    }

  return -1;
}

static void
neug_setup_endpoints_for_interface (struct usb_dev *dev,
				    uint16_t interface, int stop)
{
#if !defined(GNU_LINUX_EMULATION)
  (void)dev;
#endif
  
  if (interface == 0)
    {
#ifdef FRAUCHEKY_SUPPORT
      if (running_neug)
	{
	  if (!stop)
#ifdef GNU_LINUX_EMULATION
	    usb_lld_setup_endp (dev, ENDP2, 0, 1);
#else
	    usb_lld_setup_endpoint (ENDP2, EP_INTERRUPT, 0, 0, ENDP2_TXADDR, 0);
#endif
	  else
	    usb_lld_stall_tx (ENDP2);
	}
      else
	fraucheky_setup_endpoints_for_interface (stop);
#else
      if (!stop)
#ifdef GNU_LINUX_EMULATION
	usb_lld_setup_endp (dev, ENDP2, 0, 1);
#else
	usb_lld_setup_endpoint (ENDP2, EP_INTERRUPT, 0, 0, ENDP2_TXADDR, 0);
#endif
      else
	usb_lld_stall_tx (ENDP2);
#endif
    }
  else if (interface == 1)
    {
      if (!stop)
	{
#ifdef GNU_LINUX_EMULATION
	  usb_lld_setup_endp (dev, ENDP1, 0, 1);
	  usb_lld_setup_endp (dev, ENDP3, 1, 0);
#else
	  usb_lld_setup_endpoint (ENDP1, EP_BULK, 0, 0, ENDP1_TXADDR, 0);
	  usb_lld_setup_endpoint (ENDP3, EP_BULK, 0, ENDP3_RXADDR, 0, 64);
#endif
	  /* Start with no data receiving (ENDP3 not enabled) */
	}
      else
	{
	  usb_lld_stall_tx (ENDP1);
	  usb_lld_stall_rx (ENDP3);
	}
    }
}

static int
usb_set_configuration (struct usb_dev *dev)
{
  int i;
  uint8_t current_conf;

  current_conf = usb_lld_current_configuration (dev);
  if (current_conf == 0)
    {
      if (dev->dev_req.value != 1)
	return -1;

      usb_lld_set_configuration (dev, 1);
      for (i = 0; i < NUM_INTERFACES; i++)
	neug_setup_endpoints_for_interface (dev, i, 0);
      chopstx_mutex_lock (&usb_mtx);
      bDeviceState = CONFIGURED;
      chopstx_mutex_unlock (&usb_mtx);
    }
  else if (current_conf != dev->dev_req.value)
    {
      if (dev->dev_req.value != 0)
	return -1;

      usb_lld_set_configuration (dev, 0);
      for (i = 0; i < NUM_INTERFACES; i++)
	neug_setup_endpoints_for_interface (dev, i, 1);
      chopstx_mutex_lock (&usb_mtx);
      bDeviceState = ADDRESSED;
      chopstx_cond_signal (&usb_cnd);
      chopstx_mutex_unlock (&usb_mtx);
    }

  /* Do nothing when current_conf == value */
  return usb_lld_ctrl_ack (dev);
}


static int
usb_set_interface (struct usb_dev *dev)
{
  uint16_t interface = dev->dev_req.index;
  uint16_t alt = dev->dev_req.value;

  if (interface >= NUM_INTERFACES)
    return -1;

  if (alt != 0)
    return -1;
  else
    {
      neug_setup_endpoints_for_interface (dev, interface, 0);
      return usb_lld_ctrl_ack (dev);
    }
}

static int
usb_get_interface (struct usb_dev *dev)
{
  const uint8_t zero = 0;
  uint16_t interface = dev->dev_req.index;

  if (interface >= NUM_INTERFACES)
    return -1;

  return usb_lld_ctrl_send (dev, &zero, 1);
}


static int
usb_get_status_interface (struct usb_dev *dev)
{
  const uint16_t status_info = 0;
  uint16_t interface = dev->dev_req.index;

  if (interface >= NUM_INTERFACES)
    return -1;

  return usb_lld_ctrl_send (dev, &status_info, 2);
}

static void usb_tx_done (uint8_t ep_num, uint16_t len);
static void usb_rx_ready (uint8_t ep_num, uint16_t len);


#define INTR_REQ_USB 20
#define PRIO_USB 3

static void *
usb_main (void *arg)
{
  chopstx_intr_t interrupt;
  struct usb_dev dev;
  int e;

  (void)arg;
  usb_lld_init (&dev, VCOM_FEATURE_BUS_POWERED);
  chopstx_claim_irq (&interrupt, INTR_REQ_USB);
  goto event_handle;	/* For old SYS < 3.0 */

  while (1)
    {
      chopstx_intr_wait (&interrupt);

      if (interrupt.ready)
	{
	  uint8_t ep_num;

	event_handle:
	  e = usb_lld_event_handler (&dev);
	  ep_num = USB_EVENT_ENDP (e);

	  if (ep_num != 0)
	    {
	      if (USB_EVENT_TXRX (e))
		usb_tx_done (ep_num, USB_EVENT_LEN (e));
	      else
		usb_rx_ready (ep_num, USB_EVENT_LEN (e));
	    }
	  else
	    switch (USB_EVENT_ID (e))
	      {
	      case USB_EVENT_DEVICE_RESET:
		usb_device_reset (&dev);
		continue;

	      case USB_EVENT_DEVICE_ADDRESSED:
		chopstx_mutex_lock (&usb_mtx);
		bDeviceState = ADDRESSED;
		chopstx_cond_signal (&usb_cnd);
		chopstx_mutex_unlock (&usb_mtx);
		continue;

	      case USB_EVENT_GET_DESCRIPTOR:
		if (usb_get_descriptor (&dev) < 0)
		  usb_lld_ctrl_error (&dev);
		continue;

	      case USB_EVENT_SET_CONFIGURATION:
		if (usb_set_configuration (&dev) < 0)
		  usb_lld_ctrl_error (&dev);
		continue;

	      case USB_EVENT_SET_INTERFACE:
		if (usb_set_interface (&dev) < 0)
		  usb_lld_ctrl_error (&dev);
		continue;

	      case USB_EVENT_CTRL_REQUEST:
		/* Device specific device request.  */
		if (usb_setup (&dev) < 0)
		  usb_lld_ctrl_error (&dev);
		continue;

	      case USB_EVENT_GET_STATUS_INTERFACE:
		if (usb_get_status_interface (&dev) < 0)
		  usb_lld_ctrl_error (&dev);
		continue;

	      case USB_EVENT_GET_INTERFACE:
		if (usb_get_interface (&dev) < 0)
		  usb_lld_ctrl_error (&dev);
		continue;

	      case USB_EVENT_SET_FEATURE_DEVICE:
	      case USB_EVENT_SET_FEATURE_ENDPOINT:
	      case USB_EVENT_CLEAR_FEATURE_DEVICE:
	      case USB_EVENT_CLEAR_FEATURE_ENDPOINT:
		usb_lld_ctrl_ack (&dev);
		continue;

	      case USB_EVENT_CTRL_WRITE_FINISH:
		/* Control WRITE transfer finished.  */
		usb_ctrl_write_finish (&dev);
		continue;

	      case USB_EVENT_OK:
	      case USB_EVENT_DEVICE_SUSPEND:
	      default:
		continue;
	      }
	}
    }

  return NULL;
}

#define ID_OFFSET (2+SERIALNO_STR_LEN*2)
static void fill_serial_no_by_unique_id (void)
{
  extern const uint8_t * unique_device_id (void);
  uint8_t *p = &neug_string_serial[ID_OFFSET];
  const uint8_t *u = unique_device_id () + 8;
  int i;

  for (i = 0; i < 4; i++)
    {
      uint8_t b = u[3-i];
      uint8_t nibble;

      nibble = (b >> 4);
      nibble += (nibble >= 10 ? ('A' - 10) : '0');
      p[i*4] = nibble;
      p[i*4+1] = 0;
      nibble = (b & 0x0f);
      nibble += (nibble >= 10 ? ('A' - 10) : '0');
      p[i*4+2] = nibble;
      p[i*4+3] = 0;
    }
}


static void
usb_tx_done (uint8_t ep_num, uint16_t len)
{
  if (ep_num == ENDP1)
    {
      chopstx_mutex_lock (&usb_mtx);
      chopstx_cond_signal (&usb_cnd);
      chopstx_mutex_unlock (&usb_mtx);
    }
  else if (ep_num == ENDP2)
    {
      /* INTERRUPT Transfer done */
    }
#ifdef FRAUCHEKY_SUPPORT
  else if (ep_num == ENDP6)
    EP6_IN_Callback (len);
#else
  (void)len;
#endif
}

#ifdef GNU_LINUX_EMULATION
static uint8_t endp3_buf[64];
#endif

static void
usb_rx_ready (uint8_t ep_num, uint16_t len)
{
  if (ep_num == ENDP3)
#ifdef GNU_LINUX_EMULATION
    usb_lld_rx_enable_buf (ENDP3, endp3_buf, 64);
#else
    usb_lld_rx_enable (ENDP3);
#endif
#ifdef FRAUCHEKY_SUPPORT
  else if (ep_num == ENDP6)
    EP6_OUT_Callback (len);
#else
  (void)len;
#endif
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

#ifdef GNU_LINUX_EMULATION
extern char __process1_stack_base__[4096];
extern char __process3_stack_base__[4096];
#define STACK_SIZE_LED (sizeof __process1_stack_base__)
#define STACK_SIZE_USB (sizeof __process3_stack_base__)
#else
extern uint8_t __process1_stack_base__[], __process1_stack_size__[];
extern uint8_t __process3_stack_base__[], __process3_stack_size__[];
#define STACK_SIZE_LED ((uintptr_t)__process1_stack_size__)
#define STACK_SIZE_USB ((uintptr_t)__process3_stack_size__)
#endif

#define STACK_ADDR_LED ((uintptr_t)__process1_stack_base__)
#define STACK_ADDR_USB ((uintptr_t)__process3_stack_base__)


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

#ifdef GNU_LINUX_EMULATION
static uint8_t endp1_buf[RANDOM_BYTES_LENGTH];
#endif

static void copy_to_tx (uint32_t v, int i)
{
#ifdef GNU_LINUX_EMULATION
  memcpy (&endp1_buf[i], &v, 4);
#else
  usb_lld_txcpy (&v, ENDP1, i * 4, 4);
#endif
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
  uintptr_t v = p[0] + (p[1] << 8) + (p[2] << 16) + (p[3] << 24);

  v -= REGNUAL_START_ADDRESS_COMPATIBLE;
  v += (uintptr_t)addr;
  return v;
}


static int
check_usb_status (void *arg)
{
  (void)arg;

  return (connected || bDeviceState != CONFIGURED
	  || fsij_device_state != FSIJ_DEVICE_RUNNING);
}


#ifdef GNU_LINUX_EMULATION
#define main emulated_main
#endif

/*
 * Entry point.
 *
 * NOTE: the main function is already a thread in the system on entry.
 */
int
main (int argc, char **argv)
{
  uintptr_t entry;
  chopstx_t led_thread, usb_thd;
  unsigned int count;

  (void)argc;
  (void)argv;

  fill_serial_no_by_unique_id ();

  adc_init ();

  event_flag_init (&led_event);

  chopstx_mutex_init (&usb_mtx);
  chopstx_cond_init (&usb_cnd);

#ifdef FRAUCHEKY_SUPPORT
  if (fraucheky_enabled ())
    {
    go_fraucheky:
      bDeviceState = UNCONNECTED;
      running_neug = 0;
      usb_thd = chopstx_create (PRIO_USB, STACK_ADDR_USB, STACK_SIZE_USB,
				usb_main, NULL);
      while (bDeviceState != CONFIGURED)
	chopstx_usec_wait (250*1000);
      set_led (1);
      fraucheky_main ();
      chopstx_cancel (usb_thd);
      chopstx_join (usb_thd, NULL);
      usb_lld_shutdown ();
      bDeviceState = UNCONNECTED;
    }

  running_neug = 1;
#endif

  led_thread = chopstx_create (PRIO_LED, STACK_ADDR_LED, STACK_SIZE_LED,
			       led_blinker, NULL);

  usb_thd = chopstx_create (PRIO_USB, STACK_ADDR_USB, STACK_SIZE_USB,
			    usb_main, NULL);

  neug_init (random_word, RANDOM_BYTES_LENGTH/sizeof (uint32_t));

  chopstx_mutex_lock (&usb_mtx);

 not_configured:
  count = 0;
  /* Initial run-up */
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

  /* Holding USB_MTX...  */
  while (1)
    {
      int last_was_fullsizepacket = 0;

      if (fsij_device_state != FSIJ_DEVICE_RUNNING)
	break;

      chopstx_mutex_unlock (&usb_mtx);
      while (1)
	{
	  uint32_t usec = 5000*1000;
	  chopstx_poll_cond_t poll_desc;
	  struct chx_poll_head *pd_array[1] = {
	    (struct chx_poll_head *)&poll_desc
	  };

	  poll_desc.type = CHOPSTX_POLL_COND;
	  poll_desc.ready = 0;
	  poll_desc.cond = &usb_cnd;
	  poll_desc.mutex = &usb_mtx;
	  poll_desc.check = check_usb_status;
	  poll_desc.arg = NULL;

	  if (chopstx_poll (&usec, 1, pd_array))
	    break;

	  /* Timeout */
	  neug_flush ();
	  neug_mode_select (line_coding.paritytype);
	  event_flag_signal (&led_event, LED_TWOSHOTS);
	}

      chopstx_mutex_lock (&usb_mtx);
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
#ifdef GNU_LINUX_EMULATION
	      usb_lld_tx_enable_buf (ENDP1, endp1_buf, i * 4);
#else
	      usb_lld_tx_enable (ENDP1, i * 4);
#endif
	      chopstx_cond_wait (&usb_cnd, &usb_mtx);
	      count++;
	    }
	}
    }

  chopstx_mutex_unlock (&usb_mtx);

  chopstx_cancel (led_thread);
  chopstx_join (led_thread, NULL);

  /*
   * We come here, because of FSIJ_DEVICE_NEUG_EXIT_REQUESTED
   * or FSIJ_DEVICE_NEUG_FRAUCHEKY_REQUESTED.
   */
  neug_fini ();

  chopstx_mutex_lock (&usb_mtx);
#ifdef FRAUCHEKY_SUPPORT
  if (line_coding.bitrate == NEUG_SPECIAL_BITRATE)
    {
      while (fsij_device_state == FSIJ_DEVICE_NEUG_FRAUCHEKY_REQUESTED)
	{
	  chopstx_mutex_unlock (&usb_mtx);
	  set_led (1);
	  chopstx_usec_wait (500*1000);
	  set_led (0);
	  chopstx_usec_wait (500*1000);
	  chopstx_mutex_lock (&usb_mtx);
	}

      usb_lld_prepare_shutdown ();
      chopstx_mutex_unlock (&usb_mtx);
      chopstx_cancel (usb_thd);
      chopstx_join (usb_thd, NULL);
      usb_lld_shutdown ();
      goto go_fraucheky;
    }
#endif

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

#ifndef GNU_LINUX_EMULATION
  /* Set vector */
  SCB->VTOR = (uintptr_t)&_regnual_start;
#endif
  entry = calculate_regnual_entry_address (&_regnual_start);
#ifdef DFU_SUPPORT
#define FLASH_SYS_START_ADDR 0x08000000
#define FLASH_SYS_END_ADDR (0x08000000+0x1000)
#define CHIP_ID_REG ((uint32_t *)0xE0042000)
  {
    extern uint8_t _sys;
    uint32_t addr;
    handler *new_vector = (handler *)FLASH_SYS_START_ADDR;
    void (*func) (void (*)(void)) = (void (*)(void (*)(void)))new_vector[9];
    uint32_t flash_page_size = 1024; /* 1KiB default */

   if ((*CHIP_ID_ADDR)&0x07 == 0x04) /* High dencity device.  */
     flash_page_size += 0x0400; /* It's 2KiB. */

    /* Kill DFU */
    for (addr = FLASH_SYS_START_ADDR; addr < FLASH_SYS_END_ADDR;
	 addr += flash_page_size)
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
