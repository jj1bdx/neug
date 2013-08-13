#! /usr/bin/python

"""
neug_check.py - a tool to check NeuG device

Copyright (C) 2012 Free Software Initiative of Japan
Author: NIIBE Yutaka <gniibe@fsij.org>

This file is a part of NeuG, a TRNG implementation.

NeuG is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

NeuG is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

from struct import *
import sys, time, os, string

# Assume only single NeuG device is attached to computer

import usb

# USB class, subclass, protocol
COM_CLASS = 0x0a
COM_SUBCLASS = 0x00
COM_PROTOCOL_0 = 0x00

class neug(object):
    def __init__(self, device, configuration, interface):
        """
        __init__(device, configuration, interface) -> None
        Initialize the device.
        device: usb.Device object.
        configuration: configuration number.
        interface: usb.Interface object representing the interface and altenate setting.
        """
        if interface.interfaceClass !=COM_CLASS:
            raise ValueError, "Wrong interface class"
        if interface.interfaceSubClass != COM_SUBCLASS:
            raise ValueError, "Wrong interface sub class"
        self.__devhandle = device.open()
        # self.__devhandle.claimInterface(interface)
        # self.__devhandle.setAltInterface(interface)

        self.__intf = interface.interfaceNumber
        self.__alt = interface.alternateSetting
        self.__conf = configuration

        self.__timeout = 10000

    def get_string(self,no,maxsize):
        return self.__devhandle.getString(no,maxsize)

    def get_mode(self):
        mode = self.__devhandle.controlMsg(requestType = 0xc0, request = 254,
                                          value = 0, index = 0, buffer = 1,
                                          timeout = 1000)
        if mode[0] == 0:
            return "Conditioned"
        elif mode[0] == 1:
            return "Raw data (CRC32 filter)"
        else:
            return "Raw data (ADC samples)"

    def get_info(self, index):
        err = self.__devhandle.controlMsg(requestType = 0xc0, request = 254,
                                          value = 0, index = index, buffer = 2,
                                          timeout = 1000)
        return err[1]*256 + err[0]

def com_devices():
    busses = usb.busses()
    for bus in busses:
        devices = bus.devices
        for dev in devices:
            for config in dev.configurations:
                for intf in config.interfaces:
                    for alt in intf:
                        if alt.interfaceClass == COM_CLASS and \
                                alt.interfaceSubClass == COM_SUBCLASS and \
                                alt.interfaceProtocol == COM_PROTOCOL_0:
                            yield dev, config, alt

field = [ '', 'Vendor', 'Product', 'Serial', 'Revision', 'Config', 'Sys' ]

def main():
    com = None
    for (dev, config, intf) in com_devices():
        try:
            com = neug(dev, config, intf)
            print "Device: ", dev.filename
            print "Configuration: ", config.value
            print "Interface: ", intf.interfaceNumber
            break
        except:
            pass
    if not com:
        raise ValueError, "No NeuG Device Present"
    print
    for i in range(1,7):
        s = com.get_string(i, 512)
        print "%9s: %s" % (field[i], s)
    print
    print "mode: %s" % com.get_mode()
    print "Repeat errors: %d" % com.get_info(2)
    print "PP 64  errors: %d" % com.get_info(3)
    print "PP 4k  errors: %d" % com.get_info(4)
    print "Total  errors: %d" % com.get_info(1)
    print "Repeat max counts: %d" % com.get_info(5)
    print "PP 64  max counts: %d" % com.get_info(6)
    print "PP 4k  max counts: %d" % com.get_info(7)
    return 0


if __name__ == '__main__':
    main()
