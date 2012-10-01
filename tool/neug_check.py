#! /usr/bin/python

"""
neug_check.py - a tool to check NeuG device

Copyright (C) 2012 Free Software Initiative of Japan
Author: NIIBE Yutaka <gniibe@fsij.org>

This file is a part of NeuG, a TRNG implementation.

Gnuk is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Gnuk is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

from struct import *
import sys, time, os, string

# INPUT: binary file

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

    def get_err_count(self):
        err = self.__devhandle.controlMsg(requestType = 0xc0, request = 254,
                                          value = 0, index = 0, buffer = 2,
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
    print "ERRORS: %d" % com.get_err_count()
    return 0


if __name__ == '__main__':
    main()
