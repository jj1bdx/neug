#! /usr/bin/python

"""
neug_upgrade.py - a tool to upgrade firmware of Gnuk Token / NeuG device

Copyright (C) 2012, 2015 Free Software Initiative of Japan
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
import sys, time, os, binascii, string
from getpass import getpass

DEFAULT_PW3 = "12345678"

# INPUT: <regnual binary file> <new firmware binary file>

# Assume only single NeuG device is attached to computer

import usb

# USB class, subclass, protocol
COM_CLASS = 0x0a
COM_SUBCLASS = 0x00
COM_PROTOCOL_0 = 0x00

class regnual(object):
    def __init__(self, dev):
        conf = dev.configurations[0]
        intf_alt = conf.interfaces[0]
        intf = intf_alt[0]
        if intf.interfaceClass != 0xff:
            raise ValueError("Wrong interface class")
        self.__devhandle = dev.open()
        try:
            self.__devhandle.setConfiguration(conf)
        except:
            pass
        self.__devhandle.claimInterface(intf)
        self.__devhandle.setAltInterface(intf)

    def mem_info(self):
        mem = self.__devhandle.controlMsg(requestType = 0xc0, request = 0,
                                          buffer = 8, value = 0, index = 0,
                                          timeout = 10000)
        start = ((mem[3]*256 + mem[2])*256 + mem[1])*256 + mem[0]
        end = ((mem[7]*256 + mem[6])*256 + mem[5])*256 + mem[4]
        return (start, end)

    def download(self, start, data, verbose=False):
        addr = start
        addr_end = (start + len(data)) & 0xffffff00
        i = int((addr - 0x08000000) / 0x100)
        j = 0
        print("start %08x" % addr)
        print("end   %08x" % addr_end)
        while addr < addr_end:
            if verbose:
                print("# %08x: %d: %d : %d" % (addr, i, j, 256))
            self.__devhandle.controlMsg(requestType = 0x40, request = 1,
                                        buffer = data[j*256:j*256+256],
                                        value = 0, index = 0,
                                        timeout = 10000)
            crc32code = crc32(data[j*256:j*256+256])
            res = self.__devhandle.controlMsg(requestType = 0xc0, request = 2,
                                              buffer = 4, value = 0, index = 0,
                                              timeout = 10000)
            r_value = ((res[3]*256 + res[2])*256 + res[1])*256 + res[0]
            if (crc32code ^ r_value) != 0xffffffff:
                print("failure")
            self.__devhandle.controlMsg(requestType = 0x40, request = 3,
                                        buffer = None, value = i, index = 0,
                                        timeout = 10000)
            time.sleep(0.010)
            res = self.__devhandle.controlMsg(requestType = 0xc0, request = 2,
                                              buffer = 4, value = 0, index = 0,
                                              timeout = 10000)
            r_value = ((res[3]*256 + res[2])*256 + res[1])*256 + res[0]
            if r_value == 0:
                print("failure")
            i = i+1
            j = j+1
            addr = addr + 256
        residue = len(data) % 256
        if residue != 0:
            if verbose:
                print("# %08x: %d : %d" % (addr, i, residue))
            self.__devhandle.controlMsg(requestType = 0x40, request = 1,
                                        buffer = data[j*256:],
                                        value = 0, index = 0, timeout = 10000)
            crc32code = crc32(data[j*256:].ljust(256,b'\xff'))
            res = self.__devhandle.controlMsg(requestType = 0xc0, request = 2,
                                              buffer = 4, value = 0, index = 0,
                                              timeout = 10000)
            r_value = ((res[3]*256 + res[2])*256 + res[1])*256 + res[0]
            if (crc32code ^ r_value) != 0xffffffff:
                print("failure")
            self.__devhandle.controlMsg(requestType = 0x40, request = 3,
                                        buffer = None, value = i, index = 0,
                                        timeout = 10000)
            time.sleep(0.010)
            res = self.__devhandle.controlMsg(requestType = 0xc0, request = 2,
                                              buffer = 4, value = 0, index = 0,
                                              timeout = 10000)
            r_value = ((res[3]*256 + res[2])*256 + res[1])*256 + res[0]
            if r_value == 0:
                print("failure")

    def protect(self):
        self.__devhandle.controlMsg(requestType = 0x40, request = 4,
                                    buffer = None, value = 0, index = 0,
                                    timeout = 10000)
        time.sleep(0.100)
        res = self.__devhandle.controlMsg(requestType = 0xc0, request = 2,
                                          buffer = 4, value = 0, index = 0,
                                          timeout = 10000)
        r_value = ((res[3]*256 + res[2])*256 + res[1])*256 + res[0]
        if r_value == 0:
            print("protection failure")

    def finish(self):
        self.__devhandle.controlMsg(requestType = 0x40, request = 5,
                                    buffer = None, value = 0, index = 0,
                                    timeout = 10000)

    def reset_device(self):
        try:
            self.__devhandle.reset()
        except:
            pass

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
            raise ValueError("Wrong interface class")
        if interface.interfaceSubClass != COM_SUBCLASS:
            raise ValueError("Wrong interface sub class")
        self.__devhandle = device.open()
        # self.__devhandle.claimInterface(interface)
        # self.__devhandle.setAltInterface(interface)

        self.__intf = interface.interfaceNumber
        self.__alt = interface.alternateSetting
        self.__conf = configuration

        self.__timeout = 10000

    def detach_driver(self):
        self.__devhandle.detachKernelDriver(self.__intf)

    def reset_device(self):
        try:
            self.__devhandle.reset()
        except:
            pass

    def set_passwd(self, passwd):
        self.__devhandle.controlMsg(requestType = 0x40, request = 253,
                                    buffer = passwd, value = 0, index = 0,
                                    timeout = 1000)
        return

    def stop_neug(self, passwd):
        self.__devhandle.controlMsg(requestType = 0x40, request = 255,
                                    buffer = passwd, value = 0, index = 0,
                                    timeout = 1000)
        # self.__devhandle.releaseInterface()
        # self.__devhandle.setConfiguration(0)
        return

    def mem_info(self):
        mem = self.__devhandle.controlMsg(requestType = 0xc0, request = 0,
                                          buffer = 8, value = 0, index = 0,
                                          timeout = 1000)
        start = ((mem[3]*256 + mem[2])*256 + mem[1])*256 + mem[0]
        end = ((mem[7]*256 + mem[6])*256 + mem[5])*256 + mem[4]
        return (start, end)

    def download(self, start, data, verbose=False):
        addr = start
        addr_end = (start + len(data)) & 0xffffff00
        i = int((addr - 0x20000000) / 0x100)
        j = 0
        print("start %08x" % addr)
        print("end   %08x" % addr_end)
        while addr < addr_end:
            if verbose:
                print("# %08x: %d : %d" % (addr, i, 256))
            self.__devhandle.controlMsg(requestType = 0x40, request = 1,
                                        buffer = data[j*256:j*256+256],
                                        value = i, index = 0, timeout = 10)
            i = i+1
            j = j+1
            addr = addr + 256
        residue = len(data) % 256
        if residue != 0:
            print("# %08x: %d : %d" % (addr, i, residue))
            self.__devhandle.controlMsg(requestType = 0x40, request = 1,
                                        buffer = data[j*256:],
                                        value = i, index = 0, timeout = 10)

    def execute(self, last_addr):
        i = int((last_addr - 0x20000000) / 0x100)
        o = (last_addr - 0x20000000) % 0x100
        self.__devhandle.controlMsg(requestType = 0x40, request = 2,
                                    buffer = None, value = i, index = o,
                                    timeout = 10)

def compare(data_original, data_in_device):
    i = 0 
    for d in data_original:
        if ord(d) != data_in_device[i]:
            raise ValueError("verify failed at %08x" % i)
        i += 1

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

USB_VENDOR_FSIJ=0x234b
USB_PRODUCT_GNUK=0x0000

def gnuk_devices_by_vidpid():
    busses = usb.busses()
    for bus in busses:
        devices = bus.devices
        for dev in devices:
            if dev.idVendor != USB_VENDOR_FSIJ:
                continue
            if dev.idProduct != USB_PRODUCT_GNUK:
                continue
            yield dev

def to_string(t):
    result = ""
    for c in t:
        result += chr(c)
    return result

def UNSIGNED(n):
    return n & 0xffffffff

def crc32(bytestr):
    crc = binascii.crc32(bytestr)
    return UNSIGNED(crc)

def main(wait_e, passwd, data_regnual, data_upgrade):
    l = len(data_regnual)
    if (l & 0x03) != 0:
        data_regnual = data_regnual.ljust(l + 4 - (l & 0x03), b'\x00')
    crc32code = crc32(data_regnual)
    print("CRC32: %04x\n" % crc32code)
    data_regnual += pack('<I', crc32code)
    com = None
    for (dev, config, intf) in com_devices():
        try:
            com = neug(dev, config, intf)
            print("Device: %s" % dev.filename)
            print("Configuration: %d" % config.value)
            print("Interface: %d" % intf.interfaceNumber)
            break
        except:
            pass
    if not com:
        raise ValueError("No NeuG Device Present")
    com.stop_neug(passwd)
    time.sleep(1.500)
    mem_info = com.mem_info()
    print("%08x:%08x" % mem_info)
    com.detach_driver()
    time.sleep(1.500)
    print("Downloading flash upgrade program...")
    com.download(mem_info[0], data_regnual)
    print("Run flash upgrade program...")
    com.execute(mem_info[0] + len(data_regnual) - 4)
    #
    time.sleep(3)
    com.reset_device()
    del com
    com = None
    #
    reg = None
    while not reg:
        print("Wait %d seconds..." % wait_e)
        time.sleep(wait_e)
        for dev in gnuk_devices_by_vidpid():
            try:
                reg = regnual(dev)
                print("Device: %s" % dev.filename)
                break
            except:
                pass
    # Then, send upgrade program...
    mem_info = reg.mem_info()
    print("%08x:%08x" % mem_info)
    print("Downloading the program")
    reg.download(mem_info[0], data_upgrade)
    reg.protect()
    reg.finish()
    reg.reset_device()
    return 0


# This should be event driven, not guessing some period.
DEFAULT_WAIT_FOR_REENUMERATION=10

if __name__ == '__main__':
    passwd = None
    wait_e = DEFAULT_WAIT_FOR_REENUMERATION
    if len(sys.argv) == 2 and sys.argv[1] == '-s': # S for set passwd
        passwd = getpass("Admin password: ")
        com = None
        for (dev, config, intf) in com_devices():
            try:
                com = neug(dev, config, intf)
                print("Device: %s" % dev.filename)
                print("Configuration: %d" % config.value)
                print("Interface: %d" % intf.interfaceNumber)
                break
            except:
                pass
        if not com:
            raise ValueError("No NeuG Device Present")
        com.set_passwd(passwd)
        exit(0)
    while len(sys.argv) > 3:
        option = sys.argv[1]
        sys.argv.pop(1)
        if option == '-f':      # F for Factory setting
            passwd = DEFAULT_PW3
        elif option == '-e':    # E for Enumeration
            wait_e = int(sys.argv[1])
            sys.argv.pop(1)
        else:
            raise ValueError("unknown option", option)
    if not passwd:
        passwd = getpass("Admin password: ")
    filename_regnual = sys.argv[1]
    filename_upgrade = sys.argv[2]
    f = open(filename_regnual, "rb")
    data_regnual = f.read()
    f.close()
    print("%s: %d" % (filename_regnual, len(data_regnual)))
    f = open(filename_upgrade, "rb")
    data_upgrade = f.read()
    f.close()
    print("%s: %d" % (filename_upgrade, len(data_upgrade)))
    main(wait_e, passwd, data_regnual, data_upgrade[4096:])
