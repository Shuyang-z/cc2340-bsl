#!/usr/bin/env python3

# Copyright (c) 2014, Jelmer Tiete <jelmer@tiete.be>.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. The name of the author may not be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.

# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Implementation based on stm32loader by Ivan A-R <ivan@tuxotronic.org>

# Serial boot loader over UART for CC13xx / CC2538 / CC26xx
# Based on the info found in TI's swru333a.pdf (spma029.pdf)
#
# Bootloader only starts if no valid image is found or if boot loader
# backdoor is enabled.
# Make sure you don't lock yourself out!! (enable backdoor in your firmware)
# More info at https://github.com/JelmerT/cc2538-bsl

from subprocess import Popen, PIPE

import sys
import glob
import math
import time
import os
import struct
import binascii
import traceback
import argparse

import serial.tools
import serial.tools.list_ports

# version
__version__ = "2.1"

# Verbose level
QUIET = 5

# Default Baud
DEFAULT_BAUD = 500000

# Addresses and sizes for device family
CC2340_CCFG_ADDR = 0x4E020000
CC2340_CCFG_SIZE = 0x800
CC2340R5_FLASH_SIZE = 0x80000


try:
    import serial
except ImportError:
    print('{} requires the Python serial library'.format(sys.argv[0]))
    print('Please install it with:')
    print('')
    print('   pip3 install pyserial')
    sys.exit(1)


def mdebug(level, message, attr='\n'):
    if QUIET >= level:
        print(message, end=attr, file=sys.stderr)


RETURN_CMD_STRS = {0x40: 'Success',
                   0x41: 'Unknown command',
                   0x42: 'Invalid command',
                   0x43: 'Invalid address',
                   0x44: 'Flash fail',
                   0x45: 'CRC fail',
                   0x46: 'Need chip erase'
                   }

COMMAND_RET_SUCCESS = 0x40
COMMAND_RET_UNKNOWN_CMD = 0x41
COMMAND_RET_INVALID_CMD = 0x42
COMMAND_RET_INVALID_ADR = 0x43
COMMAND_RET_FLASH_FAIL = 0x44
COMMAND_RET_CRC_FAIL = 0x45
COMMAND_RET_NEEDS_CHIP_ERASE = 0x46


class CmdException(Exception):
    pass


class FirmwareFile(object):
    HEX_FILE_EXTENSIONS = ('hex', 'ihx', 'ihex')

    def __init__(self, path):
        """
        Read a firmware file and store its data ready for device programming.

        This class will try to guess the file type if python-magic is available.

        If python-magic indicates a plain text file, and if IntelHex is
        available, then the file will be treated as one of Intel HEX format.

        In all other cases, the file will be treated as a raw binary file.

        In both cases, the file's contents are stored in bytes for subsequent
        usage to program a device or to perform a crc check.

        CC23xx/CC27xx hex contains main flash and CCFG content, need to split
        the bytes since the addresses are not consecutive.

        Parameters:
            path -- A str with the path to the firmware file.

        Attributes:
            mainFlashBytes: A bytearray with firmware main flash contents ready 
            to send to the device
            ccfgBytes: A bytearray with firmware CCFG contents ready to send to 
            the device
        """
        self._main_flash_crc32 = None
        self._ccfg_crc32 = None

        try:
            from magic import from_file
            file_type = from_file(path, mime=True)

            if file_type == 'text/plain' or file_type == 'text/x-hex':
                mdebug(5, "Firmware file: Intel Hex")
                self.__read_hex(path)
            elif file_type == 'application/octet-stream':
                # mdebug(5, "Firmware file: Raw Binary")
                # self.__read_bin(path)
                error_str = "This tool only supports hex. Magic " \
                            "indicates '%s'" % (file_type)
                raise CmdException(error_str)
            else:
                error_str = "Could not determine firmware type. Magic " \
                            "indicates '%s'" % (file_type)
                raise CmdException(error_str)
        except ImportError:
            if os.path.splitext(path)[1][1:] in self.HEX_FILE_EXTENSIONS:
                mdebug(5, "Your firmware looks like an Intel Hex file")
                self.__read_hex(path)
            else:
                mdebug(5, "Cannot auto-detect firmware filetype: Assuming .bin")
                self.__read_bin(path)

            mdebug(10, "For more solid firmware type auto-detection, install "
                       "python-magic.")
            mdebug(10, "Please see the readme for more details.")
        
        self._read_crc32()
        mdebug(10, "CRC32 for main flash is 0x%x" % self._main_flash_crc32)
        mdebug(10, "CRC32 for CCFG is 0x%x" % self._ccfg_crc32)

    def __read_hex(self, path):
        try:
            from intelhex import IntelHex
            self.mainFlashBytes = bytearray(IntelHex(path).tobinarray(start=0x0, size=CC2340R5_FLASH_SIZE))
            self.ccfgBytes = bytearray(IntelHex(path).tobinarray(start=CC2340_CCFG_ADDR, size=CC2340_CCFG_SIZE))
            if self.ccfgBytes == None:
                error_str = "The image does not contain a valid CCFG region. " \
                            "CCFG is needed to bring up the device after firmware update.\n" \
                            "Please check the image to make sure a valid CCFG is included."
                raise CmdException(error_str)
        except ImportError:
            error_str = "Firmware is Intel Hex, but the IntelHex library " \
                        "could not be imported.\n" \
                        "Install IntelHex in site-packages or program " \
                        "your device with a raw binary (.bin) file.\n" \
                        "Please see the readme for more details."
            raise CmdException(error_str)

    # def __read_bin(self, path):
    #     with open(path, 'rb') as f:
    #         self.bytes = bytearray(f.read())

    def _read_crc32(self):
        """
        Read the crc32 checksum of the firmware image

        For CC2340, need to seperate crc32 for main flash and CCFG
        """
        if self._main_flash_crc32 == None:
            self._main_flash_crc32 = binascii.crc32(bytearray(self.mainFlashBytes)) & 0xffffffff
        if self._ccfg_crc32 == None:
            self._ccfg_crc32 = binascii.crc32(bytearray(self.ccfgBytes)) & 0xffffffff


class CommandInterface(object):

    ACK_BYTE = 0xCC
    NACK_BYTE = 0x33

    def open(self, aport=None, abaudrate=DEFAULT_BAUD):
        # Try to create the object using serial_for_url(), or fall back to the
        # old serial.Serial() where serial_for_url() is not supported.
        # serial_for_url() is a factory class and will return a different
        # object based on the URL. For example serial_for_url("/dev/tty.<xyz>")
        # will return a serialposix.Serial object for Ubuntu or Mac OS;
        # serial_for_url("COMx") will return a serialwin32.Serial oject for Windows OS.
        # For that reason, we need to make sure the port doesn't get opened at
        # this stage: We need to set its attributes up depending on what object
        # we get.
        try:
            self.sp = serial.serial_for_url(aport, do_not_open=True, timeout=10, write_timeout=10)
        except AttributeError:
            self.sp = serial.Serial(port=None, timeout=10, write_timeout=10)
            self.sp.port = aport

        if ((os.name == 'nt' and isinstance(self.sp, serial.serialwin32.Serial)) or \
           (os.name == 'posix' and isinstance(self.sp, serial.serialposix.Serial))):
            self.sp.baudrate=abaudrate        # baudrate
            self.sp.bytesize=8                # number of databits
            self.sp.parity=serial.PARITY_NONE # parity
            self.sp.stopbits=1                # stop bits
            self.sp.xonxoff=0                 # s/w (XON/XOFF) flow control
            self.sp.rtscts=0                  # h/w (RTS/CTS) flow control
            self.sp.timeout=1.0               # set the timeout value

        self.sp.open()

    def close(self):
        self.sp.close()

    def _wait_for_ack(self, info="", timeout=4):
        stop = time.time() + timeout
        got = bytearray(2)
        while got[-2] != 00 or got[-1] not in (CommandInterface.ACK_BYTE,
                                               CommandInterface.NACK_BYTE):
            got += self._read(1)
            if time.time() > stop:
                raise CmdException("Timeout waiting for ACK/NACK after '%s'"
                                   % (info,))

        # Our bytearray's length is: 2 initial bytes + 2 bytes for the ACK/NACK
        # plus a possible N-4 additional (buffered) bytes
        mdebug(10, "Got %d additional bytes before ACK/NACK" % (len(got) - 4,))

        # wait for ask
        ask = got[-1]

        if ask == CommandInterface.ACK_BYTE:
            # ACK
            return 1
        elif ask == CommandInterface.NACK_BYTE:
            # NACK
            mdebug(10, "Target replied with a NACK during %s" % info)
            return 0

        # Unknown response
        mdebug(10, "Unrecognised response 0x%x to %s" % (ask, info))
        return 0

    def _encode_addr(self, addr):
        byte3 = (addr >> 0) & 0xFF
        byte2 = (addr >> 8) & 0xFF
        byte1 = (addr >> 16) & 0xFF
        byte0 = (addr >> 24) & 0xFF
        return bytes([byte0, byte1, byte2, byte3])

    def _decode_addr(self, byte0, byte1, byte2, byte3):
        return ((byte3 << 24) | (byte2 << 16) | (byte1 << 8) | (byte0 << 0))

    def _calc_checks(self, cmd, addr, size, expectedCRC):
        return ((sum(bytearray(self._encode_addr(addr))) +
                 sum(bytearray(self._encode_addr(size))) +
                 sum(bytearray(self._encode_addr(expectedCRC))) +
                 cmd) & 0xFF)

    def _write(self, data, is_retry=False):
        if type(data) == int:
            assert data < 256
            goal = 1
            written = self.sp.write(bytes([data]))
        elif type(data) == bytes or type(data) == bytearray:
            goal = len(data)
            written = self.sp.write(data)
        else:
            raise CmdException("Internal Error. Bad data type: {}"
                               .format(type(data)))

        if written < goal:
            mdebug(10, "*** Only wrote {} of target {} bytes"
                   .format(written, goal))
            if is_retry and written == 0:
                raise CmdException("Failed to write data on the serial bus")
            mdebug(10, "*** Retrying write for remainder")
            if type(data) == int:
                return self._write(data, is_retry=True)
            else:
                return self._write(data[written:], is_retry=True)

    def _read(self, length):
        return bytearray(self.sp.read(length))

    def sendAck(self):
        self._write(0x00)
        self._write(0xCC)
        return

    def sendNAck(self):
        self._write(0x00)
        self._write(0x33)
        return

    def receivePacket(self):
        # stop = time.time() + 5
        # got = None
        # while not got:
        got = self._read(2)
        #     if time.time() > stop:
        #         break

        # if not got:
        #     raise CmdException("No response to %s" % info)

        size = got[0]  # rcv size
        chks = got[1]  # rcv checksum
        data = bytearray(self._read(size - 2))  # rcv data

        mdebug(10, "*** received %x bytes" % size)
        if chks == sum(data) & 0xFF:
            self.sendAck()
            return data
        else:
            self.sendNAck()
            # TODO: retry receiving!
            raise CmdException("Received packet checksum error")
            return 0

    def sendSynch(self):
        cmd = 0x55

        # flush serial input buffer for first ACK reception
        self.sp.flushInput()

        mdebug(10, "*** sending synch sequence")
        self._write(cmd)  # send U
        self._write(cmd)  # send U
        return self._wait_for_ack("Synch (0x55 0x55)", 8)

    def checkLastCmd(self):
        stat = self.cmdGetStatus()
        if not (stat):
            raise CmdException("No response from target on status request. "
                               "(Did you disable the bootloader?)")

        if stat[0] == COMMAND_RET_SUCCESS:
            mdebug(10, "Command Successful")
            return 1
        else:
            stat_str = RETURN_CMD_STRS.get(stat[0], None)
            if stat_str == None:
                mdebug(0, "Warning: unrecognized status returned "
                          "0x%x" % stat[0])
            else:
                mdebug(0, "Target returned: 0x%x, %s" % (stat[0], stat_str))
            return 0

    def cmdPing(self):
        cmd = 0x20
        lng = 3

        self._write(lng)  # send size
        self._write(cmd)  # send checksum
        self._write(cmd)  # send data

        mdebug(10, "*** Ping command (0x20)")
        if self._wait_for_ack("Ping (0x20)"):
            return self.checkLastCmd()

    def cmdReset(self):
        cmd = 0x23
        lng = 3

        self._write(lng)  # send size
        self._write(cmd)  # send checksum
        self._write(cmd)  # send data

        mdebug(10, "*** Reset command (0x23)")
        if self._wait_for_ack("Reset (0x23)"):
            return 1

    def cmdGetChipId(self):
        cmd = 0x22
        lng = 3

        self._write(lng)  # send size
        self._write(cmd)  # send checksum
        self._write(cmd)  # send data

        mdebug(10, "*** GetChipId command (0x22)")
        if self._wait_for_ack("Get ChipID (0x22)"):
            # 4 byte answ, the 2 LSB hold chip ID
            version = self.receivePacket()
            if self.checkLastCmd():
                assert len(version) == 4, ("Unreasonable chip "
                                           "id: %s" % repr(version))
                mdebug(10, "    Version 0x%02X%02X%02X%02X" % tuple(version))
                variant_and_chip_id = (version[1] << 16) | (version[2] << 8) | version[3]
                return variant_and_chip_id
            else:
                raise CmdException("GetChipID (0x22) failed")

    def cmdGetStatus(self):
        cmd = 0x21
        lng = 3

        self._write(lng)  # send size
        self._write(cmd)  # send checksum
        self._write(cmd)  # send data

        mdebug(10, "*** GetStatus command (0x21)")
        if self._wait_for_ack("Get Status (0x21)"):
            stat = self.receivePacket()
            return stat

    def cmdBankErase(self):
        cmd = 0x24
        lng = 3

        self._write(lng)  # send length
        self._write(cmd)  # send checksum
        self._write(cmd)  # send cmd

        mdebug(10, "*** Bank Erase command(0x24)")
        if self._wait_for_ack("Bank Erase (0x24)", 10):
            return self.checkLastCmd()

    def cmdCRC32(self, addr, size, expectedCRC):
        cmd = 0x25
        lng = 15

        self._write(lng)  # send length
        self._write(self._calc_checks(cmd, addr, size, expectedCRC))  # send checksum
        self._write(cmd)  # send cmd
        self._write(self._encode_addr(addr))  # send addr
        self._write(self._encode_addr(size))  # send size
        self._write(self._encode_addr(expectedCRC))  # send expected CRC

        mdebug(10, "*** CRC32 command(0x25)")
        if self._wait_for_ack("Get CRC32 (0x25)", 10):
            return self.checkLastCmd()

    def cmdDownload(self, addr, size):
        cmd = 0x26
        lng = 11

        if (size % 4) != 0:  # check for invalid data lengths
            raise Exception('Invalid data size: %i. '
                            'Size must be a multiple of 4.' % size)

        self._write(lng)  # send length
        self._write(self._calc_checks(cmd, addr, size, 0x0))  # send checksum
        self._write(cmd)  # send cmd
        self._write(self._encode_addr(addr))  # send addr
        self._write(self._encode_addr(size))  # send size

        mdebug(10, "*** Download command (0x26)")
        if self._wait_for_ack("Download (0x26)", 2):
            return self.checkLastCmd()

    def cmdSendData(self, data):
        cmd = 0x28
        lng = len(data)+3
        # TODO: check total size of data!! max 252 bytes!

        self._write(lng)  # send size
        self._write((sum(bytearray(data))+cmd) & 0xFF)  # send checksum
        self._write(cmd)  # send cmd
        self._write(bytearray(data))  # send data

        mdebug(10, "*** Send Data (0x28)")
        if self._wait_for_ack("Send data (0x28)", 10):
            return self.checkLastCmd()


# Complex commands section

    def writeMemory(self, addr, data):
        lng = len(data)
        # amount of data bytes transferred per packet (theory: max 252 + 3)
        trsf_size = 248
        empty_packet = bytearray((0xFF,) * trsf_size)

        offs = 0
        addr_set = 0

        # check if amount of remaining data is less then packet size
        while lng > trsf_size:
            # skip packets filled with 0xFF
            if data[offs:offs+trsf_size] != empty_packet:
                if addr_set != 1:
                    # set starting address if not set
                    self.cmdDownload(addr, lng)
                    addr_set = 1
                mdebug(5, " Write %(len)d bytes at 0x%(addr)08X"
                       % {'addr': addr, 'len': trsf_size}, '\r')
                sys.stdout.flush()

                # send next data packet
                self.cmdSendData(data[offs:offs+trsf_size])
            else:   # skipped packet, address needs to be set
                addr_set = 0

            offs = offs + trsf_size
            addr = addr + trsf_size
            lng = lng - trsf_size

        mdebug(5, "Write %(len)d bytes at 0x%(addr)08X" % {'addr': addr,
                                                           'len': lng})
        self.cmdDownload(addr, lng)
        return self.cmdSendData(data[offs:offs+lng])  # send last data packet


class Chip(object):
    def __init__(self, command_interface):
        self.command_interface = command_interface

        # Some defaults. The child can override.
        self.flash_start_addr = 0x00000000
        self.ccfgStartAddr = CC2340_CCFG_ADDR
        self.page_size = 2048


    def page_align_up(self, value):
        return int(math.ceil(value / self.page_size) * self.page_size)


    def page_align_down(self, value):
        return int(math.floor(value / self.page_size) * self.page_size)


    def page_to_addr(self, pages):
        addresses = []
        for page in pages:
            addresses.append(int(device.flash_start_addr) +
                             int(page)*self.page_size)
        return addresses

    def crc(self, address, size, expectedCRC):
        return getattr(self.command_interface, self.crc_cmd)(address, size, expectedCRC)


class CC2340(Chip):
    # Class constants
    CCFG_ADDR = CC2340_CCFG_ADDR

    def __init__(self, command_interface):
        super(CC2340, self).__init__(command_interface)
        self.bootloader_dis_val = 0x00000000
        self.crc_cmd = "cmdCRC32"

    def erase(self):
        mdebug(5, "Erasing all main bank flash sectors")
        return self.command_interface.cmdBankErase()


def query_yes_no(question, default="yes"):
    valid = {"yes": True,
             "y": True,
             "ye": True,
             "no": False,
             "n": False}
    if default == None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = input().lower()
        if default != None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "
                             "(or 'y' or 'n').\n")


def _parse_range_values(device, values):
    if len(values) and len(values) < 3:
        page_addr_range = []
        try:
            for value in values:
                try:
                    if int(value) % int(device.page_size) != 0:
                        raise ValueError("Supplied addresses are not page_size: "
                                         "{} aligned".format(device.page_size))
                    page_addr_range.append(int(value))
                except ValueError:
                    if int(value, 16) % int(device.page_size) != 0:
                        raise ValueError("Supplied addresses are not page_size: "
                                         "{} aligned".format(device.page_size))
                    page_addr_range.append(int(value, 16))
            return page_addr_range
        except ValueError:
            raise ValueError("Supplied value is not a page or an address")
    else:
        raise ValueError("Supplied range is neither a page or address range")


def parse_page_address_range(device, pg_range):
    """Convert the address/page range into a start address and byte length"""
    values = pg_range.split(',')
    page_addr = []
    # check if first argument is character
    if values[0].isalpha():
        values[0].lower()
        if values[0] == 'p' or values[0] == 'page':
            if values[0] == 'p':
                values[1:] = device.page_to_addr(values[1:])
        elif values[0] != 'a' and values[0] != 'address':
            raise ValueError("Prefix is neither a(address) or p(page)")
        page_addr.extend(_parse_range_values(device, values[1:]))
    else:
        page_addr.extend(_parse_range_values(device, values))
    if len(page_addr) == 1:
        return [page_addr[0], device.page_size]
    else:
        return [page_addr[0], (page_addr[1] - page_addr[0])]


def version():
    # Get the version using "git describe".
    try:
        p = Popen(['git', 'describe', '--tags', '--match', '[0-9]*'],
                  stdout=PIPE, stderr=PIPE)
        p.stderr.close()
        line = p.stdout.readlines()[0]
        return line.decode('utf-8').strip()
    except:
        # We're not in a git repo, or git failed, use fixed version string.
        return __version__


def cli_setup():
    parser = argparse.ArgumentParser()

    parser.add_argument('-q', action='store_true', help='Quiet')
    parser.add_argument('-V', action='store_true', help='Verbose')
    parser.add_argument('-f', '--force', action='store_true', help='Force operation(s) without asking any questions')
    parser.add_argument('-e', '--erase', action='store_true', help='Mass erase')
    parser.add_argument('-w', '--write', action='store_true', help='Write')
    parser.add_argument('-v', '--verify', action='store_true', help='Verify (CRC32 check)')
    parser.add_argument('-p', '--port', help='Serial port (default: first USB-like port in /dev)')
    parser.add_argument('-b', '--baud', type=int, help=f"Baud speed (default: {DEFAULT_BAUD})")
    parser.add_argument('-a', '--address', type=int, help='Target address')
    parser.add_argument('--version', action='version', version='%(prog)s ' + version())
    parser.add_argument('file')

    return parser.parse_args()

if __name__ == "__main__":
    args = cli_setup()

    force_speed = False

    if args.baud:
        if args.baud < 9600 or args.baud > 1000000:
            raise CmdException("Baud rate out of range. Support baud rate "
                               "from 9600 to 1000000.")
        else:
            force_speed = True
    else:
        args.baud = DEFAULT_BAUD

    if args.V:
        QUIET = 10
    elif args.q:
        QUIET = 0

    try:
        # Try and find the port automatically
        if not args.port:
            ports = []

            if (os.name == 'nt'):
                for port in serial.tools.list_ports.comports():
                    # Find port description of XDS110 Appllication UART
                    if "XDS110 Class Application/User UART" in port.description:
                        ports.extend(port)
            else:
                # Get a list of all USB-like names in /dev
                for name in ['ttyACM',
                            'tty.usbserial',
                            'ttyUSB',
                            'tty.usbmodem',
                            'tty.SLAB_USBtoUART']:
                    ports.extend(glob.glob('/dev/%s*' % name))

            ports = sorted(ports)

            if ports:
                # Found something - take it
                args.port = ports[0]
            else:
                raise Exception('No serial port found.')

        cmd = CommandInterface()
        cmd.open(args.port, args.baud)
        mdebug(5, "Opening port %(port)s, baud %(baud)d"
               % {'port': args.port, 'baud': args.baud})
        if args.write or args.verify:
            mdebug(5, "Reading data from %s" % args.file)
            firmware = FirmwareFile(args.file)

        mdebug(5, "Connecting to target...")

        if not cmd.sendSynch():
            raise CmdException("Can't connect to target. Ensure boot loader "
                               "is started. (no answer on synch sequence)")

        if (cmd.cmdPing() != 1):
            raise CmdException("Can't connect to target. Ensure boot loader "
                               "is started. (no answer on ping command)")

        variant_and_chip_id = cmd.cmdGetChipId()
        chip_id = variant_and_chip_id & 0xFFFF
        variant = (variant_and_chip_id & 0xFF0000) >> 16
        if (chip_id == 0x1A96) or ((chip_id == 0x2DDA) and (variant != 0x9E)):
            chip_id_str = 'CC2340R5'
        elif (chip_id == 0xF9EC) or ((chip_id == 0x2DDA) and (variant == 0x9E)):
            chip_id_str = 'CC2340R2'

        if chip_id_str == None:
            mdebug(10, '    Unrecognized chip ID. Trying CC2340')
            device = CC2340(cmd)
        else:
            mdebug(10, "    Target id 0x%x, %s" % (chip_id, chip_id_str))
            device = CC2340(cmd)

        # Choose a good default address unless the user specified -a
        if not args.address:
            args.address = device.flash_start_addr

        if args.erase:
            mdebug(5, "    Performing mass erase")
            if device.erase():
                mdebug(5, "    Erase done")
            else:
                raise CmdException("Erase failed")

        if args.write:
            # TODO: check if boot loader back-door is open, need to read
            #       flash size first to get address
            if cmd.writeMemory(device.CCFG_ADDR, firmware.ccfgBytes):
                mdebug(5, "    Write CCFG done                           ")
            else:
                raise CmdException("Write CCFG failed                  ")
            if cmd.writeMemory(args.address, firmware.mainFlashBytes):
                mdebug(5, "    Write main flash done                     ")
            else:
                raise CmdException("Write main flash failed            ")

        if args.verify:
            mdebug(5, "Verifying by comparing CRC32 calculations.")

            crc_local_main_flash = firmware._main_flash_crc32
            crc_local_ccfg = firmware._ccfg_crc32
            # CRC of target will change according to length input file
            # crc_target_main_flash = device.crc(args.address, len(firmware.mainFlashBytes))
            crc_match_main_flash = device.crc(args.address, 0x80000, crc_local_main_flash)
            crc_match_ccfg = device.crc(CC2340_CCFG_ADDR, len(firmware.ccfgBytes), crc_local_ccfg)

            if crc_match_main_flash == 1:
                mdebug(5, "    Main flash verified (match: 0x%08x)" % crc_local_main_flash)
            else:
                cmd.cmdReset()
                raise Exception("NO CRC32 match for main flash: status = 0x%x"
                                 % crc_match_main_flash)
            
            if crc_match_ccfg == 1:
                mdebug(5, "    CCFG verified (match: 0x%08x)" % crc_local_ccfg)
            else:
                cmd.cmdReset()
                raise Exception("NO CRC32 match for CCFG: Local = 0x%x"
                                 % crc_match_ccfg)

        cmd.cmdReset()

    except Exception as err:
        if QUIET >= 10:
            traceback.print_exc()
        exit('ERROR: %s' % str(err))
