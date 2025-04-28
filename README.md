TI CC13xx/CC2538/CC26xx Serial Boot Loader
==========================================

This folder contains a python script that communicates with the boot loader of the Texas Instruments CC2340 SoCs (System on Chips).
It can be used to erase, program, verify and read the flash of those SoCs with a simple USB to serial converter.

### Requirements

To run this script you need a Python interpreter, Linux and Mac users should be fine, Windows users have a look here: [Python Download][python].

Alternatively, Docker can be used to run this script as a one-liner without the need to install dependencies, see [git-developer/ti-cc-tool](https://github.com/git-developer/ti-cc-tool) for details.

To communicate with the uart port of the SoC you need a usb to serial converter:
* If you use XDS110 you can use the on-board USB to UART port. Select XDS110 Class Application/User UART for the COM port.
* If you use a different platform, there are many cheap USB to UART converters available, but make sure you use one with 3.3v voltage levels.

### Dependencies

This script uses the pyserial package to communicate with the serial port and chip (https://pypi.org/project/pyserial/). You can install it by running `pip install pyserial`.

If you want to be able to program your device from an Intel Hex file, you will need to install the IntelHex package: https://pypi.python.org/pypi/IntelHex (e.g. by running `pip install intelhex`).

The script will try to auto-detect whether your firmware is a raw binary or an Intel Hex by using python-magic:
(https://pypi.python.org/pypi/python-magic). You can install it by running `pip install python-magic-bin` on Windows PC, and `pip install python-magic-bin` on Linux or MacOS machines. Please bear in mind that installation of python-magic may have additional dependencies, depending on your OS: (https://github.com/ahupp/python-magic#dependencies).

If python-magic is _not_ installed, the script will try to auto-detect the firmware type by looking at the filename extension, but this is sub-optimal. If the extension is `.hex`, `.ihx` or `.ihex`, the script will assume that the firmware is an Intel Hex file. In all other cases, the firmware will be treated as raw binary.

You can run `pip install -r requirements.txt` to install requirements for this tool.

### CC2340

The script has been tested with LP-EM-CC2340R5 + LP-XDS110. 

For CC23xx families, the ROM bootloader is configured through CCFG.bootCfg. ROM bootloader is running by default on a blank device. If a valid image is present, then the bootloader index and backdoor pin must also be configured correctly. The bootloader configuration is in SysConfig - Device Configurations - Boot Configuration.

See chapter 8 Device Boot and Bootloader in CC23xx SimpleLink Wireless Microcontroller Unit Technical Reference Manual to find the detail.

### Usage

Install requirements by running `pip install -r requirements.txt`.

Usage: cc2340-bsl.py [-h] [-q] [-V] [-f] [-e] [-w] [-v] [-p PORT] [-b BAUD] [-a ADDRESS] [--version] file

positional arguments:

  file

options:

  -h, --help            show this help message and exit
  
  -q                    Quiet
  
  -V                    Verbose
  
  -f, --force           Force operation(s) without asking any questions
  
  -e, --erase           Mass erase
  
  -w, --write           Write
  
  -v, --verify          Verify (CRC32 check)
  
  -p PORT, --port PORT  Serial port (default: first USB-like port in /dev)
  
  -b BAUD, --baud BAUD  Baud speed (default: 500000)
  
  -a ADDRESS, --address ADDRESS
  
                        Target address
						
  --version             show program's version number and exit

Example usage: python cc2340-bsl.py -p COM32 -b 1000000 -e -w -v test.hex