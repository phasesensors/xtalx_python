xtalx
=====
This package provides a library for interfacing with the XtalX pressure sensor.
Python version 3 is required.  The easiest way to install the xtalx library is
using pip::

    python3 -m pip install xtalx

Note that you may wish to use sudo to install xtalx for all users on your
system::

    sudo python3 -m pip install xtalx

You may also install the package from the source using::

    make install

or::

    sudo make install

The xtalx python libraries currently require the pyusb package to be installed
for communicating with the sensor; this requires the libusb back-end to be
installed on the target system.  On Linux-based systems libusb typically comes
pre-installed.  On macOS-based systems it can be installed via HomeBrew::

    brew install libusb

On Windows-based systems, libusb releases are available at:

    https://github.com/libusb/libusb/releases

with more information available here:

    https://github.com/libusb/libusb/wiki/Windows#How_to_use_libusb_on_Windows

Since the XtalX sensor is plug-and-play compatible with the WinUSB driver, no
other special driver should be required for Windows compatibility.


xtalx_discover
==============

The xtalx package includes the xtalx_discover binary which can be used to list
all XtalX sensors that are attached to the system and their corresponding
firmware versions::

    ~$ xtalx_discover
    ******************
    Sensor SN: XTI-7-1000035
     git SHA1: 61be0469c1162b755d02fd9156a2754bebf24f59.dirty
       Version: 0x0107


xtalx_test
==========
The xtalx package includes a simple test binary that will connect to an XtalX
sensor and continuously print the current pressure and temperature reading::

    ~$ xtalx_test
    XtalX(XTI-7-1000035): 23.973375 PSI, 23.947930 C
    XtalX(XTI-7-1000035): 23.973375 PSI, 23.947930 C
    XtalX(XTI-7-1000035): 23.973375 PSI, 23.947930 C
    XtalX(XTI-7-1000035): 23.963872 PSI, 23.947930 C
    XtalX(XTI-7-1000035): 23.963872 PSI, 23.947930 C
    XtalX(XTI-7-1000035): 23.954370 PSI, 23.947930 C
    XtalX(XTI-7-1000035): 23.954370 PSI, 23.947930 C
    XtalX(XTI-7-1000035): 23.973375 PSI, 23.947930 C
    ...

Terminate the program by pressing Ctrl-C.
