xtalx
=====
This package provides a library for interfacing with the XtalX pressure and
density/viscosity sensors.  Python version 3 is required.  The easiest way to
install the xtalx module is using pip::

    python3 -m pip install xtalx

Note that you may wish to use sudo to install xtalx for all users on your
system::

    sudo python3 -m pip install xtalx

You may also install the package from the source using::

    make install

or::

    sudo make install


xtalx_p_discover
================
The xtalx package includes the xtalx_p_discover binary which can be used to
list all XtalX pressure sensors that are attached to the system and their
corresponding firmware versions::

    ~$ xtalx_p_discover
    ******************
    Sensor SN: XTI-7-1000035
     git SHA1: 61be0469c1162b755d02fd9156a2754bebf24f59.dirty
       Version: 0x0107


xtalx_p_test
============
The xtalx package includes a simple test binary that will connect to an XtalX
pressure sensor and continuously print the current pressure and temperature
reading::

    ~$ xtalx_p_test
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


xtalx_z_discover
================
The xtalx packages includes the xtalx_z_discover binary which can be used to
list all XtalX density/viscosity sensors that are attached to the system and
their corresponding firmware version::

    ~% xtalx_z_discover
    ******************
      Product: XtalX TCSC
    Sensor SN: TCSC-1-2000045
     git SHA1: b50b8e4a406f0b8585f50c1f9aa7c95145d4810d
      Version: 0x0101


xtalx_z_get_info
================
The xtalx package includes the xtalx_z_get_info binary that can be used to get
a bit more detailed information about a sensor::

    ~% xtalx_z_get_info
    [1691531122175214000] I: Controlling sensor TCSC-1-2000045 with firmware 0x101 (b50b8e4a406f0b8585f50c1f9aa7c95145d4810d).
    *************************************
         Serial Num: TCSC-1-2000045
           Git SHA1: b50b8e4a406f0b8585f50c1f9aa7c95145d4810d
         FW Version: 0x0101
               HCLK: 80000000
               DCLK: 2500000
               ACLK: 1250000
                HSE: 10 MHz
        Max cmd len: 16384
           # resets: 1
         Drive Type: DriveType.INTERNAL_DRIVE
       Reset Reason: ResetReason.POWER_ON_RESET
          Reset CSR: 0x0C004400
          Reset SR1: 0x00000000
    Max sweep freqs: 1024
    -----------------------
    Electrical calibration:
     Calibration Date: 1683682186 (Tue May  9 19:29:46 2023)
             R Source: 10037.000000
           R Feedback: 149850.000000
             DAC-to-V: 2.208075 + -0.000256*x
             ADC-to-V: 1.674066 + 0.000204*x
         Nominal VDDA: 3.35
    -----------------------
    Tuning fork calibration:
     Calibration Date: 1683682197 (Tue May  9 19:29:57 2023)
    Cal DAC Amplitude: 975 (0.25V)


xtalx_z_gl_track_mode
=====================
The xtalx package includes the xtalx_z_gl_track_mode binary that is the core
tool for use with the density/viscosity sensor.  It features a GUI instead of
command-line output.  To execute it::

    ~% xtalx_z_gl_track_mode


xtalx_z_cli_track_mode
======================
The xtalx package includes the xtalx_z_cli_track_mode binary that is similar
to the GUI track mode tool but only prints log messages to the console and
write measurements to the .csv files.  This allows it to be run on a headless
system.  To execute it::

    ~% xtalx_z_cli_track_mode


Windows use
===========
On Windows, the binaries are all launched via the command line just like on
Linux and macOS, however Windows can't automatically find them.  In order to
launch them, you need to execute them using their full Python path as follows::

    python3 -m xtalx.tools.p_sensor.discover
    python3 -m xtalx.tools.p_sensor.xtalx_test_yield
    python3 -m xtalx.tools.z_sensor.discover
    python3 -m xtalx.tools.z_sensor.get_info
    python3 -m xtalx.tools.z_sensor.gl_track_mode

On Windows, it is also common to have the Python interpreter installed under
the name "py" or "py3", so if "python3" does not work for you it is recommended
to try one of the shorter command names.  If the Python interpreter cannot be
found, it may not be installed at all so should be installed via the Microsoft
Store (it is a free, open-source install).
