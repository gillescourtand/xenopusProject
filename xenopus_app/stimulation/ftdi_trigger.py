# -*- coding: utf-8 -*-
"""
FTDI trigger helpers used to send a short hardware trigger pulse.

@author: Courtand, Kadri 
"""

import sys

from PyQt5 import QtWidgets
from pylibftdi import BitBangDevice, Driver


def get_ftdi_device_list():
    """
    Return the reference of the last detected FTDI device.

    The returned list follows the historical format used by the application:
    ``[vendor, product, serial]``. If no FTDI device is detected, an empty
    list is returned.
    """
    dev_list = []
    ref_list = []

    for device in Driver().list_devices():
        vendor, product, serial = device
        dev_list.append("%s:%s:%s" % (vendor, product, serial))
        print("ftdi : ", dev_list)
        ref_list = [vendor, product, serial]

    return ref_list


def trigger():
    """
    Send a short digital pulse through the detected FTDI device.

    The pulse is produced by briefly setting bit 1 on the FTDI port. Errors are
    displayed through a Qt warning dialog to keep the behavior visible from the
    GUI.
    """
    try:
        ftdi_ref_list = get_ftdi_device_list()

        if len(ftdi_ref_list) != 0:
            with BitBangDevice(ftdi_ref_list[1]) as bb:
                bb.direction = 0x0F
                bb.port |= 2
                bb.port &= 0xFE

    except sys.exc_info()[0] as e:
        print("error sys : ", e)
        QtWidgets.QMessageBox.warning(None, "Error", str(e))

    except IOError as e:
        print(e)
        QtWidgets.QMessageBox.warning(None, "Error", str(e))
