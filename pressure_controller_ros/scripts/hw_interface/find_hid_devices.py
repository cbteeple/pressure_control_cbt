#!/usr/bin/env python

import hid

# Look for teensyduino "RawUSB" devices
manufacturer_strings = ['Teensyduino']

snums = []
found = []
for device_dict in hid.enumerate():
    vid = device_dict.get('vendor_id')
    pid = device_dict.get('product_id')
    snum = device_dict.get('serial_number')
    m_string = device_dict.get('manufacturer_string')

    keys = list(device_dict.keys())
    keys.sort()
    for key in keys:
        print("%s : %s" % (key, device_dict[key]))
    print(" ")

    if (m_string in manufacturer_strings):
        if snum in snums:
            continue

        else:
            found.append(device_dict)
            snums.append(snum)

if len(found) >0:
    print("========================================")
    print('FOUND DEVICES WITH MANUFACTURERS:')
    for string in manufacturer_strings:
        print('    - %s'%(string))
    
    for dev in found:
        print("----------------------------------------")
        keys = list(device_dict.keys())
        keys.sort()
        for key in keys:
            print("%s : %s" % (key, dev[key]))