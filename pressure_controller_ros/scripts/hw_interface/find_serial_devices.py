#!/usr/bin/env python

from serial.tools import list_ports
import os

search = ['ttyUSB', 'ttyACM']

def find_port(search=None):
    all_port_tuples = list_ports.comports()
    all_ports = set()
    for ap, _, _ in all_port_tuples:
        p = os.path.basename(ap)
        if search is not None:
            for mstring in search:
                if p.startswith(mstring):
                    all_ports |= {ap}
        else:
            all_ports |= {ap}
    return list(all_ports)


all_ports = find_port(search)

if len(all_ports) >0:
    print("========================================")
    print('FOUND DEVICES WITH NAMES:')
    for string in search:
        print('    - %s'%(string))

    print("----------------------------------------")

    for dev in all_ports:
        print(dev)
else:
    print("")
    print("No valid port detected - Maybe it's unplugged?")