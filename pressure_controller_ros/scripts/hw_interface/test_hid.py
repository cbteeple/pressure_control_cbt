import hid

vendor_id     = 5824
product_id    = 1158
serial_number = 6467390

serial_number = str(serial_number)
found = None
for device_dict in hid.enumerate():
    vid = device_dict.get('vendor_id')
    pid = device_dict.get('product_id')
    snum = device_dict.get('serial_number')

    keys = list(device_dict.keys())
    keys.sort()
    for key in keys:
        print("%s : %s" % (key, device_dict[key]))
    print(" ")

    if (vid==vendor_id) & (pid==product_id) & (snum==serial_number):
        found = device_dict

if found is not None:
    print("=====================================")
    print('FOUND THE DEVICE YOU ARE LOOKING FOR:')
    print("-------------------------------------")
    keys = list(device_dict.keys())
    keys.sort()
    for key in keys:
        print("%s : %s" % (key, found[key]))