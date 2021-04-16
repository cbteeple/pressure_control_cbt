#!/usr/bin/env python

import sys

from pressure_controller_setup.set_pressures_basic import MyPlugin
from rqt_gui.main import Main

plugin = 'pressure_controller_setup.set_pressures_basic'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))