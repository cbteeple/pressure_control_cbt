#!/usr/bin/env python

import sys

from pressure_controller_setup.calibrate_valves import MyPlugin
from rqt_gui.main import Main

plugin = 'pressure_controller_setup.calibrate_valves'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))