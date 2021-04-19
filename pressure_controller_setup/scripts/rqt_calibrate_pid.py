#!/usr/bin/env python

import sys

from pressure_controller_setup.calibrate_pid import MyPlugin
from rqt_gui.main import Main

plugin = 'pressure_controller_setup.calibrate_pid'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))