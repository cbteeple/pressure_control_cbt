#!/usr/bin/env python

import sys

from pressure_calibrate.my_module import MyPlugin
from rqt_gui.main import Main

plugin = 'pressure_calibrate'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))