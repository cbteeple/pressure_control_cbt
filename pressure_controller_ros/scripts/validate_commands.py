#! /usr/bin/env python
from __future__ import print_function

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import pressure_controller_ros.msg


import time
import sys
import os
import numbers



def go(command, args):

    # Validate inputs
    if not isinstance(command, str):
        raise ValueError('CONFIG: Command must be a string')

    if isinstance(args, list) or isinstance(args, tuple):
        pass
    elif isinstance(args, numbers.Number):
        args=[args]
    else:
        raise ValueError('CONFIG: Args must be a list, tuple, or number')

    return command, args




