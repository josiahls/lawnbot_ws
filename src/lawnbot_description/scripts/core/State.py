#!/usr/bin/env python
import numpy as np


class State:

    def __init__(self):
        self.odom_call_ready = False
        self.laser_call_ready = False
        self.state = np.zeros((1,1), float)

    def odom_callback(self):
        pass

    def laser_callback(self):
        pass

