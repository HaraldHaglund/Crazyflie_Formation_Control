#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
import numpy as np


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cfs = allcfs.crazyflies

    TIMESCALE = 1.0
    for cf in cfs:
        cf.takeoff(targetHeight=1.0, duration=3.0)
        timeHelper.sleep(1.0)
    
    timeHelper.sleep(5.0)
    for cf in cfs: 
        cf.land(targetHeight=0.02, duration=2.0)
        timeHelper.sleep(0.5)


if __name__ == '__main__':
    main()
