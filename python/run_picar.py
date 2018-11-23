#!/usr/bin/python
# -*- coding: utf-8 -*-

from driver import driver
import time


def run_picar():
    print("==========piCar Client Start==========")
    d = driver()
    d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
    t = 10
    b = True

    # d.setStatus(...)
    # Motor control:
    # motor = ...
    # forward  0 -> 1.0
    # backward 0 -> -1.0
                
    # Servo control:
    # servo = ...
    # left = mid = right
    # 1.0  = 0.0 =  -1.0

    # Distance:
    # dist = ...
    # 0 -> 0xffffffff
    
    # Mode control:
    # 1:speed
    # 2:distance
    # 3:stop

    # d.getStatus(...)
    # query mode:
    # mode = any
    # sensor = any    
    try:
        print("Test 1/2: speed mode")
        d.setStatus(mode="speed")
        while True:
            st = t * 0.1 -1
            sm = st
            if abs(st) < 0.4:
                sm = 0.4 if st > 0 else -0.4
                if st == 0:
                    sm = 0
            d.setStatus(motor = sm, servo = st)
            print("Motor: %0.2f, Servo: %0.2f" % (sm, st))
            if b:
                t += 1
            else:
                t -= 1
            if t > 20:
                b = False
                t -= 2
            if t < 0:
                b = True
                t += 2
            time.sleep(1)
            # d.heartBeat()
            d.getStatus(sensor = 0, mode = 0)
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    try:
        print("Test 2/2: distance mode")
        d.setStatus(motor = 0.5, dist = 0x1ff00, mode="distance")
        while True:
            ss = t * 0.1 - 1
            d.setStatus(servo = ss)
            print("Servo: %0.2f" % ss)
            if b:
                t += 1
            else:
                t -= 1
            if t > 20:
                b = False
                t -= 2
            if t < 0:
                b = True
                t += 2
            time.sleep(1)
            d.getStatus(sensor = 0, mode = 0)
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
    d.close()
    del d
    print("==========piCar Client Fin==========")
    return 0
               
    
if __name__ == '__main__':
    run_picar()

