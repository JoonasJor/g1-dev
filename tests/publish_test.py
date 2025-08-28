import time
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
import inspire.inspire_dds as inspire_dds
import inspire.inspire_defaults as inspire_defaults

if __name__ == '__main__':
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(1, "lo")

    pub_r = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
    pub_r.Init()

    pub_l = ChannelPublisher("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
    pub_l.Init()

    cmd = inspire_defaults.ctrl()
    cmd.mode = 0b0001

    cycle_duration = 5.0
    dt = 0.1
    t = 0

    cycle = False

    print("Setting joints to zero... ")
    for i in range(50):
        cmd.angle_set = [0, 0, 0, 0, 0, 0]
        
        pub_l.Write(cmd)
        pub_r.Write(cmd)
        time.sleep(dt)

    while True:
        if cycle:
            phase = (t % cycle_duration) / cycle_duration

            if phase < 0.5:
                value = int(phase * 2 * 1000)
            else:
                value = int((1 - phase) * 2 * 1000)

            cmd.angle_set = [value] * 6
        else:
            cmd.angle_set = [500] * 6

        pub_l.Write(cmd)
        pub_r.Write(cmd)

        print(cmd.angle_set)

        t += dt
        time.sleep(dt)
        

       



