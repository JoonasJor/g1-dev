import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
import inspire_dds
import inspire_defaults

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
    short_value = 1000

    cmd.angle_set = [0, 0, 0, 0, 1000, 1000]
    cmd.mode = 0b0001
    pub_l.Write(cmd)
    pub_r.Write(cmd)

    time.sleep(1.0)

    cmd.angle_set = [0, 0, 0, 0, 0, 1000]
    cmd.mode = 0b0001
    pub_l.Write(cmd)
    pub_r.Write(cmd)

    time.sleep(3.0)

    for cnd in range(100000):
        start_address = 1486
        num_registers = 6

        if (cnd + 1) % 10 == 0:
            short_value = 1000 - short_value

        values_to_write = [short_value] * num_registers
        values_to_write[-1] = 1000 - values_to_write[-1]
        values_to_write[-2] = 1000 - values_to_write[-2]

        value_to_write_np = np.array(values_to_write)
        value_to_write_np = np.clip(value_to_write_np, 200, 800)

        cmd.angle_set = value_to_write_np.tolist()
        cmd.mode = 0b0001

        if pub_l.Write(cmd) and pub_r.Write(cmd):
            print("Publish success. msg:", cmd)
        else:
            print("Waiting for subscriber.")

        time.sleep(0.3)
