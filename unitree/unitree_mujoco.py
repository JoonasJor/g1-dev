import time
import mujoco
import mujoco.viewer
import threading
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree.unitree_bridge import UnitreeBridge, ElasticBand
from inspire.inspire_bridge import InspireBridge

import config as Cfg

locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(Cfg.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

if Cfg.START_ON_FLOOR:
    mj_data.qpos[0:3] = Cfg.FLOOR_POSITION
    mj_data.qpos[3:7] = Cfg.FLOOR_ORIENTATION
    mj_data.qpos[7:60] = Cfg.FLOOR_JOINT_ANGLES
    mj_data.qvel[:] = 0   
    mujoco.mj_step(mj_model, mj_data)

if Cfg.ENABLE_ELASTIC_BAND and not Cfg.START_ON_FLOOR:
    elastic_band = ElasticBand()
    if Cfg.ROBOT == "h1" or Cfg.ROBOT == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.mujoco_key_callback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = Cfg.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)

def simulation_thread():
    global mj_data, mj_model

    if Cfg.START_ON_FLOOR:
        print("simulation_thread Setting robot initial position...")


    try:
        print("simulation_thread Initializing Unitree SDK...")
        ChannelFactoryInitialize(Cfg.DOMAIN_ID, Cfg.INTERFACE)
        unitree = UnitreeBridge(mj_model, mj_data)

        inspire_r = InspireBridge(mj_model, mj_data, "r")
        inspire_l = InspireBridge(mj_model, mj_data, "l")

        if Cfg.USE_JOYSTICK:
            print("simulation_thread Setting up joystick...")
            unitree.setup_joystick(device_id=0, js_type=Cfg.JOYSTICK_TYPE)

        if Cfg.PRINT_SCENE_INFORMATION:
            unitree.print_scene_info()

        print("simulation_thread Starting simulation loop...")
        while viewer.is_running():
            step_start = time.perf_counter()

            locker.acquire()

            try:
                if Cfg.ENABLE_ELASTIC_BAND and not Cfg.START_ON_FLOOR:
                    # Check qpos and qvel lengths
                    if len(mj_data.qpos) < 3 or len(mj_data.qvel) < 3:
                        print(f"simulation_thread qpos or qvel too short: qpos={mj_data.qpos}, qvel={mj_data.qvel}")
                    else:
                        force = elastic_band.advance(mj_data.qpos[:3], mj_data.qvel[:3])
                        if band_attached_link >= mj_data.xfrc_applied.shape[0]:
                            print(f"simulation_thread Invalid band_attached_link index: {band_attached_link}")
                        else:
                            mj_data.xfrc_applied[band_attached_link, :3] = force

                mujoco.mj_step(mj_model, mj_data)

            except Exception as e:
                print(f"simulation_thread Inner loop error: {type(e).__name__}: {e}")
                raise

            finally:
                locker.release()

            time_until_next_step = mj_model.opt.timestep - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    except Exception as e:
        print(f"simulation_thread Fatal error: {type(e).__name__}: {e}")
        raise



def physics_viewer_thread():
    print("[physics_viewer_thread] Starting viewer loop...")
    try:
        while viewer.is_running():
            locker.acquire()
            try:
                viewer.sync()
            except IndexError as e:
                print(f"[physics_viewer_thread] IndexError during sync: {e}")
                raise
            except Exception as e:
                print(f"[physics_viewer_thread] Unexpected error: {type(e).__name__}: {e}")
                raise
            finally:
                locker.release()

            time.sleep(Cfg.VIEWER_DT)
    except Exception as e:
        print(f"[physics_viewer_thread] Fatal error: {type(e).__name__}: {e}")
        raise


if __name__ == "__main__":
    viewer_thread = threading.Thread(target=physics_viewer_thread)
    sim_thread = threading.Thread(target=simulation_thread)

    viewer_thread.start()
    sim_thread.start()
