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
from camera.camera_bridge import CameraBridge

import config as Cfg
import g1_joints as Joints

locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(Cfg.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

"""
for i in range(mj_model.njnt):
    joint_type = mj_model.jnt_type[i]
    joint_name = mj_model.joint(i).name
    print(f"Joint {i}: {joint_name} — type: {joint_type}")
"""

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

if Cfg.LOCK_LOWER_BODY:
    lower_body_joints = Joints.Body.lower_body_joints()

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

                if Cfg.LOCK_LOWER_BODY:
                    for joint in lower_body_joints:
                        jnt_index = joint.mujoco_idx + 1  # offset by 1 for floating base joint

                        qpos_idx = int(mj_model.jnt_qposadr[jnt_index])

                        mj_data.qpos[qpos_idx] = joint.default_angle

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
    camera_bridge = CameraBridge(mj_model, mj_data, (640, 480), "head_camera")

    counter = 0

    print("[physics_viewer_thread] Starting viewer loop...")
    try:
        while viewer.is_running():
            locker.acquire()
            try:
                viewer.sync()

                if counter >= 10:
                    camera_bridge.capture_and_publish()
                    counter = 0
                counter += 1

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
