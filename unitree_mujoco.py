import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config


locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

if config.START_ON_FLOOR:
    mj_data.qpos[0:3] = config.FLOOR_POSITION
    mj_data.qpos[3:7] = config.FLOOR_ORIENTATION
    mj_data.qpos[7:60] = config.FLOOR_JOINT_ANGLES
    mj_data.qvel[:] = 0   
    mujoco.mj_step(mj_model, mj_data)

if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()
    if config.ROBOT == "h1" or config.ROBOT == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model

    if config.START_ON_FLOOR:
        print("[SimulationThread] Setting robot initial position...")


    try:
        print("[SimulationThread] Initializing Unitree SDK...")
        ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
        unitree = UnitreeSdk2Bridge(mj_model, mj_data)

        if config.USE_JOYSTICK:
            print("[SimulationThread] Setting up joystick...")
            unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)

        if config.PRINT_SCENE_INFORMATION:
            unitree.PrintSceneInformation()

        print("[SimulationThread] Starting simulation loop...")
        while viewer.is_running():
            step_start = time.perf_counter()

            locker.acquire()

            try:
                if config.ENABLE_ELASTIC_BAND:
                    # Check qpos and qvel lengths
                    if len(mj_data.qpos) < 3 or len(mj_data.qvel) < 3:
                        print(f"[SimulationThread] qpos or qvel too short: qpos={mj_data.qpos}, qvel={mj_data.qvel}")
                    else:
                        force = elastic_band.Advance(mj_data.qpos[:3], mj_data.qvel[:3])
                        if band_attached_link >= mj_data.xfrc_applied.shape[0]:
                            print(f"[SimulationThread] Invalid band_attached_link index: {band_attached_link}")
                        else:
                            mj_data.xfrc_applied[band_attached_link, :3] = force

                mujoco.mj_step(mj_model, mj_data)

            except Exception as e:
                print(f"[SimulationThread] Inner loop error: {type(e).__name__}: {e}")
                raise

            finally:
                locker.release()

            time_until_next_step = mj_model.opt.timestep - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    except Exception as e:
        print(f"[SimulationThread] Fatal error: {type(e).__name__}: {e}")
        raise



def PhysicsViewerThread():
    print("[PhysicsViewerThread] Starting viewer loop...")
    try:
        while viewer.is_running():
            locker.acquire()
            try:
                viewer.sync()
            except IndexError as e:
                print(f"[PhysicsViewerThread] IndexError during sync: {e}")
                raise
            except Exception as e:
                print(f"[PhysicsViewerThread] Unexpected error: {type(e).__name__}: {e}")
                raise
            finally:
                locker.release()

            time.sleep(config.VIEWER_DT)
    except Exception as e:
        print(f"[PhysicsViewerThread] Fatal error: {type(e).__name__}: {e}")
        raise


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
