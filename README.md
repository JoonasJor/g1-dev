# g1-dev

## Installation
Requires python 3.8 - 3.10
1. Setup environment
```
git clone https://github.com/JoonasJor/g1-dev.git
cd g1-dev
python3.10 -m venv venv
source venv/bin/activate
pip install mujoco pygame pymodbus
```
2. Install [Unitree SDK](https://github.com/unitreerobotics/unitree_sdk2_python  )
```
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

## Run
### Mujoco sim:  
terminal 1:
```
python unitree/unitree_mujoco.py
```
terminal 2:
```
python demo_terminal.py
```
### Real robot:
1. Plug in ethernet cable
2. Change network interface ip to 192.168.123.99
3. Get network inferface name with ```ip addr```
4. Run ```python demo_terminal.py <network_interface_name>```

To use high-level control while simultaneously controlling upper body  
from hanging position:
1. Damping (L2+B) -> Locked Standing (L2+UP) -> Regular Mode (R1+X)

from squatting position:
1. Damping (L2+B) -> Squat-Stand (L2+A) -> Regular Mode (R1+X)

2. In demo_terminal.py try any example named "Arm: x"

## Teleoperation
### Mujoco sim:  
1. Install https://github.com/JoonasJor/xr_teleoperate
2. In terminal 1 run ```python g1-dev/unitree/unitree_mujoco.py```
3. In terminal 2 run ```python xr_teleoperate/teleop/teleop_hand_and_arm.py --xr-mode=hand --arm=G1_29 --sim --headless --ee=inspire1 --disable-img-passthrough --motion```
4. Go to ```https://<host_ip>:8012?ws=wss://<host_ip>:8012``` and press VR or passthrough
5. In terminal 2 press "r"
   
### Real robot:
1. Install https://github.com/JoonasJor/xr_teleoperate
2. In terminal 1 run ```python xr_teleoperate/teleop/teleop_hand_and_arm.py --xr-mode=hand --arm=G1_29 --headless --ee=inspire1 --motion```
3. Go to ```https://<host_ip>:8012?ws=wss://<host_ip>:8012``` and press VR or passthrough
4. In terminal 1 press "r"



