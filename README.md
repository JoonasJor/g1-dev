# g1-dev

## Installation
Requires python 3.9 - 3.10
1. Setup environment
```
git clone https://github.com/JoonasJor/g1-dev.git
cd g1-dev
python3.10 -m venv venv
source venv/bin/activate
pip install -e .
```
2. Install [Unitree SDK](https://github.com/unitreerobotics/unitree_sdk2_python  )
```
git submodule init
git submodule update
cd external/unitree_sdk2_python
pip install -e .
```

3. (Optional) For teleoperation install [xr_teleoperate](https://github.com/JoonasJor/xr_teleoperate)

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
### Physical deployment:
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

1. Terminal 1:  
   ```
   cd xr_teleoperate/teleop/  
   conda activate tv  
   python teleop_hand_and_arm.py --xr-mode=hand --arm=G1_29 --sim --headless --ee=inspire1 --disable-img-passthrough --motion
   ```
2. Terminal 2:
   ```
   cd g1-dev  
   source venv/bin/activate  
   python unitree/unitree_mujoco.py  
   ```  

3. Go to ```https://<host_ip>:8012?ws=wss://<host_ip>:8012``` and click "passthrough"  
4. Terminal 1: press "r"
   
### Physical deployment:
1. [Setup image service](https://github.com/JoonasJor/xr_teleoperate?tab=readme-ov-file#31-%EF%B8%8F-image-service)  
2. Terminal 1:
   ```
   cd xr_teleoperate/teleop/  
   conda activate tv 
   python xr_teleoperate/teleop/teleop_hand_and_arm.py --xr-mode=hand --arm=G1_29 --headless --ee=inspire1 --motion
   ```
3. Terminal 2:
   ```
   cd g1-dev  
   source venv/bin/activate  
   python inspire/modbus_data_handler.py
   ```
3. Go to ```https://<host_ip>:8012?ws=wss://<host_ip>:8012``` and click "VR" or "passthrough"
4. Terminal 1: press "r"



