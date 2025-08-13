# g1-dev

## Installation

1. Setup environment
```
git clone https://github.com/JoonasJor/g1-dev.git
cd g1-dev
python3.8 -m venv venv
source venv/bin/activate
pip install mujoco pygame
```
2. Install SDKs  
https://github.com/unitreerobotics/unitree_sdk2_python  
https://github.com/NaCl-1374/inspire_hand_ws

## Run
Mujoco sim:
```
python sim2real.py
```
Real robot:
```
python sim2real.py <network_interface_name>
```

