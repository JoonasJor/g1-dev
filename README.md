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
2. Install unitree sdk  
https://github.com/unitreerobotics/unitree_sdk2_python  

## Run
### Mujoco sim:  
terminal 1:
```
python unitree/unitree_mujoco.py
```
terminal 2:
```
python demo_(body/hands).py
```
### Real robot:
```
python demo_(body/hands).py <network_interface_name>
```

