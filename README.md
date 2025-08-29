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
