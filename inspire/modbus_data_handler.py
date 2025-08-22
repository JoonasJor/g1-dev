import numpy as np
import struct
import time
import threading
from pymodbus.client import ModbusTcpClient

import inspire.inspire_dds as inspire_dds
import inspire.inspire_defaults as inspire_defaults

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.utils.thread import RecurrentThread

import config as Cfg

class ModbusDataHandler:
    def __init__(self, history_length=100, ip=None, port=6000, device_id=1, l_r="r", max_retries=5, retry_delay=2):
        """_summary_
        Calling self.read() in a loop reads and returns the data, and publishes the DDS message at the same time        
        Args:
            history_length (int, optional): Hand state history_length. Defaults to 100.
            ip (str): ModbusTcp IP. Defaults are: left=192.168.123.210, right=192.168.123.211.
            port (int, optional): ModbusTcp IP port. Defaults to 6000.
            device_id (int, optional): Hand ID. Defaults to 1.
            max_retries (int, optional): Number of retries for connecting to Modbus server. Defaults to 3.
            retry_delay (int, optional): Delay between retries in seconds. Defaults to 2.
        Raises:
            ConnectionError: raise when connection fails after max_retries
        """      

        self.data = inspire_defaults.TOUCH_DATA
        self.history_length = history_length
        self.history = {
            "POS_ACT": [np.zeros(history_length) for _ in range(6)],
            "ANGLE_ACT": [np.zeros(history_length) for _ in range(6)],
            "FORCE_ACT": [np.zeros(history_length) for _ in range(6)],
            "CURRENT": [np.zeros(history_length) for _ in range(6)],
            "ERROR": [np.zeros(history_length) for _ in range(6)],
            "STATUS": [np.zeros(history_length) for _ in range(6)],
            "TEMP": [np.zeros(history_length) for _ in range(6)]
        }

        self.states_structure = [
            ("pos_act", 1534, 6, "short"),
            ("angle_act", 1546, 6, "short"),
            ("force_act", 1582, 6, "short"),
            ("current", 1594, 6, "short"),
            ("err", 1606, 3, "byte"),
            ("status", 1612, 3, "byte"),
            ("temperature", 1618, 3, "byte")
        ]

        self.client = ModbusTcpClient(ip, port=port)
        self.connect_to_modbus(max_retries, retry_delay)
        self.device_id = device_id
        self.modbus_lock = threading.Lock()

        self.client.write_register(1004, 1, self.device_id) #reset error

        self.l_r = l_r
        self.hand_state = inspire_defaults.state()
        self.hand_state_pub = ChannelPublisher(f"{Cfg.TOPIC_HAND_STATE}/{l_r}", inspire_dds.inspire_hand_state)
        self.hand_state_pub.Init()

        self.hand_touch = inspire_defaults.touch()
        self.hand_touch_pub = ChannelPublisher(f"{Cfg.TOPIC_HAND_TOUCH}/{l_r}", inspire_dds.inspire_hand_touch)
        self.hand_touch_pub.Init()

        self.sub = ChannelSubscriber(f"{Cfg.TOPIC_HAND_CMD}/{l_r}", inspire_dds.inspire_hand_ctrl)
        self.sub.Init(self.write_registers_callback, 10)

        self.register_read_thread = RecurrentThread(
            interval=0.002, target=self.read, name=f"register_read_{l_r}"
        )
        self.register_read_thread.Start()

    def connect_to_modbus(self, max_retries, retry_delay):
        retries = 0
        while retries < max_retries:
            try:
                if not self.client.connect():
                    raise ConnectionError("Failed to connect to Modbus server.")
                print("Modbus client connected successfully.")
                return
            except ConnectionError as e:
                print(f"Connection attempt {retries + 1} failed: {e}")
                retries += 1
                if retries < max_retries:
                    print(f"Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                else:
                    print("Max retries reached. Could not connect.")
                    raise

    def write_registers_callback(self, msg: inspire_dds.inspire_hand_ctrl):
        with self.modbus_lock:
            if msg.mode & 0b0001:  # mode 1 - angle
                self.client.write_registers(1486, msg.angle_set, self.device_id)
                # print("angle_set")
            if msg.mode & 0b0010:  # mode 2 - position
                self.client.write_registers(1474, msg.pos_set, self.device_id)
                # print("pos_set")

            if msg.mode & 0b0100:  # mode 4 - force
                self.client.write_registers(1498, msg.force_set, self.device_id)
                # print("force_set")

            if msg.mode & 0b1000:  # mode 8 - speed
                self.client.write_registers(1522, msg.speed_set, self.device_id)
                
    def read(self):
        touch_msg = inspire_defaults.touch()
        matrices = {}
        for i, (var, addr, length, size) in enumerate(self.data):
            value = self.read_and_parse_registers(addr, length // 2,"short")
            if value is not None:
                setattr(touch_msg, var, value)
                matrix = np.array(value).reshape(size)
                matrices[var]=matrix
        self.pub.Write(touch_msg)

        # Read the states for POS_ACT, ANGLE_ACT, etc.
        states_msg = inspire_defaults.state()

        for attr_name, start_address, length, data_type in self.states_structure:
            setattr(states_msg, attr_name, self.read_and_parse_registers(start_address, length, data_type))
            
        self.state_pub.Write(states_msg)

        return {
            "states":{
                "POS_ACT": states_msg.pos_act,
                "ANGLE_ACT": states_msg.angle_act,
                "FORCE_ACT": states_msg.force_act,
                "CURRENT": states_msg.current,
                "ERROR": states_msg.err,
                "STATUS": states_msg.status,
                "TEMP": states_msg.temperature
            },
            "touch": matrices
        }

    def read_and_parse_registers(self, start_address, num_registers, data_type="short"):
         with self.modbus_lock:
            # Read the register
            response = self.client.read_holding_registers(start_address, num_registers, self.device_id)

            if not response.isError():
                if data_type == "short":
                    # Pack the read register into binary data
                    packed_data = struct.pack(">" + "H" * num_registers, *response.registers)
                    # Unpack the register into a signed 16-bit integer (short)
                    angles = struct.unpack(">" + "h" * num_registers, packed_data)
                    return angles
                elif data_type == "byte":
                    # Split each 16-bit register into two 8-bit (uint8) values
                    byte_list = []
                    for reg in response.registers:
                        high_byte = (reg >> 8) & 0xFF  # High 8 bits
                        low_byte = reg & 0xFF          # Low 8 bits
                        byte_list.append(high_byte)
                        byte_list.append(low_byte)
                    return byte_list
            else:
                print("Error reading registers")
                return None
