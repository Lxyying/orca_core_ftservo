# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import os
import time
import math
import threading
from typing import Dict, List, Union
from collections import deque
from threading import RLock
import numpy as np
from .hardware.ftservo_client import DynamixelClient
from .hardware.mock_dynamixel_client import MockDynamixelClient
from .utils.utils import *

class OrcaHand:
    """OrcaHand class is used to abtract hardware control the hand of the robot with simple high level control methods in joint space."""
    """OrcaHand类用于抽象硬件控制，通过关节空间的高级控制方法控制灵巧手"""

    def __init__(self, model_path: str = None):
        """Initialize the OrcaHand class.

        Args:
            model_path (str): The path to model_path folder, which includes the config.yaml and calibration.yaml 
        """
        # Find the model directory if not provided
        self.model_path = get_model_path(model_path)
                
        # Load configurations from the YAML files
        self.config_path = os.path.join(self.model_path, "config.yaml")  #加载硬件配置
        self.calib_path = os.path.join(self.model_path, "calibration.yaml")  #加载校准参数
        
        config = read_yaml(self.config_path)
        calib = read_yaml(self.calib_path)
            
        self.baudrate: int = config.get('baudrate', 1000000)  #通信波特率
        self.port: str = config.get('port', 'COM9')  #串口
        self.max_current: int = config.get('max_current', 2500)  # 最大电流：2.5A（符合堵转电流规格）
        self.control_mode: str = config.get('control_mode', 'position')  #控制模式
        self.type: str = config.get('type', None)
        
        self.calib_current: str = config.get('calib_current', 650)  #校准电流：设置为额定电流650mA
        self.wrist_calib_current: str = config.get('wrist_calib_current', 500)
        self.calib_step_size: float = config.get('calib_step_size', 0.1)  #校准步长
        self.calib_step_period: float = config.get('calib_step_period', 0.01) #手腕校准电流：500mA（安全值）
        self.calib_threshold: float = config.get('calib_threshold', 0.01)  #校准阈值
        self.calib_num_stable: int = config.get('calib_num_stable', 20)
        self.calib_sequence: Dict[str, Dict[str, str]] = config.get('calib_sequence', [])
        self.calibrated: bool = calib.get('calibrated', False)
     
        self.neutral_position: Dict[str, float] = config.get('neutral_position', {})
        
        self.motor_ids: List[int] = config.get('motor_ids', [])
        self.joint_ids: List[str] = config.get('joint_ids', [])
        self.motor_id_to_idx_dict: Dict[int, int] = {motor_id: i for i, motor_id in enumerate(self.motor_ids)}

        motor_limits_from_calib_dict = calib.get('motor_limits', {})
        self.motor_limits_dict: Dict[int, List[float]] = {
            motor_id: motor_limits_from_calib_dict.get(motor_id, [None, None]) for motor_id in self.motor_ids}

        joint_to_motor_ratios_from_calib_dict = calib.get('joint_to_motor_ratios', {})
        self.joint_to_motor_ratios_dict: Dict[int, float] = {
            motor_id: joint_to_motor_ratios_from_calib_dict.get(motor_id, 0.0) for motor_id in self.motor_ids}
            
        self.joint_to_motor_map: Dict[str, float] = config.get('joint_to_motor_map', {})
        self.joint_roms_dict: Dict[str, List[float]] = config.get('joint_roms', {})
        
        self.joint_inversion_dict = {}
        for joint, motor_id in self.joint_to_motor_map.items():
            if motor_id < 0 or math.copysign(1, motor_id) < 0:
                self.joint_inversion_dict[joint] = True
                self.joint_to_motor_map[joint] = int(abs(motor_id))
            else:
                self.joint_inversion_dict[joint] = False
        
        self.joint_to_motor_map = {k: int(v) for k, v in self.joint_to_motor_map.items()} # This is to make IDs integers

        self.motor_to_joint_dict: Dict[int, str] = {v: k for k, v in self.joint_to_motor_map.items()}

        self._wrap_offsets_dict: Dict[int, float] = None

        self._dxl_client: DynamixelClient = None
        self._motor_lock: RLock = RLock()

        # Task thread to start and stop longer tasks like tensioning, calibration, etc. externally
        self._task_thread: threading.Thread = None
        self._task_stop_event = threading.Event()
        self._lock = threading.Lock() 
        self._current_task = None
        
        self._sanity_check()       
        self.is_calibrated(verbose=True)

    def __del__(self):
        """Destructor to disconnect from the hand."""
        self.disconnect()
        
    def connect(self) -> tuple[bool, str]:
        """Connect to the hand with the DynamixelClient.  #创建DynamixelClient并连接硬件

        Returns:
            tuple[bool, str]: (Success status, message).
        """
        try:
            #创建Dynamixel客户端实例
            self._dxl_client = DynamixelClient(self.motor_ids, self.port, self.baudrate)
            with self._motor_lock:
                self._dxl_client.connect()
            return True, "Connection successful"
        except Exception as e:
            self._dxl_client = None
            return False, f"Connection failed: {str(e)}"
        
    def disconnect(self) -> tuple[bool, str]:
        """Disconnect from the hand.    #断开连接并禁用扭矩

        Returns:
            tuple[bool, str]: (Success status, message).
        """
        try:
            with self._motor_lock:
                self.disable_torque()
                time.sleep(0.1)
                self._dxl_client.disconnect()
            return True, "Disconnected successfully"
        except Exception as e:
            return False, f"Disconnection failed: {str(e)}"
        
    def is_connected(self) -> bool:
        """Check if the hand is connected.

        Returns:
            bool: True if connected, False otherwise.
        """
        return self._dxl_client.is_connected if self._dxl_client else False

    def enable_torque(self, motor_ids: List[int] = None):
        """Enable torque for the motors."""
        if motor_ids is None:
            motor_ids = self.motor_ids

        with self._motor_lock:
            for motor_id in motor_ids:
                # 检查是否为模拟客户端
                if hasattr(self._dxl_client, '_connected') and self._dxl_client._connected:
                    # 模拟客户端，直接设置状态
                    self._dxl_client._torque_enabled[motor_id] = True
                else:
                    # 真实客户端，使用协议通信
                    result, error = self._dxl_client.protocol.write1ByteTxRx(motor_id, 40, 1)
                    if result != 0:
                        print(f"Failed to enable torque for motor {motor_id}")

    def disable_torque(self, motor_ids: List[int] = None):
        """Disable torque for the motors."""
        if motor_ids is None:
            motor_ids = self.motor_ids

        with self._motor_lock:
            for motor_id in motor_ids:
                # 检查是否为模拟客户端
                if hasattr(self._dxl_client, '_connected') and self._dxl_client._connected:
                    # 模拟客户端，直接设置状态
                    self._dxl_client._torque_enabled[motor_id] = False
                else:
                    # 真实客户端，使用协议通信
                    result, error = self._dxl_client.protocol.write1ByteTxRx(motor_id, 40, 0)
                    if result != 0:
                        print(f"Failed to disable torque for motor {motor_id}")

    def calibrate_current_position(self, motor_ids: List[int] = None): #原来没有此函数，飞特舵机独有
        """将任意当前位置校正为2048（中间位置）

        Args:
            motor_ids (list): 要校正的电机ID列表，如果为None则校正所有电机
        """
        if motor_ids is None:
            motor_ids = self.motor_ids

        with self._motor_lock:
            for motor_id in motor_ids:
                # 飞特舵机：写128到地址40进行当前位置校正
                result, error = self._dxl_client.protocol.write1ByteTxRx(motor_id, 40, 128)
                if result != 0:
                    print(f"Failed to calibrate current position for motor {motor_id}")
                else:
                    print(f"Motor {motor_id} current position calibrated to 2048")

    def set_max_current(self, current: Union[float, List[float]]):
        """设置最大扭矩限制（替代电流控制）

        Args:
            current (int or list): 如果为列表，则是每个电机的最大电流值，否则所有电机使用相同值
        """
        # 飞特舵机通过转矩限制控制输出，范围0-1000（对应0-100%）
        if isinstance(current, list):
            if len(current) != len(self.motor_ids):
                raise ValueError("Number of currents do not match the number of motors.")
            torque_limits = [int(c * 1000 / self.max_current) for c in current]
            with self._motor_lock:
                for motor_id, torque in zip(self.motor_ids, torque_limits):
                    self._dxl_client.set_torque_limit([motor_id], torque)
        else:
            torque_limit = int(current * 1000 / self.max_current)
            with self._motor_lock:
                self._dxl_client.set_torque_limit(self.motor_ids, torque_limit)

    def set_torque_hold_mode(self, motor_ids: List[int] = None,
                             torque_limit: int = 300,
                             position_tolerance: float = 0.01):
        """设置扭矩保持模式，在当前位置保持设定的扭矩

        Args:
            motor_ids: 要设置的电机ID列表，None表示所有电机
            torque_limit: 扭矩限制值 (0-1000, 1000=100%)
            position_tolerance: 位置容差，用于检测是否到达目标位置
        """
        if motor_ids is None:
            motor_ids = self.motor_ids

        with self._motor_lock:
            # 1. 获取当前位置作为保持位置
            current_positions = self.get_motor_pos()
            hold_positions = {}
            for motor_id in motor_ids:
                idx = self.motor_id_to_idx_dict[motor_id]
                hold_positions[motor_id] = current_positions[idx]

            # 2. 设置扭矩限制
            self.set_torque_limit_direct(motor_ids, torque_limit)

            # 3. 设置目标位置为当前位置
            self._dxl_client.write_desired_pos(motor_ids,
                                               [hold_positions[motor_id] for motor_id in motor_ids])

            # 4. 等待到达目标位置
            self._wait_for_position(motor_ids, hold_positions, position_tolerance, timeout=2.0)

            print(f"Torque hold mode activated for motors {motor_ids} with torque limit {torque_limit}")

    def set_torque_limit_direct(self, motor_ids: List[int], torque_limit: int):
        """直接设置扭矩限制值

        Args:
            motor_ids: 电机ID列表
            torque_limit: 扭矩限制值 (0-1000)
        """
        if not (0 <= torque_limit <= 1000):
            raise ValueError("Torque limit must be between 0 and 1000")

        with self._motor_lock:
            for motor_id in motor_ids:
                result, error = self._dxl_client.protocol.write2ByteTxRx(motor_id, 48, torque_limit)
                if result != 0:
                    print(f"Failed to set torque limit for motor {motor_id}")

    def get_torque_limit(self, motor_ids: List[int] = None) -> Dict[int, int]:
        """读取当前扭矩限制值

        Args:
            motor_ids: 电机ID列表，None表示所有电机

        Returns:
            字典 {motor_id: torque_limit}
        """
        if motor_ids is None:
            motor_ids = self.motor_ids

        torque_limits = {}
        with self._motor_lock:
            for motor_id in motor_ids:
                torque_value, result, error = self._dxl_client.protocol.read2ByteTxRx(motor_id, 48)
                if result == 0:
                    torque_limits[motor_id] = torque_value
                else:
                    torque_limits[motor_id] = 0
                    print(f"Failed to read torque limit for motor {motor_id}")

        return torque_limits

    def _wait_for_position(self, motor_ids: List[int], target_positions: Dict[int, float],
                           tolerance: float = 0.01, timeout: float = 5.0):
        """等待电机到达目标位置

        Args:
            motor_ids: 电机ID列表
            target_positions: 目标位置字典 {motor_id: position}
            tolerance: 位置容差
            timeout: 超时时间
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_positions = self.get_motor_pos()
            all_reached = True

            for motor_id in motor_ids:
                idx = self.motor_id_to_idx_dict[motor_id]
                current_pos = current_positions[idx]
                target_pos = target_positions[motor_id]

                if abs(current_pos - target_pos) > tolerance:
                    all_reached = False
                    break

            if all_reached:
                return True

            time.sleep(0.01)

        print("Warning: Timeout waiting for position reach")
        return False

    def set_adaptive_torque_hold(self, motor_ids: List[int] = None,
                                 base_torque: int = 200,
                                 max_torque: int = 800,
                                 duration: float = 60.0,
                                 temperature_limit: float = 60.0):
        """自适应扭矩保持，带温度监控

        Args:
            motor_ids: 电机ID列表
            base_torque: 基础扭矩值
            max_torque: 最大扭矩值
            duration: 保持时间(秒)
            temperature_limit: 温度限制(摄氏度)
        """
        if motor_ids is None:
            motor_ids = self.motor_ids

        def adaptive_hold():
            start_time = time.time()
            current_torque = base_torque

            while time.time() - start_time < duration and not self._task_stop_event.is_set():
                # 检查温度
                temps = self.get_motor_temp(as_dict=True)
                max_temp = max(temps.values())

                if max_temp > temperature_limit:
                    # 温度过高，降低扭矩
                    current_torque = max(100, current_torque - 100)
                    print(f"Temperature {max_temp}°C too high, reducing torque to {current_torque}")
                elif max_temp < temperature_limit - 10 and current_torque < max_torque:
                    # 温度正常，可以适当增加扭矩
                    current_torque = min(max_torque, current_torque + 50)

                # 更新扭矩限制
                self.set_torque_limit_direct(motor_ids, current_torque)

                # 检查错误状态
                errors = self.check_motor_errors(motor_ids)
                if any(len(error_list) > 1 or "No errors" not in error_list for error_list in errors.values()):
                    print("Motor error detected, stopping torque hold")
                    break

                time.sleep(1.0)  # 每秒检查一次

        # 在后台线程中运行
        self._start_task(adaptive_hold)

    def set_motor_positions_direct(self, motor_positions: Dict[int, float]):
        """直接设置电机位置，绕过所有关节转换逻辑

        Args:
            motor_positions: 字典 {motor_id: 目标位置(弧度)}
        """
        if not self.is_connected():
            raise RuntimeError("Hand is not connected.")

        motor_ids = []
        positions = []

        for motor_id, position in motor_positions.items():
            if motor_id not in self.motor_ids:
                print(f"Warning: Motor ID {motor_id} is not in configured motor IDs. Skipping.")
                continue

            motor_ids.append(motor_id)
            positions.append(float(position))  # 确保转换为浮点数

        if motor_ids:
            # 直接使用现有的硬件控制功能，不进行任何关节转换
            # 确保 positions 是 numpy 数组且类型正确
            positions_array = np.array(positions, dtype=np.float64)
            self._dxl_client.write_desired_pos(motor_ids, positions_array)
            return True
        return False

    def get_motor_positions_dict(self) -> Dict[int, float]:
        """获取电机位置字典形式"""
        motor_pos_array = self.get_motor_pos()
        return {self.motor_ids[i]: motor_pos_array[i] for i in range(len(self.motor_ids))}

    def set_control_mode(self, mode: str, motor_ids: List[int] = None):
        """Set the control mode for the motors.
        
        Args:
            mode (str): Control mode.
                (0) current: Current control mode,
                (1) velocity: Velocity control mode,
                (3) position: Position control mode,
                (4) multi_turn_position: Multi-turn position control mode,
                (5) current_based_position: Current-based position control mode.
            motor_ids (list): List of motor IDs to set the control mode. If None, all motors will be set.
        """
        
        mode_map = {
            'current': None,  #飞特不支持直接电流控制
            'velocity': 1,  #电机恒速模式
            'position': 0,  #位置伺服模式
            'multi_turn_position': 0,  #位置模式+特殊配置实现多圈
            'current_based_position': 0,  #用模式0替代（注意：飞特的多圈需要特殊配置，但这里我们先用模式0，并通过软件包装处理）
            'pwm_velocity': 2,  # 新增（原orcahand没有）：模式2：速度开环电机模式（PWM调速度）
            'step_servo': 3  # 新增：模式3：步进模式
        }

        mode = mode_map.get(mode)
        if mode is None:
            raise ValueError(f"Control mode '{mode}' not supported by Feite servos.")
        
        with self._motor_lock:
            if motor_ids is None:
                motor_ids = self.motor_ids
            else:
                if not all(motor_id in self.motor_ids for motor_id in motor_ids):
                    raise ValueError("Invalid motor IDs.")
            self._dxl_client.set_operating_mode(motor_ids, mode)
            
    def get_motor_pos(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Get the current motor positions in radians (Note that this includes offsets of the motors).
        
        Args:
            as_dict (bool): If True, return the motor positions as a dictionary with motor IDs as keys.
                           If False, return as numpy array.
        
        Returns:
            Union[np.ndarray, dict]: Motor positions either as numpy array or dictionary {motor_id: position}.
        """
        with self._motor_lock:
            motor_pos = self._dxl_client.read_pos_vel_cur()[0]
            if as_dict:
                return {motor_id: pos for motor_id, pos in zip(self.motor_ids, motor_pos)}
            return motor_pos
        
    def get_motor_current(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Get the current motor currents in mA.
        
        Args:
            as_dict (bool): If True, return the motor currents as a dictionary with motor IDs as keys.
                           If False, return as numpy array.
        
        Returns:
            Union[np.ndarray, dict]: Motor currents either as numpy array or dictionary {motor_id: current}.
        """
        with self._motor_lock:
            motor_current = self._dxl_client.read_pos_vel_cur()[2]
            if as_dict:
                return {motor_id: current for motor_id, current in zip(self.motor_ids, motor_current)}
            return motor_current

    def get_motor_temp(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Get the current motor temperatures in Celsius.

        Args:
            as_dict (bool): If True, return the motor temperatures as a dictionary with motor IDs as keys.
                           If False, return as numpy array.

        Returns:
            Union[np.ndarray, dict]: Motor temperatures either as numpy array or dictionary {motor_id: temperature}.
        """
        with self._motor_lock:
            # 飞特舵机温度读取地址为63，单位°C
            temp_data = np.zeros(len(self.motor_ids), dtype=np.float32)
            for i, motor_id in enumerate(self.motor_ids):
                temp_value, result, error = self._dxl_client.protocol.read1ByteTxRx(motor_id, 63)
                if result == 0:
                    temp_data[i] = float(temp_value)
                else:
                    temp_data[i] = 0.0
                    print(f"Failed to read temperature for motor {motor_id}")

            if as_dict:
                return {motor_id: temp for motor_id, temp in zip(self.motor_ids, temp_data)}
            return temp_data

    def get_motor_status(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Get the current motor status (error states).

        Args:
            as_dict (bool): If True, return the motor status as a dictionary with motor IDs as keys.
                           If False, return as numpy array.

        Returns:
            Union[np.ndarray, dict]: Motor status either as numpy array or dictionary {motor_id: status}.
        """
        with self._motor_lock:
            # 飞特舵机状态读取地址为65，1字节，用于读取飞特舵机的错误状态
            status_data = np.zeros(len(self.motor_ids), dtype=np.int32)
            for i, motor_id in enumerate(self.motor_ids):
                status_value, result, error = self._dxl_client.protocol.read1ByteTxRx(motor_id, 65)
                if result == 0:
                    status_data[i] = int(status_value)
                else:
                    status_data[i] = 0
                    print(f"Failed to read status for motor {motor_id}")

            if as_dict:
                return {motor_id: status for motor_id, status in zip(self.motor_ids, status_data)}
            return status_data

    def get_motor_moving_status(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Get the current motor moving status.

        Args:
            as_dict (bool): If True, return the moving status as a dictionary with motor IDs as keys.
                           If False, return as numpy array.

        Returns:
            Union[np.ndarray, dict]: Moving status either as numpy array or dictionary {motor_id: moving_status}.
        """
        with self._motor_lock:
            # 飞特舵机移动状态读取地址为66，1字节，用于检测舵机是否正在运动
            moving_data = np.zeros(len(self.motor_ids), dtype=np.int32)
            for i, motor_id in enumerate(self.motor_ids):
                moving_value, result, error = self._dxl_client.protocol.read1ByteTxRx(motor_id, 66)
                if result == 0:
                    moving_data[i] = int(moving_value)
                else:
                    moving_data[i] = 0
                    print(f"Failed to read moving status for motor {motor_id}")

            if as_dict:
                return {motor_id: moving for motor_id, moving in zip(self.motor_ids, moving_data)}
            return moving_data

    def parse_motor_status(self, status_value: int) -> List[str]:
        """Parse motor status value to get error descriptions.

        Args:
            status_value (int): The status byte value from address 65.

        Returns:
            List[str]: List of error descriptions.
        """
        errors = []

        # 飞特舵机状态字节位映射
        if status_value & 0x01:  # BIT0: 电压状态
            errors.append("Voltage error (over/under voltage)")
        if status_value & 0x02:  # BIT1: 磁编码状态
            errors.append("Magnetic encoder error")
        if status_value & 0x04:  # BIT2: 温度状态
            errors.append("Temperature error (overheat)")
        if status_value & 0x08:  # BIT3: 电流状态
            errors.append("Current error (overcurrent)")
        if status_value & 0x20:  # BIT5: 负载状态
            errors.append("Load error (overload)")

        return errors if errors else ["No errors"]

    def check_motor_errors(self, motor_ids: List[int] = None) -> Dict[int, List[str]]:
        """Check for errors on specified motors.

        Args:
            motor_ids (list): List of motor IDs to check. If None, check all motors.

        Returns:
            Dict[int, List[str]]: Dictionary mapping motor IDs to their error lists.
        """
        if motor_ids is None:
            motor_ids = self.motor_ids

        status_dict = self.get_motor_status(as_dict=True)
        error_report = {}

        for motor_id in motor_ids:
            if motor_id in status_dict:
                error_list = self.parse_motor_status(status_dict[motor_id])
                error_report[motor_id] = error_list
                if len(error_list) > 1 or "No errors" not in error_list:
                    print(f"Motor {motor_id} errors: {', '.join(error_list)}")

        return error_report

    def get_joint_pos(self, as_list: bool = True) -> Union[dict, list]:
        """Get the current joint positions.
    
        Args:
            as_list (bool): If True, return the joint positions as a list in the order of joint_ids.
                            If False, return the joint positions as a dictionary.
    
        Returns:
            Union[dict, list]: Joint positions as a list [position1, position2, ...] in the order of joint_ids
                               or as a dictionary {joint_name: position}.
        """
        motor_pos = self.get_motor_pos()
        joint_pos = self._motor_to_joint_pos(motor_pos)
        if as_list:
            return [joint_pos[joint] for joint in self.joint_ids]
    
        return joint_pos
         
    def set_joint_pos(self, joint_pos: Union[dict, list], num_steps: int = 1, step_size: float = 1.0):
        """Set the desired joint positions. If nun_steps > 1, the hand will move to the target position in a smooth, gradual motion (depending also on step_size).
    
        Args:
            joint_pos (dict or list): If dict, it should be {joint_name: desired_position}.
                                    If list, it should contain positions in the order of joint_ids.
            num_steps (int): Number of steps to reach the target position. If 1, moves directly to target.
            step_size (float): Time to wait between steps in seconds.
        """
        
        if num_steps > 1:
            current_positions = self.get_joint_pos(as_list=False)
            
            if isinstance(joint_pos, list):
                if len(joint_pos) != len(self.joint_ids):
                    raise ValueError("Length of joint_pos list must match the number of joint_ids.")
                target_positions = {joint: pos for joint, pos in zip(self.joint_ids, joint_pos)}
            else:
                target_positions = joint_pos.copy()
            
            for step in range(num_steps + 1):
                t = step / num_steps
                
                interpolated_positions = {}
                for joint in self.joint_ids:
                    if joint in target_positions:
                        current_pos = current_positions[joint]
                        if current_pos is None or target_positions[joint] is None:
                            interpolated_positions[joint] = None
                            continue
                        target_pos = target_positions[joint]
                        interpolated_positions[joint] = current_pos * (1 - t) + target_pos * t
                    else:
                        interpolated_positions[joint] = current_positions[joint]
                
                motor_pos = self._joint_to_motor_pos(interpolated_positions)
                                
                self._set_motor_pos(motor_pos)
                if step < num_steps: 
                    time.sleep(step_size)
        else:
            if isinstance(joint_pos, dict):
                motor_pos = self._joint_to_motor_pos(joint_pos)
            elif isinstance(joint_pos, list):
                if len(joint_pos) != len(self.joint_ids):
                    raise ValueError("Length of joint_pos list must match the number of joint_ids.")
                joint_pos_dict = {joint: pos for joint, pos in zip(self.joint_ids, joint_pos)}
                motor_pos = self._joint_to_motor_pos(joint_pos_dict)
            else:
                raise ValueError("joint_pos must be a dict or a list.")

            self._set_motor_pos(motor_pos)

    def set_zero_position(self, num_steps: int = 25, step_size: float = 0.001):
        """Set the hand to the zero position by moving all joints simultaneously to their zero positions
        in a smooth, gradual motion.
        
        Args:
            num_steps (int): Number of steps to reach the zero position.
            step_size (float): Step size for each joint.
        """
        self.set_joint_pos({joint: 0 for joint in self.joint_ids}, num_steps=num_steps, step_size=step_size)
        
    def set_neutral_position(self, num_steps: int = 25, step_size: float = 0.001):
        """Set the hand to the neutral position by moving all joints simultaneously to their neutral positions
        in a smooth, gradual motion.

        Args:
            num_steps (int): Number of steps to reach the neutral position.
            step_size (float): Step size for each joint.
        """
        if self.neutral_position is None:
            raise ValueError("Neutral position is not set. Please set the neutral position in the config.yaml file.")
        self.set_joint_pos(self.neutral_position, num_steps=num_steps, step_size=step_size)
        
    def init_joints(self, calibrate: bool = False
                    ):
        """Initialize the joints, enables torque, sets the control mode and sets to the zero position.
        If the hand is not calibrated, it will calibrate the hand. 
        
        Args:
            calibrate (bool): If True, the hand will be calibrated
        
        """
        self.enable_torque()
        self.set_control_mode(self.control_mode)
        self.set_max_current(self.max_current)
        
        if not self.calibrated or calibrate:
            self.calibrate()
   
        self._compute_wrap_offsets_dict()
        self.set_joint_pos(self.neutral_position)

    def is_calibrated(self, verbose: bool = False) -> bool:
        """Check if the hand is calibrated.

        Args:
            verbose (bool): If True, print detailed calibration status for uncalibrated joints.

        Returns:
            bool: True if all joints are calibrated, False otherwise.
        """
        overall_calibrated = True
        uncalibrated_messages = []
        motors_with_warnings = set()

        for motor_id, limits in self.motor_limits_dict.items():
            if any(limit is None for limit in limits):
                overall_calibrated = False
                if not verbose:
                    return False 
                joint_name = self.motor_to_joint_dict.get(motor_id, "Unknown")
                # Corrected escape sequence
                uncalibrated_messages.append(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing motor limits).\033[0m")
                motors_with_warnings.add(motor_id)


        for motor_id, ratio in self.joint_to_motor_ratios_dict.items():
            if ratio is None or ratio == 0.0:
                overall_calibrated = False
                if not verbose:
                    return False
                if motor_id not in motors_with_warnings:
                    joint_name = self.motor_to_joint_dict.get(motor_id, "Unknown")
                    uncalibrated_messages.append(
                        f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m"
                    )
                    motors_with_warnings.add(motor_id)
        
        if verbose:
            for msg in uncalibrated_messages:
                print(msg)
        
        return overall_calibrated

    def calibrate(self, blocking: bool = True):
        if blocking:
            self._calibrate()
        else:
            self._start_task(self._calibrate)

    def _calibrate(self):
            
        # Store the min and max values for each motor
        motor_limits = self.motor_limits_dict.copy()

        self._compute_wrap_offsets_dict()
        for step in self.calib_sequence:
            for joint in step["joints"].keys():
                motor_id = self.joint_to_motor_map[joint]
                motor_limits[motor_id] = [None, None]
                self._wrap_offsets_dict[motor_id] = 0.0

        # Set calibration control mode
        self.set_control_mode('position')
        self.set_max_current(self.calib_current)
        self.enable_torque()
        
        for step in self.calib_sequence:
            if self._task_stop_event.is_set():
                return

            desired_increment, motor_reached_limit, directions, position_buffers, motor_reached_limit, calibrated_joints, position_logs, current_log = {}, {}, {}, {}, {}, {}, {}, {}

            for joint, direction in step["joints"].items(): 
                if self._task_stop_event.is_set():
                    return

                if joint == 'wrist':
                    self.set_max_current(self.wrist_calib_current)
                else:
                    self.set_max_current(self.calib_current)
                    
                motor_id = self.joint_to_motor_map[joint]
                sign = 1 if direction == 'flex' else -1
                if self.joint_inversion_dict.get(joint, False):
                    sign = -sign
                directions[motor_id] = sign
                position_buffers[motor_id] = deque(maxlen=self.calib_num_stable)
                position_logs[motor_id] = []
                current_log[motor_id] = []
                motor_reached_limit[motor_id] = False
            
            while(not all(motor_reached_limit.values()) and not self._task_stop_event.is_set()):               
                for motor_id, reached_limit in motor_reached_limit.items():
                    if not reached_limit:
                        desired_increment[motor_id] = directions[motor_id] * self.calib_step_size

                self._set_motor_pos(desired_increment, rel_to_current=True)
                time.sleep(self.calib_step_period)
                curr_pos = self.get_motor_pos()
                
                for motor_id in desired_increment.keys():
                    if not motor_reached_limit[motor_id]:
                        position_buffers[motor_id].append(curr_pos[self.motor_id_to_idx_dict[motor_id]])
                        position_logs[motor_id].append(float(curr_pos[self.motor_id_to_idx_dict[motor_id]]))
                        current_log[motor_id].append(float(self.get_motor_current()[self.motor_id_to_idx_dict[motor_id]]))

                        # Check if buffer is full and all values are close
                        if len(position_buffers[motor_id]) == self.calib_num_stable and np.allclose(position_buffers[motor_id], position_buffers[motor_id][0], atol=self.calib_threshold):
                            motor_reached_limit[motor_id] = True
                            # disable torque for the motor
                            if 'wrist' in joint or 'abd' in joint:
                                avg_limit = float(np.mean(position_buffers[motor_id]))
                            else:
                                self.disable_torque([motor_id])
                                time.sleep(0.05)
                                avg_limit = float(self.get_motor_pos()[self.motor_id_to_idx_dict[motor_id]])
                            print(f"Motor {motor_id} corresponding to joint {self.motor_to_joint_dict[motor_id]} reached the limit at {avg_limit} rad.")
                            if directions[motor_id] == 1:
                                motor_limits[motor_id][1] = avg_limit
                            if directions[motor_id] == -1:
                                motor_limits[motor_id][0] = avg_limit
                            self.enable_torque([motor_id])
                
            # find ratios of all motors that have been calibrated in this step
            for joint, direction in step["joints"].items(): 
                motor_id = self.joint_to_motor_map[joint]
                if motor_limits[motor_id][0] is None or motor_limits[motor_id][1] is None:
                    continue
                delta_motor = motor_limits[motor_id][1] - motor_limits[motor_id][0]
                delta_joint = self.joint_roms_dict[self.motor_to_joint_dict[motor_id]][1] - self.joint_roms_dict[self.motor_to_joint_dict[motor_id]][0]
                self.joint_to_motor_ratios_dict[motor_id] = float(delta_motor / delta_joint) 
                print("Joint calibrated: ", joint)
                calibrated_joints[joint] = 0.0
  
            update_yaml(self.calib_path, 'joint_to_motor_ratios', self.joint_to_motor_ratios_dict)
            update_yaml(self.calib_path, 'motor_limits', motor_limits)
            self.motor_limits_dict = motor_limits
            if calibrated_joints:
                self.set_joint_pos(calibrated_joints, num_steps=25, step_size=0.001)
            time.sleep(0.1)    
            
        self.calibrated = self.is_calibrated()
        update_yaml(self.calib_path, 'calibrated', self.calibrated)
        self.set_joint_pos(calibrated_joints, num_steps=25, step_size=0.001)
        self.set_max_current(self.max_current)
       
    def calibrate_manual(self):
        
        raise NotImplementedError("Manual calibration is not implemented yet. Please use the automatic calibration method.")
        
        self.disable_torque()

        calibrated_joints = {}
        self._compute_wrap_offsets_dict()
        motor_limits = self.motor_limits_dict.copy()

        for step in self.calib_sequence:
            for joint in step["joints"].keys():
                motor_id = self.joint_to_motor_map[joint]
                motor_limits[motor_id] = [None, None]
                self._wrap_offsets_dict[motor_id] = 0.0

        for i, step in enumerate(self.calib_sequence, start=1):
            for joint, _ in step["joints"].items():
                motor_id = self.joint_to_motor_map[joint]

                print(f"Progress: {i}/{len(self.calib_sequence)}")
                print(f"\033[1;35mPlease flex joint {joint} corresponding to motor {motor_id} fully and press enter.\033[0m")
                input()
                flex_position = float(self.get_motor_pos()[self.motor_id_to_idx_dict[motor_id]])
                motor_limits[motor_id][1] = flex_position

                print(f"\033[1;35mPlease extend the joint {joint} corresponding to motor {motor_id} fully and press enter.\033[0m")
                input()
                extend_position = float(self.get_motor_pos()[self.motor_id_to_idx_dict[motor_id]])
                motor_limits[motor_id][0] = extend_position
                
                delta_motor = abs(motor_limits[motor_id][1] - motor_limits[motor_id][0])
                delta_joint = abs(self.joint_roms_dict[joint][1] - self.joint_roms_dict[joint][0])
                self.joint_to_motor_ratios_dict[motor_id] = float(delta_motor / delta_joint)

                calibrated_joints[joint] = 0.0

                print(f"Joint {joint} calibrated. Motor limits: {motor_limits[motor_id]} rad. Ratio: {self.joint_to_motor_ratios_dict[motor_id]}")
                update_yaml(self.calib_path, 'joint_to_motor_ratios', self.joint_to_motor_ratios_dict)
                update_yaml(self.calib_path, 'motor_limits', motor_limits)

                stop_flag = False

                def wait_for_enter_local():
                    nonlocal stop_flag
                    input()
                    stop_flag = True

                thread = threading.Thread(target=wait_for_enter_local, daemon=True)
                thread.start()

                while not stop_flag:
                    curr_pos = self.get_motor_pos()[self.motor_id_to_idx_dict[motor_id]]
                    joint_pos = self.get_joint_pos()[joint]
                    print(f"\rMotor Pos: {curr_pos}, Joint Pos: {joint_pos}", end="")
                    time.sleep(0.01)

                print()

        self.motor_limits_dict.update(motor_limits)
        update_yaml(self.calib_path, 'motor_limits', self.motor_limits_dict)
        update_yaml(self.calib_path, 'joint_to_motor_ratios', self.joint_to_motor_ratios_dict)

        self.set_joint_pos(calibrated_joints)
        time.sleep(1)
        self.set_max_current(self.max_current)

        print("Is fully calibrated: ", self.is_calibrated())
        self.calibrated = self.is_calibrated()
        update_yaml(self.calib_path, 'calibrated', self.calibrated)

        print("\033[1;33mMove away from the hand. Setting joints to 0 in:\033[0m")
        for i in range(3, 0, -1):
            print(f"\033[1;33m{i}\033[0m")
            time.sleep(1)

        self.set_joint_pos(calibrated_joints)
        time.sleep(1)

    def _compute_wrap_offsets_dict(self):
        """Read motor_pos positions once and figure out ±1-turn offsets so that
        pos + offset ∈ [motor_limits_lo, motor_limits_hi].
        """

        motor_pos = self.get_motor_pos()

        lower_limit = np.array([self.motor_limits_dict[motor_id][0] for motor_id in self.motor_ids])
        higher_limit = np.array([self.motor_limits_dict[motor_id][1] for motor_id in self.motor_ids])

        offsets = {}
        for i, motor_id in enumerate(self.motor_ids):
            if lower_limit[i] is None or higher_limit[i] is None:
                offsets[motor_id] = 0.0
                continue

            if motor_pos[i] < lower_limit[i] - 0.25 * np.pi: # Some buffer to compensate for noise/slack differences
                print(f"Motor ID {motor_id} is out of bounds: "
                    f"{lower_limit[i]} < {motor_pos[i]} < {higher_limit[i]}")
                offsets[motor_id] = -2 * np.pi

            elif motor_pos[i] > higher_limit[i] + 0.25 * np.pi: # Some buffer to compensate for noise/slack differences
                print(f"Motor ID {motor_id} is out of bounds: "
                    f"{lower_limit[i]} < {motor_pos[i]} < {higher_limit[i]}")
                offsets[motor_id] = +2 * np.pi

            else:
                offsets[motor_id] = 0.0

        print(f"Offsets: {offsets}")

        self._wrap_offsets_dict = offsets

    def _set_motor_pos(self, desired_pos: Union[dict, np.ndarray, list], rel_to_current: bool = False):
        """Set the desired motor positions in radians.
        
        Args:
            desired_pos (dict or np.ndarray or list): 
                - If dict: {motor_id: desired_position}. Can be partial. Only motors in the dict will be commanded.
                - If np.ndarray or list: Desired positions for all motors in the order of self.motor_ids.
                                         None values will be skipped, and the corresponding motor won't be commanded.
            rel_to_current (bool): If True, the desired position is relative to the current position.
        """
        with self._motor_lock:
            current_positions = self.get_motor_pos() # np.ndarray of all motor positions

            motor_ids_to_write = []
            positions_to_write = []

            if isinstance(desired_pos, dict):
                
                for motor_id, pos_val in desired_pos.items():
                    if motor_id not in self.motor_ids:
                        print(f"Warning: Motor ID {motor_id} in desired_pos dict is not in self.motor_ids. Skipping.")
                        continue

                    if pos_val is None or math.isnan(pos_val):
                        continue

                    pos_to_write = float(pos_val)
                    if rel_to_current:
                        pos_to_write += current_positions[self.motor_id_to_idx_dict[motor_id]]
                    
                    motor_ids_to_write.append(motor_id)
                    positions_to_write.append(pos_to_write)

                if not motor_ids_to_write:
                    return

                positions_to_write = np.array(positions_to_write, dtype=float)

            elif isinstance(desired_pos, (np.ndarray, list)):
                if len(desired_pos) != len(self.motor_ids):
                    raise ValueError(
                        f"Length of desired_pos (list/ndarray) ({len(desired_pos)}) "
                        f"must match the number of configured motor_ids ({len(self.motor_ids)})."
                    )
                
                for i, pos_val in enumerate(desired_pos):
                    if pos_val is None or math.isnan(pos_val):
                        continue
                    else:
                        motor_ids_to_write.append(self.motor_ids[i])
                        current_pos_of_motor = current_positions[i]
                        if rel_to_current:
                            positions_to_write.append(float(pos_val) + current_pos_of_motor)
                        else:
                            positions_to_write.append(float(pos_val))
                
                if not motor_ids_to_write:
                    print("Info: All positions in desired_pos (list/array) were None. No motor commands sent.")
                    return

                motor_ids_to_write = motor_ids_to_write
                positions_to_write = np.array(positions_to_write, dtype=float)
            
            else:
                raise ValueError("desired_pos must be a dict, np.ndarray, or list.")
   
            self._dxl_client.write_desired_pos(motor_ids_to_write, positions_to_write)
    
    def _motor_to_joint_pos(self, motor_pos: np.ndarray) -> dict:
        """Convert motor positions into joint positions.
        
        Args:
            motor_pos (np.ndarray): Motor positions.
        
        Returns:
            dict: {joint_name: position}
        """
        if self._wrap_offsets_dict is None:
            self._compute_wrap_offsets_dict()

        joint_pos = {}
        for idx, pos in enumerate(motor_pos):
            motor_id = self.motor_ids[idx]
            joint_name = self.motor_to_joint_dict.get(motor_id)
            if any(limit is None for limit in self.motor_limits_dict[motor_id]):
                joint_pos[joint_name] = None #TODO: Add a warning here the probably the motor is not calibrated
            elif self.joint_to_motor_ratios_dict[motor_id] == 0:
                joint_pos[joint_name] = None #TODO: Add a warning here the probably the motor is not calibrated
            else:
                wrapped_pos = pos - self._wrap_offsets_dict.get(motor_id, 0.0)
                
                if self.joint_inversion_dict.get(joint_name, False):
                    joint_pos[joint_name] = self.joint_roms_dict[joint_name][1] - (wrapped_pos - self.motor_limits_dict[motor_id][0]) / self.joint_to_motor_ratios_dict[motor_id]
                else:
                    joint_pos[joint_name] = self.joint_roms_dict[joint_name][0] + (wrapped_pos - self.motor_limits_dict[motor_id][0]) / self.joint_to_motor_ratios_dict[motor_id]
        return joint_pos
    
    def _joint_to_motor_pos(self, joint_pos: dict) -> np.ndarray:
        """Convert desired joint positions into motor commands.
    
        Args:
            joint_pos (dict): {joint_name: desired_position}

        Returns:
            np.ndarray: Motor positions.
        """
        if self._wrap_offsets_dict is None:
            self._compute_wrap_offsets_dict()

        motor_pos = [None] * len(self.get_motor_pos())
                
        for joint_name, pos in joint_pos.items():
            motor_id = self.joint_to_motor_map.get(joint_name)
            if motor_id is None or pos is None:
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = None
                continue

            if self.motor_limits_dict[motor_id][0] is None or self.motor_limits_dict[motor_id][1] is None or self.joint_to_motor_ratios_dict[motor_id] == 0:
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = None
                print(f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing joint-to-motor ratio).\033[0m")
                continue
            
            min_pos, max_pos = self.joint_roms_dict[joint_name]
            
            if pos < min_pos or pos > max_pos:
                clipped_pos = max(min_pos, min(max_pos, pos))
                pos = clipped_pos

            if self.joint_inversion_dict.get(joint_name, False):
                # Inverted: higher ROM value corresponds to lower motor position.
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = self.motor_limits_dict[motor_id][0] + (self.joint_roms_dict[joint_name][1] - pos) * self.joint_to_motor_ratios_dict[motor_id]
            else:
                motor_pos[self.motor_id_to_idx_dict[motor_id]] = self.motor_limits_dict[motor_id][0] + (pos - self.joint_roms_dict[joint_name][0]) * self.joint_to_motor_ratios_dict[motor_id]  
            
            motor_pos[self.motor_id_to_idx_dict[motor_id]] += self._wrap_offsets_dict.get(motor_id, 0.0)
            
        return motor_pos
    
    def _sanity_check(self):
        """Check if the configuration is correct and the IDs are consistent."""
        if len(self.motor_ids) != len(self.joint_ids):
            raise ValueError("Number of motor IDs and joints do not match.")
        
        if len(self.motor_ids) != len(self.joint_to_motor_map):
            raise ValueError("Number of motor IDs and joints do not match.")
        
        if self.control_mode not in ['current_position', 'current_velocity', 'position', 'multi_turn_position', 'current_based_position']:
            raise ValueError("Invalid control mode.")
        
        if self.max_current < self.calib_current:
            raise ValueError("Max current should be greater than the calibration current.")
                
        for joint, motor_id in self.joint_to_motor_map.items():
            if joint not in self.joint_ids:
                raise ValueError(f"Joint {joint} is not defined.")
            if joint not in self.joint_roms_dict:
                raise ValueError(f"ROM for joint {joint} is not defined.")
            if motor_id not in self.motor_ids:
                raise ValueError(f"Motor ID {motor_id} is not in the motor IDs list.")
            
        for joint, rom in self.joint_roms_dict.items():
            if rom[1] - rom[0] <= 0:
                raise ValueError(f"ROM for joint {joint} is not valid.")
            if joint not in self.joint_ids:
                raise ValueError(f"Joint {joint} in ROMs is not defined.")
            
        for step in self.calib_sequence:
            for joint, direction in step["joints"].items():
                if joint not in self.joint_ids:
                    raise ValueError(f"Joint {joint} is not defined.")
                if direction not in ['flex', 'extend']:
                    raise ValueError(f"Invalid direction for joint {joint}.")
          
        
        for motor_limit in self.motor_limits_dict.values():
            if any(limit is None for limit in motor_limit):
                self.calibrated = False
                update_yaml(self.calib_path, 'calibrated', False)

    def tension(self, move_motors: bool = False, blocking: bool = True):
        if blocking:
            self._tension(move_motors)
        else:
            self._start_task(self._tension, move_motors)

    def _tension(self, move_motors: bool = False):
        """Freeze the motors, so that the hand can be manually tensioned.
        
        Args:
            move_motors (bool): If True, the hand will move to all motors positively for 3 seconds to set some initial tension.
        """
        self.set_control_mode('position')
        if move_motors:
            motors_to_move = [
                motor_id for joint, motor_id in self.joint_to_motor_map.items()
                if 'wrist' not in joint.lower() and motor_id in self.motor_ids
            ]
            self.set_max_current(self.calib_current)

            duration = 3
            increment_per_step = 0.1
            motor_increments = {motor_id: increment_per_step for motor_id in motors_to_move}

            start_time = time.time()
            while(time.time() - start_time < duration):
                if self._task_stop_event.is_set():
                    break
                self._set_motor_pos(motor_increments, rel_to_current=True)
                time.sleep(0.1)

        self.set_max_current(self.max_current)
        self.disable_torque()
        time.sleep(0.25)
        self.enable_torque()
        print("Holding motors. Please tension carefully. Press Ctrl+C to exit.")
        try:
            while not self._task_stop_event.is_set():
                time.sleep(0.1) 
        finally:
            self.disable_torque()  

    def _run_task(self, task_fn, *args, **kwargs):
        """Run a task in a separate thread, so that it can be stopped externally.
        
        Args:
            task_fn (function): The task function to run.
            *args: Additional arguments to pass to the task function.
            **kwargs: Additional keyword arguments to pass to the task function.
        """
        with self._lock:
            self._task_stop_event.clear()
            self._current_task = task_fn.__name__
            try:
                task_fn(*args, **kwargs)
            finally:
                self._current_task = None

    def _start_task(self, task_fn, *args, **kwargs):
        """Start a task in a separate thread, so that it can be stopped externally.
        
        Args:
            task_fn (function): The task function to run.
            *args: Additional arguments to pass to the task function.
            **kwargs: Additional keyword arguments to pass to the task function.
        """
        if self._task_thread and self._task_thread.is_alive():
            print(f"Task '{self._current_task}' is already running.")
            return

        self._task_thread = threading.Thread(target=self._run_task, args=(task_fn,) + args, kwargs=kwargs)
        self._task_thread.start()

    def stop_task(self):
        """Stop the currently running task.
        """
        if self._task_thread and self._task_thread.is_alive():
            self._task_stop_event.set()
            self._task_thread.join()
            print("Task stopped.")
        else:
            print("No running task to stop.")               

def require_connection(func):
    def wrapper(self, *args, **kwargs):
        if not self._dxl_client.is_connected():
            raise RuntimeError("Hand is not connected.")
        return func(self, *args, **kwargs)
    return wrapper

def require_calibration(func):
    def wrapper(self, *args, **kwargs):
        if not self.calibrated:
            raise RuntimeError("Hand is not calibrated. Please run .calibrate() first.")
        return func(self, *args, **kwargs)
    return wrapper


class MockOrcaHand(OrcaHand):
    """MockOrcaHand class is used to simulate the OrcaHand class for testing."""

    def __init__(self, model_path: str = None):
        """Initialize the MockOrcaHand class."""
        # 先调用父类的初始化
        super().__init__(model_path)

        # 覆盖_dxl_client为None，确保在connect中创建模拟客户端
        self._dxl_client = None

    def connect(self) -> tuple[bool, str]:
        """Connects to the mock Dynamixel client.

        Returns:
            tuple[bool, str]: A tuple containing a boolean indicating success or failure,
                              and a string message.
        """
        try:
            # 确保导入模拟客户端
            from .hardware.mock_dynamixel_client import MockDynamixelClient
            # 创建模拟客户端实例
            self._dxl_client = MockDynamixelClient(self.motor_ids, self.port, self.baudrate)

            # 直接设置连接状态，不调用真实的connect方法
            self._dxl_client._connected = True

            # 初始化模拟客户端的状态
            for motor_id in self.motor_ids:
                self._dxl_client._torque_enabled[motor_id] = False
                self._dxl_client._operating_mode[motor_id] = 0
                self._dxl_client._pos[motor_id] = 2048.0  # 中间位置
                self._dxl_client._vel[motor_id] = 0.0
                self._dxl_client._cur[motor_id] = 0.0
                self._dxl_client._temp[motor_id] = 25.0
                self._dxl_client._torque_limit[motor_id] = 1000

            return True, "Mock connection successful"
        except Exception as e:
            self._dxl_client = None
            return False, f"Mock connection failed: {str(e)}"

    def disconnect(self) -> tuple[bool, str]:
        """Disconnect from the mock hand."""
        try:
            with self._motor_lock:
                if self._dxl_client:
                    # 对于模拟客户端，只需要标记为断开连接
                    self._dxl_client._connected = False
            return True, "Mock disconnected successfully"
        except Exception as e:
            return False, f"Mock disconnection failed: {str(e)}"
        
    
if __name__ == "__main__":
    # Example usage:
    hand = OrcaHand()
    status = hand.connect()
    hand.enable_torque()
    hand.calibrate()

    # Set the desired joint positions to 0
    hand.set_joint_pos({joint: 0 for joint in hand.joint_ids})
    hand.disable_torque()
    hand.disconnect()