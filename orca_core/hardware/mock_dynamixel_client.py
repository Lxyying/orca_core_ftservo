# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Communication using a simulated Dynamixel client."""

import atexit
import logging
import time
import random
from typing import Optional, Sequence, Union, Tuple
import numpy as np

# 飞特舵机控制表地址
ADDR_TORQUE_ENABLE = 40
ADDR_OPERATING_MODE = 33
ADDR_GOAL_POSITION = 42
ADDR_TORQUE_LIMIT = 48
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_VELOCITY = 58
ADDR_PRESENT_CURRENT = 69
ADDR_PRESENT_TEMPERATURE = 63
ADDR_SERVO_STATUS = 65
ADDR_MOVING_STATUS = 66

# 飞特舵机参数配置
DEFAULT_POS_SCALE = 2.0 * np.pi / 4096  # 转换为弧度 (4096步对应360度)
DEFAULT_VEL_SCALE = 0.732 / 50.0  # 速度单位转换
DEFAULT_CUR_SCALE = 0.001  # 电流单位转换


class MockProtocol:
    """模拟飞特舵机协议处理器"""

    def __init__(self, client):
        self.client = client

    def write1ByteTxRx(self, motor_id, address, value):
        """模拟写入1字节数据"""
        logging.debug(f"MockProtocol write1ByteTxRx: motor_id={motor_id}, address={address}, value={value}")
        if address == ADDR_TORQUE_ENABLE:
            self.client._torque_enabled[motor_id] = (value == 1)
            print(f"Mock: Motor {motor_id} torque {'enabled' if value == 1 else 'disabled'}")
        elif address == ADDR_OPERATING_MODE:
            self.client._operating_mode[motor_id] = value
            print(f"Mock: Motor {motor_id} operating mode set to {value}")
        return 0, 0  # (result, error)

    def read1ByteTxRx(self, motor_id, address):
        """模拟读取1字节数据"""
        logging.debug(f"MockProtocol read1ByteTxRx: motor_id={motor_id}, address={address}")
        if address == ADDR_PRESENT_TEMPERATURE:
            return int(self.client._temp[motor_id]), 0, 0
        elif address == ADDR_SERVO_STATUS:
            return 0, 0, 0  # 假设无错误
        elif address == ADDR_MOVING_STATUS:
            return 0, 0, 0  # 假设不在移动
        else:
            return 0, 0, 0


class MockDynamixelClient:
    """Mock client for simulating communication with Feite servos."""

    # The currently open clients.
    OPEN_CLIENTS = set()

    def __init__(self,
                 motor_ids: Sequence[int],
                 port: str = 'COM6',
                 baudrate: int = 1000000,
                 lazy_connect: bool = False,
                 pos_scale: Optional[float] = None,
                 vel_scale: Optional[float] = None,
                 cur_scale: Optional[float] = None):
        """Initializes a new mock client.

        Args:
            motor_ids: All motor IDs being used by the client.
            port: The Dynamixel device to talk to. e.g.
                - Linux: /dev/ttyUSB0
                - Mac: /dev/tty.usbserial-*
                - Windows: COM1
            baudrate: The Dynamixel baudrate to communicate with.
            lazy_connect: If True, automatically connects when calling a method
                that requires a connection, if not already connected.
            pos_scale: The scaling factor for the positions. This is
                motor-dependent. If not provided, uses the default scale.
            vel_scale: The scaling factor for the velocities. This is
                motor-dependent. If not provided uses the default scale.
            cur_scale: The scaling factor for the currents. This is
                motor-dependent. If not provided uses the default scale.
        """

        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = baudrate
        self.lazy_connect = lazy_connect

        # States for simulation.
        self._connected = False
        self._torque_enabled = {mid: False for mid in self.motor_ids}
        self._operating_mode = {mid: 0 for mid in self.motor_ids}  # 默认位置模式
        self._pos = {mid: 2048.0 for mid in self.motor_ids}  # 飞特舵机中间位置2048
        self._vel = {mid: 0.0 for mid in self.motor_ids}
        self._cur = {mid: 0.0 for mid in self.motor_ids}
        self._temp = {mid: 25.0 for mid in self.motor_ids}  # 默认温度25°C
        self._torque_limit = {mid: 1000 for mid in self.motor_ids}  # 默认扭矩限制1000

        # 飞特舵机位置范围：0-4095对应0-360度
        self._min_pos = 0
        self._max_pos = 4095

        # 创建模拟协议处理器
        self.protocol = MockProtocol(self)

        # 直接使用简化的读取器
        self._pos_scale = pos_scale if pos_scale is not None else DEFAULT_POS_SCALE
        self._vel_scale = vel_scale if vel_scale is not None else DEFAULT_VEL_SCALE
        self._cur_scale = cur_scale if cur_scale is not None else DEFAULT_CUR_SCALE

        self.OPEN_CLIENTS.add(self)

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self):
        """Connects to the Dynamixel motors.

        NOTE: This should be called after all DynamixelClients on the same
            process are created.
        """
        assert not self.is_connected, 'Client is already connected.'

        logging.info('Mock: Succeeded to open port: %s', self.port_name)
        logging.info('Mock: Succeeded to set baudrate to %d', self.baudrate)

        self._connected = True

        # Start with all motors disabled (safer for simulation)
        self.set_torque_enabled(self.motor_ids, False)

    def disconnect(self):
        """Disconnects from the Dynamixel device."""
        if not self.is_connected:
            return

        self.set_torque_enabled(self.motor_ids, False)
        self._connected = False

        if self in self.OPEN_CLIENTS:
            self.OPEN_CLIENTS.remove(self)

    def set_torque_enabled(self,
                           motor_ids: Sequence[int],
                           enabled: bool,
                           retries: int = -1,
                           retry_interval: float = 0.25):
        """Sets whether torque is enabled for the motors.

        Args:
            motor_ids: The motor IDs to configure.
            enabled: Whether to engage or disengage the motors.
            retries: The number of times to retry. If this is <0, will retry
                forever.
            retry_interval: The number of seconds to wait between retries.
        """
        for mid in motor_ids:
            if mid not in self._torque_enabled:
                raise ValueError('Motor ID {} not found in client.'.format(mid))
            self._torque_enabled[mid] = enabled
            logging.info('Mock: Motor %d torque %s', mid, 'enabled' if enabled else 'disabled')

    def set_operating_mode(self, motor_ids: Sequence[int], mode_value: int):
        """
        飞特舵机运行模式：
        0: 位置伺服模式
        1: 电机恒速模式
        2: PWM开环调速度模式
        3: 步进伺服模式
        """
        # 先关闭扭矩（与ftservo_client.py一致）
        self.set_torque_enabled(motor_ids, False)
        time.sleep(0.1)

        # 设置运行模式
        for mid in motor_ids:
            if mid not in self._operating_mode:
                raise ValueError('Motor ID {} not found in client.'.format(mid))
            self._operating_mode[mid] = mode_value
            logging.info('Mock: Set operating mode for motor %d to %d', mid, mode_value)

        # 重新使能扭矩（与ftservo_client.py一致）
        time.sleep(0.1)
        self.set_torque_enabled(motor_ids, True)

    def set_torque_limit(self, motor_ids: Sequence[int], torque_limit: int):
        """设置扭矩限制，范围0-1000"""
        for mid in motor_ids:
            if mid not in self._torque_limit:
                raise ValueError('Motor ID {} not found in client.'.format(mid))
            self._torque_limit[mid] = torque_limit
            logging.info('Mock: Set torque limit for motor %d to %d', mid, torque_limit)

    def read_pos_vel_cur(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns the simulated positions, velocities, and currents."""

        # 飞特舵机位置范围：0-4095，转换为弧度
        pos_array = np.array([self._pos[mid] * self._pos_scale for mid in self.motor_ids])
        vel_array = np.array([self._vel[mid] * self._vel_scale for mid in self.motor_ids])
        cur_array = np.array([self._cur[mid] * self._cur_scale for mid in self.motor_ids])

        return pos_array, vel_array, cur_array

    def read_status_is_done_moving(self) -> bool:
        """Returns the last bit of moving status"""
        return True

    def read_temperature(self) -> np.ndarray:
        """Reads and returns the simulated temperatures."""
        temp_array = np.array([self._temp[mid] for mid in self.motor_ids])
        return temp_array

    def write_desired_pos(self, motor_ids: Sequence[int],
                          positions: np.ndarray):
        """Writes the given desired positions.

        Args:
            motor_ids: The motor IDs to write to.
            positions: The joint angles in radians to write.
        """
        assert len(motor_ids) == len(positions)

        for i, mid in enumerate(motor_ids):
            if mid not in self._pos:
                raise ValueError('Motor ID {} not found in client.'.format(mid))

            # 将弧度转换为飞特舵机位置单位 (0-4095)
            pos_steps = positions[i] / self._pos_scale

            if pos_steps > self._max_pos:
                self._pos[mid] = self._max_pos
            elif pos_steps < self._min_pos:
                self._pos[mid] = self._min_pos
            else:
                self._pos[mid] = pos_steps

        # 返回模拟的时间戳
        times = [0.0]
        for _ in range(4):
            times.append(times[-1] + random.uniform(0.01, 0.05))
        return times

    def write_byte(
            self,
            motor_ids: Sequence[int],
            value: int,
            address: int,
    ) -> Sequence[int]:
        """Writes a value to the motors.

        Args:
            motor_ids: The motor IDs to write to.
            value: The value to write to the control table.
            address: The control table address to write to.

        Returns:
            A list of IDs that were unsuccessful.
        """
        self.check_connected()
        return []

    def check_connected(self):
        """Ensures the robot is connected."""
        if self.lazy_connect and not self.is_connected:
            self.connect()
        if not self.is_connected:
            raise OSError('Must call connect() first.')

    def handle_packet_result(self,
                             comm_result: int,
                             dxl_error: Optional[int] = None,
                             dxl_id: Optional[int] = None,
                             context: Optional[str] = None):
        """Handles the result from a communication request."""
        return True

    def __enter__(self):
        """Enables use as a context manager."""
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, *args):
        """Enables use as a context manager."""
        self.disconnect()

    def __del__(self):
        """Automatically disconnect on destruction."""
        self.disconnect()


class DynamixelReader:
    """Reads data from Dynamixel motors.

    This wraps a GroupBulkRead from the DynamixelSDK.
    """

    def __init__(self, client: MockDynamixelClient, motor_ids: Sequence[int],
                 address: int, size: int):
        """Initializes a new reader."""
        self.client = client
        self.motor_ids = motor_ids
        self.address = address
        self.size = size
        self._initialize_data()

    def read(self, retries: int = 1):
        """Reads data from the motors - simplified for Feite servos."""
        self.client.check_connected()

        # 直接更新所有电机的数据，不进行实际的批量读取
        for i, motor_id in enumerate(self.motor_ids):
            self._update_data(i, motor_id)

        return self._get_data()

    def _initialize_data(self):
        """Initializes the cached data."""
        self._data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        """Updates the data index for the given motor ID."""
        # 基类的默认实现，子类会重写这个方法
        pass

    def _get_data(self):
        """Returns a copy of the data."""
        return self._data.copy()


class DynamixelPosVelCurReader(DynamixelReader):
    """Reads positions, velocities and currents for Feite servos."""

    def __init__(self,
                 client: MockDynamixelClient,
                 motor_ids: Sequence[int],
                 pos_scale: float = 1.0,
                 vel_scale: float = 1.0,
                 cur_scale: float = 1.0):
        super().__init__(
            client,
            motor_ids,
            address=ADDR_PRESENT_POSITION,  # 使用飞特舵机的批量读取地址
            size=6,  # 位置2字节+速度2字节+电流2字节
        )
        self.pos_scale = pos_scale
        self.vel_scale = vel_scale
        self.cur_scale = cur_scale

    def _initialize_data(self):
        """Initializes the cached data."""
        self._pos_data = np.zeros(len(self.motor_ids), dtype=np.float32)
        self._vel_data = np.zeros(len(self.motor_ids), dtype=np.float32)
        self._cur_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        """Updates the data index for the given motor ID for Feite servos."""
        # 飞特舵机使用不同的数据格式，直接使用模拟的状态数据
        pos = self.client._pos.get(motor_id, 2048)  # 使用2048作为默认位置
        vel = self.client._vel.get(motor_id, 0)
        cur = self.client._cur.get(motor_id, 0)

        # 飞特舵机位置已经是0-4096范围内的值，需要转换为弧度
        self._pos_data[index] = float(pos) * self.pos_scale
        self._vel_data[index] = float(vel) * self.vel_scale
        self._cur_data[index] = float(cur) * self.cur_scale

    def _get_data(self):
        """Returns a copy of the data."""
        return (self._pos_data.copy(), self._vel_data.copy(),
                self._cur_data.copy())


class DynamixelTempReader(DynamixelReader):
    """Reads present temperature (1 byte) for each Dynamixel motor."""

    def _initialize_data(self):
        # We'll store one float per motor for the temperature values.
        self._temp_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        # 直接使用模拟的温度数据
        temp_value = self.client._temp.get(motor_id, 25.0)
        self._temp_data[index] = float(temp_value)

    def _get_data(self):
        return self._temp_data.copy()


# 模拟版本的清理函数
def mock_cleanup_handler():
    """Cleanup function for mock clients."""
    open_clients = list(MockDynamixelClient.OPEN_CLIENTS)
    for open_client in open_clients:
        open_client.disconnect()


# Register global cleanup function.
atexit.register(mock_cleanup_handler)

if __name__ == '__main__':
    import argparse
    import itertools

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-m',
        '--motors',
        required=True,
        help='Comma-separated list of motor IDs.')
    parser.add_argument(
        '-d',
        '--device',
        default='/dev/cu.usbserial-FT62AFSR',
        help='The Dynamixel device to connect to.')
    parser.add_argument(
        '-b', '--baud', default=1000000, help='The baudrate to connect with.')
    parsed_args = parser.parse_args()
    motors = [int(motor) for motor in parsed_args.motors.split(',')]

    way_points = [
        np.full(len(motors), 2048),  # 中间位置 (2048步)
        np.full(len(motors), 1000),  # 较小位置 (1000步)
        np.full(len(motors), 3500)  # 较大位置 (3500步)
    ]

    with MockDynamixelClient(motors, parsed_args.device,
                             parsed_args.baud) as dxl_client:
        for step in itertools.count():
            if step > 0 and step % 50 == 0:
                way_point_steps = way_points[(step // 100) % len(way_points)]
                # 将步数转换为弧度（因为write_desired_pos需要弧度输入）
                way_point_rad = way_point_steps * DEFAULT_POS_SCALE
                print('Writing: {} steps ({} rad)'.format(way_point_steps.tolist(), way_point_rad.tolist()))
                dxl_client.write_desired_pos(motors, way_point_rad)