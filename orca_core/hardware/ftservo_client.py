# Copyright 2019 The ROBEL Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Communication using the Feite Servo SDK."""

import atexit
import logging
import time
from typing import Optional, Sequence, Union, Tuple
import numpy as np
import os
import sys

# 添加飞特SDK路径
try:
    from ftservo_python_sdk import PortHandler, protocol_packet_handler
except ImportError:
    import ftservo_python_sdk
    from ftservo_python_sdk import PortHandler, protocol_packet_handler

# 飞特舵机协议版本
PROTOCOL_VERSION = 0

# === 根据Excel表格更新飞特舵机控制表地址 ===
# EPROM区域（掉电保存）
ADDR_ID = 5                        # ID地址 (地址5, 1字节, EPROM读写)
ADDR_BAUDRATE = 6                  # 波特率地址 (地址6, 1字节, EPROM读写)
ADDR_MIN_ANGLE_LIMIT = 9           # 最小角度限制 (地址9, 2字节, EPROM读写)
ADDR_MAX_ANGLE_LIMIT = 11          # 最大角度限制 (地址11, 2字节, EPROM读写)
ADDR_MAX_TEMP = 13                 # 最高温度上限 (地址13, 1字节, EPROM读写)
ADDR_MAX_VOLTAGE = 14              # 最高输入电压 (地址14, 1字节, EPROM读写)
ADDR_MIN_VOLTAGE = 15              # 最低输入电压 (地址15, 1字节, EPROM读写)
ADDR_MAX_TORQUE = 16               # 最大扭矩 (地址16, 2字节, EPROM读写)
ADDR_PID_P = 21                    # 位置环P比例系数 (地址21, 1字节, EPROM读写)
ADDR_PID_D = 22                    # 位置环D微分系数 (地址22, 1字节, EPROM读写)
ADDR_PID_I = 23                    # 位置环I积分系数 (地址23, 1字节, EPROM读写)
ADDR_ANGLE_RESOLUTION = 30         # 角度分辨率 (地址30, 1字节, EPROM读写)
ADDR_OPERATING_MODE = 33           # 运行模式 (地址33, 1字节, EPROM读写)
ADDR_PROTECT_TORQUE = 34           # 保护扭矩 (地址34, 1字节, EPROM读写)
ADDR_PROTECT_TIME = 35             # 保护时间 (地址35, 1字节, EPROM读写)
ADDR_OVERLOAD_TORQUE = 36          # 过载扭矩 (地址36, 1字节, EPROM读写)

# RAM区域（运行时控制）
ADDR_TORQUE_ENABLE = 40            # 扭矩开关 (地址40, 1字节, SRAM读写)
ADDR_ACCELERATION = 41             # 加速度 (地址41, 1字节, SRAM读写)
ADDR_GOAL_POSITION = 42            # 目标位置 (地址42, 2字节, SRAM读写)
ADDR_GOAL_TIME = 44                # 运行时间 (地址44, 2字节, SRAM读写)
ADDR_GOAL_VELOCITY = 46            # 运行速度 (地址46, 2字节, SRAM读写)
ADDR_TORQUE_LIMIT = 48             # 转矩限制 (地址48, 2字节, SRAM读写)
ADDR_LOCK = 55                     # 锁标志 (地址55, 1字节, SRAM读写)
ADDR_PRESENT_POSITION = 56         # 当前位置 (地址56, 2字节, SRAM只读)
ADDR_PRESENT_VELOCITY = 58         # 当前速度 (地址58, 2字节, SRAM只读)
ADDR_PRESENT_LOAD = 60             # 当前负载 (地址60, 2字节, SRAM只读)
ADDR_PRESENT_VOLTAGE = 62          # 当前电压 (地址62, 1字节, SRAM只读)
ADDR_PRESENT_TEMPERATURE = 63      # 当前温度 (地址63, 1字节, SRAM只读)
ADDR_ASYNC_WRITE_FLAG = 64         # 异步写标志 (地址64, 1字节, SRAM只读)
ADDR_SERVO_STATUS = 65             # 舵机状态 (地址65, 1字节, SRAM只读)
ADDR_MOVING_STATUS = 66            # 移动标志 (地址66, 1字节, SRAM只读)
ADDR_PRESENT_CURRENT = 69          # 当前电流 (地址69, 2字节, SRAM只读)

# 数据字节长度
LEN_TORQUE_ENABLE = 1
LEN_ACCELERATION = 1
LEN_GOAL_POSITION = 2
LEN_GOAL_TIME = 2
LEN_GOAL_VELOCITY = 2
LEN_TORQUE_LIMIT = 2
LEN_PRESENT_POSITION = 2
LEN_PRESENT_VELOCITY = 2
LEN_PRESENT_LOAD = 2
LEN_PRESENT_CURRENT = 2
LEN_PRESENT_VOLTAGE = 1
LEN_PRESENT_TEMPERATURE = 1
LEN_MOVING_STATUS = 1
LEN_SERVO_STATUS = 1

# 批量读取当前位置、速度、负载的起始地址和长度
ADDR_PRESENT_POS_VEL_LOAD = 56     # 位置(56,2) + 速度(58,2) + 负载(60,2) = 6字节
LEN_PRESENT_POS_VEL_LOAD = 6

# STS3215舵机参数配置
DEFAULT_POS_SCALE = 2.0 * np.pi / 4096  # 转换为弧度 (4096步对应360度)
DEFAULT_VEL_SCALE = 0.732 / 50.0        # 速度单位转换 (50步/秒 = 0.732 RPM)
DEFAULT_CUR_SCALE = 0.001               # 电流单位转换


def dynamixel_cleanup_handler():
    """Cleanup function to ensure servos are disconnected properly."""
    open_clients = list(DynamixelClient.OPEN_CLIENTS)
    for open_client in open_clients:
        open_client.disconnect()


def signed_to_unsigned(value: int, size: int) -> int:
    """Converts the given value to its unsigned representation."""
    if value < 0:
        bit_size = 8 * size
        max_value = (1 << bit_size) - 1
        value = max_value + value
    return value


def unsigned_to_signed(value: int, size: int) -> int:
    """Converts the given value from its unsigned representation."""
    bit_size = 8 * size
    if (value & (1 << (bit_size - 1))) != 0:
        value = -((1 << bit_size) - value)
    return value


class DynamixelClient:
    """Client for communicating with Feite Servo motors.

    NOTE: This adapts Feite Servo SDK to maintain compatibility with Dynamixel interface.
    """

    # The currently open clients.
    OPEN_CLIENTS = set()

    def __init__(self,
                 motor_ids: Sequence[int],
                 port: str = 'COM9',
                 baudrate: int = 1000000,
                 lazy_connect: bool = False,
                 pos_scale: Optional[float] = None,
                 vel_scale: Optional[float] = None,
                 cur_scale: Optional[float] = None):
        """Initializes a new client for Feite Servos.

        Args:
            motor_ids: All motor IDs being used by the client.
            port: The servo device to talk to.
            baudrate: The servo baudrate to communicate with.
            lazy_connect: If True, automatically connects when calling a method
                that requires a connection, if not already connected.
            pos_scale: The scaling factor for the positions.
            vel_scale: The scaling factor for the velocities.
            cur_scale: The scaling factor for the currents.
        """
        from ftservo_python_sdk import PortHandler, protocol_packet_handler

        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = baudrate
        self.lazy_connect = lazy_connect

        # 飞特SDK初始化
        self.port_handler = PortHandler(port)
        self.protocol = protocol_packet_handler(self.port_handler, PROTOCOL_VERSION)

        self._pos_vel_cur_reader = DynamixelPosVelCurReader(
            self,
            self.motor_ids,
            pos_scale=pos_scale if pos_scale is not None else DEFAULT_POS_SCALE,
            vel_scale=vel_scale if vel_scale is not None else DEFAULT_VEL_SCALE,
            cur_scale=cur_scale if cur_scale is not None else DEFAULT_CUR_SCALE,
        )

        self._temp_reader = DynamixelTempReader(
            self,
            self.motor_ids,
            address=ADDR_PRESENT_TEMPERATURE,
            size=LEN_PRESENT_TEMPERATURE,
        )

        self._moving_status_reader = DynamixelReader(self, self.motor_ids, ADDR_MOVING_STATUS, LEN_MOVING_STATUS)
        self._servo_status_reader = DynamixelReader(self, self.motor_ids, ADDR_SERVO_STATUS, LEN_SERVO_STATUS)

        self.OPEN_CLIENTS.add(self)

    @property
    def is_connected(self) -> bool:
        try:
            # 优先使用 SDK 提供的 is_using
            if hasattr(self.port_handler, "is_using"):
                # 有些 SDK 不会自动更新 is_using，因此还要加串口状态兜底判断
                return self.port_handler.is_using or self.port_handler.ser.is_open
            # 若没有 is_using，则直接看 pyserial 串口是否打开
            elif hasattr(self.port_handler, "ser"):
                return self.port_handler.ser.is_open
            else:
                return False
        except Exception:
            return False

    def connect(self):
        """Connects to the Feite Servo motors."""
        assert not self.is_connected, 'Client is already connected.'

        print("DEBUG: port_handler =", type(self.port_handler))
        print("DEBUG: has openPort:", hasattr(self.port_handler, "openPort"))
        print("DEBUG: has is_using:", hasattr(self.port_handler, "is_using"))
        print("DEBUG: port_handler.is_using =", getattr(self.port_handler, "is_using", None))

        # 打开端口
        if not self.port_handler.openPort():
            raise OSError(
                ('Failed to open port at {} (Check that the device is powered '
                 'on and connected to your computer).').format(self.port_name))

        # 设置波特率
        if not self.port_handler.setBaudRate(self.baudrate):
            raise OSError(
                ('Failed to set the baudrate to {} (Ensure that the device was '
                 'configured for this baudrate).').format(self.baudrate))

        logging.info('Succeeded to open port: %s with baudrate: %d', self.port_name, self.baudrate)

        # 启动时使能所有电机扭矩
        self.set_torque_enabled(self.motor_ids, True)

    def disconnect(self):
        """Disconnects from the Feite Servo device."""
        if not self.is_connected:
            return

        # 确保电机扭矩被禁用
        self.set_torque_enabled(self.motor_ids, False, retries=0)

        # 关闭端口
        self.port_handler.closePort()
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
        remaining_ids = list(motor_ids)
        while remaining_ids:
            remaining_ids = self.write_byte(
                remaining_ids,
                int(enabled),
                ADDR_TORQUE_ENABLE,
            )
            if remaining_ids:
                logging.error('Could not set torque %s for IDs: %s',
                              'enabled' if enabled else 'disabled',
                              str(remaining_ids))
            if retries == 0:
                break
            time.sleep(retry_interval)
            retries -= 1

    def set_operating_mode(self, motor_ids: Sequence[int], mode_value: int):
        """
        Set operating mode for Feite servos.
        Modes: 0=位置伺服模式, 1=电机恒速模式, 2=PWM开环调速度模式, 3=步进伺服模式
        """
        # 先关闭扭矩
        self.set_torque_enabled(motor_ids, False)
        time.sleep(0.1)

        # 设置运行模式
        for motor_id in motor_ids:
            self.write_byte([motor_id], mode_value, ADDR_OPERATING_MODE)

        # 重新使能扭矩
        time.sleep(0.1)
        self.set_torque_enabled(motor_ids, True)

    def set_torque_limit(self, motor_ids: Sequence[int], torque_limit: int):
        """
        Set torque limit for servos.
        Args:
            torque_limit: 0-1000, where 1000 = 100% of max torque
        """
        for motor_id in motor_ids:
            self.write_word([motor_id], torque_limit, ADDR_TORQUE_LIMIT)

    def get_torque_limits(self, motor_ids: Sequence[int]) -> np.ndarray:
        """批量读取扭矩限制值

        Args:
            motor_ids: 电机ID列表

        Returns:
            扭矩限制值数组
        """
        self.check_connected()
        torque_data = np.zeros(len(motor_ids), dtype=np.int32)

        for i, motor_id in enumerate(motor_ids):
            torque_value, result, error = self.protocol.read2ByteTxRx(motor_id, ADDR_TORQUE_LIMIT)
            if result == 0:
                torque_data[i] = torque_value
            else:
                torque_data[i] = 0
                logging.error('Failed to read torque limit for motor %d', motor_id)

        return torque_data

    def read_pos_vel_cur(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns the positions, velocities, and currents."""
        return self._pos_vel_cur_reader.read()

    def read_status_is_done_moving(self) -> bool:
        """Returns the last bit of moving status"""
        moving_status = self._moving_status_reader.read().astype(np.int8)
        return np.bitwise_and(moving_status, np.array([0x01] * len(moving_status)).astype(np.int8))

    def read_servo_status(self) -> np.ndarray:
        """Reads servo status for error detection."""
        return self._servo_status_reader.read()

    def read_temperature(self) -> np.ndarray:
        """Reads and returns the present temperature for each motor (in deg C)."""
        return self._temp_reader.read()

    def write_desired_pos(self, motor_ids: Sequence[int],
                          positions: np.ndarray):
        """Writes the given desired positions.

        Args:
            motor_ids: The motor IDs to write to.
            positions: The joint angles in radians to write.
        """
        assert len(motor_ids) == len(positions)

        # 转换为飞特舵机位置空间 (弧度转步数)
        positions_steps = positions / self._pos_vel_cur_reader.pos_scale

        # 飞特舵机使用单独写入
        times = [time.monotonic()]
        for motor_id, position in zip(motor_ids, positions_steps):
            result, error = self.protocol.write2ByteTxRx(motor_id, ADDR_GOAL_POSITION, int(position))
            if result != 0:
                logging.error('Failed to write position for motor %d: %s', motor_id,
                              self.protocol.getTxRxResult(result))

        times.append(time.monotonic())
        return times

    def write_desired_velocity(self, motor_ids: Sequence[int], velocities: np.ndarray):
        """Writes desired velocity."""
        assert len(motor_ids) == len(velocities)

        for motor_id, velocity in zip(motor_ids, velocities):
            result, error = self.protocol.write2ByteTxRx(motor_id, ADDR_GOAL_VELOCITY, int(velocity))
            if result != 0:
                logging.error('Failed to write velocity for motor %d', motor_id)

    def write_acceleration(self, motor_ids: Sequence[int], accelerations: np.ndarray):
        """Writes acceleration values."""
        assert len(motor_ids) == len(accelerations)

        for motor_id, acceleration in zip(motor_ids, accelerations):
            result, error = self.protocol.write1ByteTxRx(motor_id, ADDR_ACCELERATION, int(acceleration))
            if result != 0:
                logging.error('Failed to write acceleration for motor %d', motor_id)

    def write_byte(
            self,
            motor_ids: Sequence[int],
            value: int,
            address: int,
    ) -> Sequence[int]:
        """Writes a byte value to the motors.

        Args:
            motor_ids: The motor IDs to write to.
            value: The value to write to the control table.
            address: The control table address to write to.

        Returns:
            A list of IDs that were unsuccessful.
        """
        self.check_connected()
        errored_ids = []
        for motor_id in motor_ids:
            comm_result, dxl_error = self.protocol.write1ByteTxRx(motor_id, address, value)
            success = self.handle_packet_result(
                comm_result, dxl_error, motor_id, context='write_byte')
            if not success:
                errored_ids.append(motor_id)
        return errored_ids

    def write_word(
            self,
            motor_ids: Sequence[int],
            value: int,
            address: int,
    ) -> Sequence[int]:
        """Writes a word (2 bytes) value to the motors.

        Args:
            motor_ids: The motor IDs to write to.
            value: The value to write to the control table.
            address: The control table address to write to.

        Returns:
            A list of IDs that were unsuccessful.
        """
        self.check_connected()
        errored_ids = []
        for motor_id in motor_ids:
            comm_result, dxl_error = self.protocol.write2ByteTxRx(motor_id, address, value)
            success = self.handle_packet_result(
                comm_result, dxl_error, motor_id, context='write_word')
            if not success:
                errored_ids.append(motor_id)
        return errored_ids

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
        error_message = None
        if comm_result != 0:  # Feite SDK: 0表示成功
            error_message = self.protocol.getTxRxResult(comm_result)

        if error_message:
            if dxl_id is not None:
                error_message = '[Motor ID: {}] {}'.format(dxl_id, error_message)
            if context is not None:
                error_message = '> {}: {}'.format(context, error_message)
            logging.error(error_message)
            return False
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
    """Reads data from Feite Servo motors."""

    def __init__(self, client: DynamixelClient, motor_ids: Sequence[int],
                 address: int, size: int):
        """Initializes a new reader."""
        self.client = client
        self.motor_ids = motor_ids
        self.address = address
        self.size = size
        self._initialize_data()

    def read(self, retries: int = 1):
        """Reads data from the motors."""
        self.client.check_connected()

        # 飞特SDK使用单独读取
        for i, motor_id in enumerate(self.motor_ids):
            success = False
            attempts = 0
            while not success and attempts <= retries:
                try:
                    if self.size == 1:
                        value, result, error = self.client.protocol.read1ByteTxRx(motor_id, self.address)
                    elif self.size == 2:
                        value, result, error = self.client.protocol.read2ByteTxRx(motor_id, self.address)
                    else:
                        logging.error('Unsupported read size: %d', self.size)
                        break

                    if result == 0:
                        self._update_data(i, motor_id, value)
                        success = True
                    else:
                        attempts += 1
                        if attempts > retries:
                            logging.error('Read failed for motor %d after %d attempts', motor_id, retries)
                except Exception as e:
                    logging.error('Exception reading motor %d: %s', motor_id, e)
                    attempts += 1

        return self._get_data()

    def _initialize_data(self):
        """Initializes the cached data."""
        self._data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int, value: int):
        """Updates the data index for the given motor ID."""
        self._data[index] = float(value)

    def _get_data(self):
        """Returns a copy of the data."""
        return self._data.copy()


class DynamixelPosVelCurReader(DynamixelReader):
    """Reads positions, velocities and currents."""

    def __init__(self,
                 client: DynamixelClient,
                 motor_ids: Sequence[int],
                 pos_scale: float = 1.0,
                 vel_scale: float = 1.0,
                 cur_scale: float = 1.0):
        super().__init__(
            client,
            motor_ids,
            address=ADDR_PRESENT_POS_VEL_LOAD,
            size=LEN_PRESENT_POS_VEL_LOAD,
        )
        self.pos_scale = pos_scale
        self.vel_scale = vel_scale
        self.cur_scale = cur_scale

    def _initialize_data(self):
        """Initializes the cached data."""
        self._pos_data = np.zeros(len(self.motor_ids), dtype=np.float32)
        self._vel_data = np.zeros(len(self.motor_ids), dtype=np.float32)
        self._cur_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def read(self, retries: int = 1):
        """Reads position, velocity and current data."""
        self.client.check_connected()

        for i, motor_id in enumerate(self.motor_ids):
            # 分别读取位置、速度、负载
            pos_success = False
            vel_success = False
            load_success = False

            attempts = 0
            while (not pos_success or not vel_success or not load_success) and attempts <= retries:
                # 读取位置
                if not pos_success:
                    pos_value, pos_result, _ = self.client.protocol.read2ByteTxRx(motor_id, ADDR_PRESENT_POSITION)
                    if pos_result == 0:
                        # 飞特舵机位置值需要转换为有符号数
                        actual_pos = self.client.protocol.scs_tohost(pos_value, 15)
                        self._pos_data[i] = float(actual_pos) * self.pos_scale
                        pos_success = True

                # 读取速度
                if not vel_success:
                    vel_value, vel_result, _ = self.client.protocol.read2ByteTxRx(motor_id, ADDR_PRESENT_VELOCITY)
                    if vel_result == 0:
                        actual_vel = self.client.protocol.scs_tohost(vel_value, 15)
                        self._vel_data[i] = float(actual_vel) * self.vel_scale
                        vel_success = True

                # 读取负载
                if not load_success:
                    load_value, load_result, _ = self.client.protocol.read2ByteTxRx(motor_id, ADDR_PRESENT_LOAD)
                    if load_result == 0:
                        self._cur_data[i] = float(load_value) * self.cur_scale
                        load_success = True

                attempts += 1

                if attempts > retries:
                    if not pos_success:
                        logging.error('Failed to read position for motor %d', motor_id)
                    if not vel_success:
                        logging.error('Failed to read velocity for motor %d', motor_id)
                    if not load_success:
                        logging.error('Failed to read load for motor %d', motor_id)

        return self._get_data()

    def _get_data(self):
        """Returns a copy of the data."""
        return (self._pos_data.copy(), self._vel_data.copy(),
                self._cur_data.copy())


class DynamixelTempReader(DynamixelReader):
    """Reads present temperature for each Feite Servo motor."""

    def _initialize_data(self):
        self._temp_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int, value: int):
        self._temp_data[index] = float(value)

    def _get_data(self):
        return self._temp_data.copy()


# Register global cleanup function.
atexit.register(dynamixel_cleanup_handler)



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
        default='COM9',
        help='The Feite Servo device to connect to.')
    parser.add_argument(
        '-b', '--baud', default=1000000, help='The baudrate to connect with.')
    parsed_args = parser.parse_args()
    motors = [int(motor) for motor in parsed_args.motors.split(',')]

    way_points = [np.zeros(len(motors)), np.full(len(motors), np.pi)]

    with DynamixelClient(motors, parsed_args.device,
                         parsed_args.baud) as dxl_client:
        for step in itertools.count():
            if step > 0 and step % 50 == 0:
                way_point = way_points[(step // 100) % len(way_points)]
                print('Writing: {}'.format(way_point.tolist()))
                dxl_client.write_desired_pos(motors, way_point)
            read_start = time.time()
            pos_now, vel_now, cur_now = dxl_client.read_pos_vel_cur()
            if step % 5 == 0:
                print('[{}] Frequency: {:.2f} Hz'.format(
                    step, 1.0 / (time.time() - read_start)))
                print('> Pos: {}'.format(pos_now.tolist()))
                print('> Vel: {}'.format(vel_now.tolist()))
                print('> Cur: {}'.format(cur_now.tolist()))