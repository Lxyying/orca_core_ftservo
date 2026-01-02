"""
抓取控制器 - 基于ORCA Hand实际16关节结构
参考main_demo.py中的关节控制实现
"""

import time
import math
import threading
from typing import Dict, Optional, List, Union

# --- 修正导入路径 ---
# 1. 导入核心库和工具：使用绝对路径从 orca_core 包导入 (依赖于 competition_demo.py 设置 sys.path)
try:
    # 修正: 使用绝对导入 from orca_core.core
    from orca_core.core import OrcaHand
    # 同样修正 utils 的导入
    from orca_core.utils.utils import get_model_path, read_yaml, ease_in_out

    ORCA_HAND_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ 警告: 无法导入OrcaHand类或Utils: {e}")
    print("🎮 将使用纯模拟模式")
    ORCA_HAND_AVAILABLE = False


    # 定义哑类/哑函数防止代码崩溃
    class OrcaHand:
        def __init__(self):
            pass

        def connect(self):
            return True, "模拟连接"

        def is_connected(self):
            return True

        def enable_torque(self):
            pass

        def set_control_mode(self, mode):
            pass

        def get_joint_pos(self, as_list=False):
            return {}

        def set_motor_positions_direct(self, positions):
            return True

        def get_motor_positions_dict(self):
            return {}

        def get_motor_pos(self):
            return []

        def disable_torque(self):
            pass

# 2. 导入模式管理器：使用正确的相对路径从同级的 mode 模块导入
try:
    from .mode import get_mode_manager

    MODES_AVAILABLE = True
except ImportError as e:
    print(f"❌ 错误: 无法导入mode模块: {e}")
    MODES_AVAILABLE = False


    # 创建临时的模式管理器替代
    class TemporaryModeManager:
        def get_mode(self, mode_id):
            # 返回16关节的默认角度
            return {
                'target_angles': {
                    'thumb_mcp': 0, 'thumb_abd': 0, 'thumb_pip': 0, 'thumb_dip': 0,
                    'index_abd': 0, 'index_mcp': 0, 'index_pip': 0,
                    'middle_abd': 0, 'middle_mcp': 0, 'middle_pip': 0,
                    'ring_abd': 0, 'ring_mcp': 0, 'ring_pip': 0,
                    'pinky_abd': 0, 'pinky_mcp': 0, 'pinky_pip': 0
                },
                'motion_params': {
                    'step_size': 1.0,
                    'approach_speed': 0.5,
                    'max_steps': 50
                },
                'name': '临时模式',
                'description': '临时模式描述',
                'grasp_tips': '临时技巧提示'
            }

        def validate_angles(self, angles):
            return True, "验证通过"

        def get_motor_ids(self):
            return list(range(1, 17))

        def get_joint_to_motor_map(self):
            return {}

        def get_motor_to_joint_map(self):
            return {}

        def get_motor_id_to_idx_dict(self):
            return {}

        @property
        def default_neutral(self):
            """返回默认中立位"""
            return {
                'thumb_mcp': 3.220, 'thumb_abd': 2.684, 'thumb_pip': 3.220, 'thumb_dip': 3.220,
                'index_abd': 2.300, 'index_mcp': 1.995, 'index_pip': 2.608,
                'middle_abd': 2.915, 'middle_mcp': 2.454, 'middle_pip': 2.761,
                'ring_abd': 2.300, 'ring_mcp': 3.220, 'ring_pip': 3.067,
                'pinky_abd': 3.034, 'pinky_mcp': 2.837, 'pinky_pip': 3.375
            }


class GraspController:
    """
    抓取控制器主类 - 支持16关节控制
    基于ORCA Hand实际关节结构
    """

    def __init__(self, simulation: bool = True, hand_instance=None, config_path: str = None):
        """
        初始化抓取控制器

        Args:
            simulation: 是否使用模拟模式 (默认True)
            hand_instance: 实际的OrcaHand实例，在硬件模式下必须提供
            config_path: 配置文件路径
        """
        self.simulation = simulation
        self.config_path = config_path

        # 初始化模式管理器
        if MODES_AVAILABLE:
            # 修正: 调用正确的 get_mode_manager
            from .mode import get_mode_manager as _get_mode_manager
            self.mode_manager = _get_mode_manager()
        else:
            print("⚠️ 使用临时模式管理器")
            self.mode_manager = TemporaryModeManager()

        # 处理硬件实例
        self.hand = hand_instance
        if not simulation and hand_instance is None and ORCA_HAND_AVAILABLE:
            try:
                # 自动创建OrcaHand实例 - 关键修改：不依赖config_path
                # 直接创建OrcaHand实例，不传递config_path，避免校准依赖
                self.hand = OrcaHand()
                print("✅ 自动创建OrcaHand实例（跳过配置文件依赖）")
            except Exception as e:
                print(f"❌ 自动创建OrcaHand失败: {e}")
                self.simulation = True

        # 从模式管理器获取电机映射信息
        self.motor_ids = self.mode_manager.get_motor_ids()
        self.joint_to_motor_map = self.mode_manager.get_joint_to_motor_map()
        self.motor_to_joint_map = self.mode_manager.get_motor_to_joint_map()
        self.motor_id_to_idx_dict = self.mode_manager.get_motor_id_to_idx_dict()

        # 状态变量
        self.current_mode = None
        self.is_moving = False
        self.motion_direction = None  # 'closing' 或 'opening'
        self.motion_thread = None
        self.stop_motion = threading.Event()

        # 线程安全锁
        self.angles_lock = threading.RLock()

        # 角度状态 - 使用16关节
        self.current_angles = {}
        self.target_angles = {}
        self.initial_angles = {}  # 初始位置（张开状态）

        # 运动参数
        self.step_size = 1.0
        self.motion_speed = 0.5
        self.max_steps = 50

        # 扭矩保持相关状态
        self._is_torque_holding = False
        self._torque_hold_timer = None
        self.current_torque_params = {}  # 当前模式的扭矩参数

        # 调试计数器
        self._debug_counter = 0

        # 初始化
        if self.simulation:
            self._init_simulation()
            print("✅ 抓取控制器：模拟模式已启动")
        else:
            if self.hand is None:
                print("❌ 硬件模式下必须提供hand_instance参数或OrcaHand可用")
                self.simulation = True
                self._init_simulation()
            else:
                self._init_hardware()
                print("✅ 抓取控制器：硬件模式已启动")

    def _init_simulation(self):
        """初始化模拟模式 - 使用16关节结构"""
        # 关键修改：使用mode.py中的中立位数据，而不是硬编码
        self.initial_angles = self.mode_manager.default_neutral.copy()
        with self.angles_lock:
            self.current_angles = self.initial_angles.copy()
            self.target_angles = self.initial_angles.copy()

        print("🎮 模拟模式：使用16关节虚拟手部进行测试")
        print(f"📊 使用中立位角度: {self._format_angles_for_display(self.initial_angles)}")

    def _init_hardware(self):
        """初始化硬件模式 - 修复版本：确保真正连接硬件"""
        try:
            # 检查hand实例是否有必要的方法
            if not hasattr(self.hand, 'get_joint_pos') or not hasattr(self.hand, 'set_joint_pos'):
                print("❌ 提供的hand实例缺少必要方法")
                raise AttributeError("hand_instance缺少必要方法")

            # 关键修复：确保调用connect方法
            if not self.hand.is_connected():
                print("🔌 尝试连接手部硬件...")
                success, message = self.hand.connect()
                if not success:
                    print(f"❌ 硬件连接失败: {message}")
                    raise ConnectionError(f"硬件连接失败: {message}")
                print("✅ 手部硬件连接成功")
            else:
                print("✅ 手部硬件已连接")

            # 启用扭矩
            print("🔧 启用电机扭矩...")
            self.hand.enable_torque()

            # 设置控制模式
            print("🎛️ 设置控制模式...")
            self.hand.set_control_mode('position')

            # 使用mode.py中的中立位作为初始角度
            with self.angles_lock:
                self.initial_angles = self.mode_manager.default_neutral.copy()
                self.current_angles = self.initial_angles.copy()
                self.target_angles = self.initial_angles.copy()

            print(f"✅ 硬件模式初始化完成")
            print(f"📊 使用中立位角度: {self._format_angles_for_display(self.initial_angles)}")

            # 尝试从硬件读取当前角度
            try:
                joint_pos = self.hand.get_joint_pos(as_list=False)
                if joint_pos and all(v is not None for v in joint_pos.values()):
                    with self.angles_lock:
                        self.current_angles = joint_pos
                    print(f"📊 从硬件读取当前角度: {self._format_angles_for_display(joint_pos)}")
                else:
                    print("⚠️ 从硬件读取的角度包含None值，使用中立位")
            except Exception as e:
                print(f"⚠️ 无法从硬件读取角度: {e}")

        except Exception as e:
            print(f"❌ 硬件初始化失败: {e}")
            self.simulation = True
            self._init_simulation()
            print("🔄 已自动切换到模拟模式")

    def _format_angles_for_display(self, angles):
        """格式化角度显示，只显示关键关节"""
        key_joints = ['thumb_mcp', 'index_mcp', 'middle_mcp', 'ring_mcp', 'pinky_mcp']
        formatted = {}
        for joint in key_joints:
            if joint in angles:
                # 转换为度显示，更易读
                angle_deg = math.degrees(angles[joint])
                formatted[joint] = f"{angle_deg:.1f}°"
        return formatted

    def _set_default_angles(self):
        """设置默认角度值 - 使用mode.py中的中立位"""
        # 关键修改：使用mode.py中的中立位
        default_angles = self.mode_manager.default_neutral.copy()
        with self.angles_lock:
            self.initial_angles = default_angles.copy()
            self.current_angles = default_angles.copy()
            self.target_angles = default_angles.copy()

    def set_motor_positions_direct(self, motor_positions: Dict[int, float]):
        """直接设置电机位置，完全绕过关节角度转换 - 最终修复版本"""
        if self.simulation:
            # 在模拟模式下也更新当前角度状态，以供状态查询使用
            with self.angles_lock:
                for motor_id, position in motor_positions.items():
                    joint_name = self.motor_to_joint_map.get(motor_id)
                    if joint_name:
                        # 模拟更新角度
                        self.current_angles[joint_name] = position
            return True

        # 硬件模式下的实际操作
        if self.hand:
            try:
                # 直接使用 hand 的 set_motor_positions_direct 方法
                return self.hand.set_motor_positions_direct(motor_positions)
            except Exception as e:
                print(f"❌ 硬件操作失败 (set_motor_positions_direct): {e}")
                import traceback
                traceback.print_exc()
                return False
        else:
            print("❌ 硬件模式下hand实例不可用")
            return False

    def get_motor_positions_dict(self) -> Dict[int, float]:
        """获取当前电机位置字典"""
        if self.simulation:
            # 模拟模式下从当前角度状态构建
            motor_positions = {}
            for joint, angle in self.current_angles.items():
                motor_id = self.joint_to_motor_map.get(joint)
                if motor_id:
                    motor_positions[motor_id] = angle
            return motor_positions
        else:
            if self.hand and hasattr(self.hand, 'get_motor_positions_dict'):
                return self.hand.get_motor_positions_dict()
            else:
                return {}

    def set_mode(self, mode_id: int) -> bool:
        try:
            # 如果正在运动，先停止
            if self.is_moving:
                self.stop_and_hold()
                time.sleep(0.1)

            mode_info = self.mode_manager.get_mode(mode_id)
            if mode_info:
                self.current_mode = mode_id
                target_angles_rad = mode_info['target_angles']

                with self.angles_lock:
                    self.target_angles = target_angles_rad.copy()

                # 更新运动参数
                motion_params = mode_info['motion_params']
                self.step_size = motion_params['step_size']
                self.motion_speed = motion_params['approach_speed']
                self.max_steps = motion_params.get('max_steps', 50)

                # 保存扭矩参数供后续使用
                self.current_torque_params = mode_info.get('torque_params', {})

                print(f"🎯 已切换到模式 {mode_id}: {mode_info['name']}")
                # _format_angles_for_display 内部会将弧度转为度数显示，方便人类阅读，这里不需要改
                print(f"📊 目标角度: {self._format_angles_for_display(target_angles_rad)}")
                print(f"🔧 扭矩设置: {self.current_torque_params.get('hold_torque', 400)}")
                return True
            else:
                print(f"❌ 模式 {mode_id} 设置失败")
                return False

        except Exception as e:
            print(f"❌ 设置模式时出错: {e}")
            return False

    def start_grasping(self):
        """开始抓取运动 - 逐步闭合手指到目标角度"""
        if self.is_moving:
            print("⚠️ 已有运动在进行，先停止当前运动")
            self.stop_and_hold()
            time.sleep(0.2)

        self.stop_motion.clear()
        self.motion_direction = 'closing'
        self.is_moving = True

        # 在新线程中运行抓取运动
        self.motion_thread = threading.Thread(target=self._grasp_motion)
        self.motion_thread.daemon = True
        self.motion_thread.start()
        print("🤏 开始抓取运动...")

    def start_releasing(self):
        """开始释放运动 - 逐步张开手指到初始位置"""
        if self.is_moving:
            print("⚠️ 已有运动在进行，先停止当前运动")
            self.stop_and_hold()
            time.sleep(0.2)

        self.stop_motion.clear()
        self.motion_direction = 'opening'
        self.is_moving = True

        # 在新线程中运行释放运动
        self.motion_thread = threading.Thread(target=self._release_motion)
        self.motion_thread.daemon = True
        self.motion_thread.start()
        print("🖐️ 开始释放运动...")

    def stop_and_hold(self):
        """停止运动并保持当前位置"""
        self.stop_motion.set()
        self.is_moving = False
        # 不重置motion_direction，以便知道上次的运动方向

        # 等待运动线程结束
        if self.motion_thread and self.motion_thread.is_alive():
            self.motion_thread.join(timeout=1.0)

        print("⏸️ 运动已停止，保持当前位置")

    def realtime_close_step(self):
        """实时闭合一步 - 静默版"""

        with self.angles_lock:
            current_angles = self.current_angles.copy()
            target_angles = self.target_angles.copy()
            # 1. 使用控制器中设置的步长（从 set_mode 读取）
            step_size_rad = self.step_size

        # 检查是否已经达到目标角度
        if self._has_reached_target(current_angles, target_angles):
            # print("✅ 已到达目标角度，停止闭合")
            return True

        # 计算每个关节的移动步长（弧度）
        new_angles = {}

        for joint, current in current_angles.items():
            target = target_angles.get(joint, current)
            if current < target:
                # 使用配置的步长
                new_angles[joint] = min(current + step_size_rad, target)
            elif current > target:
                # 使用配置的步长
                new_angles[joint] = max(current - step_size_rad, target)
            else:
                new_angles[joint] = current

        # 验证新角度安全性
        is_valid, message = self.mode_manager.validate_angles(new_angles)
        if not is_valid:
            print(f"⚠️ 角度安全性警告: {message}")
            return False

        # 使用直接电机控制
        motor_positions = {}
        for joint, angle in new_angles.items():
            motor_id = self.joint_to_motor_map.get(joint)
            if motor_id:
                motor_positions[motor_id] = angle

        if motor_positions:
            success = self.set_motor_positions_direct(motor_positions)
            if success:
                # 更新当前角度状态
                with self.angles_lock:
                    self.current_angles = new_angles.copy()
                return True
            else:
                print("❌ 直接电机控制失败")
                return False
        else:
            print("❌ 没有找到对应的电机映射")
            return False

    def realtime_open_step(self):
        """实时张开一步 - 使用直接电机控制"""

        with self.angles_lock:
            current_angles = self.current_angles.copy()
            initial_angles = self.initial_angles.copy()

        # 检查是否已经达到初始角度
        if self._has_reached_initial(current_angles, initial_angles):
            # print("✅ 已到达初始角度，停止张开")
            return True

        # 计算每个关节的移动步长（弧度）
        step_size = math.radians(5.0)  # 增加步长到5度
        new_angles = {}

        for joint, current in current_angles.items():
            initial = initial_angles.get(joint, current)
            if current > initial:
                new_angles[joint] = max(current - step_size, initial)
            elif current < initial:
                new_angles[joint] = min(current + step_size, initial)
            else:
                new_angles[joint] = current

        # 验证新角度安全性
        is_valid, message = self.mode_manager.validate_angles(new_angles)
        if not is_valid:
            print(f"⚠️ 角度安全性警告: {message}")
            return False

        # 🔥 关键修改：使用直接电机控制
        motor_positions = {}
        for joint, angle in new_angles.items():
            motor_id = self.joint_to_motor_map.get(joint)
            if motor_id:
                motor_positions[motor_id] = angle

        if motor_positions:
            # print(f"🔌 使用直接电机控制，设置 {len(motor_positions)} 个电机")
            success = self.set_motor_positions_direct(motor_positions)
            if success:
                # 更新当前角度状态
                with self.angles_lock:
                    self.current_angles = new_angles.copy()
                return True
            else:
                print("❌ 直接电机控制失败")
                return False
        else:
            print("❌ 没有找到对应的电机映射")
            return False

    def _has_reached_target(self, current_angles, target_angles):
        """检查是否已达到目标角度（弧度）"""
        tolerance = math.radians(1.0)  # 容差，1度对应的弧度
        for joint, current in current_angles.items():
            target = target_angles.get(joint, current)
            if abs(current - target) > tolerance:
                return False
        return True

    def _has_reached_initial(self, current_angles, initial_angles):
        """检查是否已达到初始角度（弧度）"""
        tolerance = math.radians(1.0)  # 容差，1度对应的弧度
        for joint, current in current_angles.items():
            initial = initial_angles.get(joint, current)
            if abs(current - initial) > tolerance:
                return False
        return True

    def emergency_stop(self):
        """紧急停止所有运动"""
        self.stop_motion.set()
        self.is_moving = False
        self.motion_direction = None

        # 等待运动线程结束
        if self.motion_thread and self.motion_thread.is_alive():
            self.motion_thread.join(timeout=0.5)

        # 如果是硬件模式，发送急停命令
        if not self.simulation and self.hand:
            try:
                if hasattr(self.hand, 'disable_torque'):
                    self.hand.disable_torque()
                    print("🔒 硬件扭矩已禁用")
            except Exception as e:
                print(f"⚠️ 急停命令发送失败: {e}")

        print("🛑 紧急停止！所有运动已终止")

    # ==================== 扭矩控制方法 ====================

    def start_torque_hold(self):
        """简化的扭矩保持方法 - 使用当前模式的扭矩参数"""
        try:
            # 取消之前的扭矩保持
            if self._is_torque_holding:
                self._cancel_torque_hold()
                time.sleep(0.1)

            # 获取当前模式的扭矩参数
            if self.current_mode:
                torque_params = self.mode_manager.get_mode_torque_params(self.current_mode)
                torque_limit = torque_params.get('hold_torque', 400)
                duration = torque_params.get('auto_hold_duration', 60.0)
            else:
                torque_limit = 400
                duration = 60.0

            print(f"🔧 启动扭矩保持: 限制={torque_limit}, 时长={duration}秒")

            # 使用修复后的方法
            self.start_timed_torque_hold(
                motor_ids=self.motor_ids,
                torque_limit=torque_limit,
                duration=duration
            )

        except Exception as e:
            print(f"❌ 启动扭矩保持失败: {e}")

    def start_timed_torque_hold(self, motor_ids: list = None,
                                torque_limit: int = 400,
                                duration: float = 60.0):
        """启动限时扭矩保持 - 简化版本，避免格式问题"""
        if motor_ids is None:
            motor_ids = self.motor_ids

        # 停止当前运动
        if self.is_moving:
            self.stop_and_hold()
            time.sleep(0.1)

        print("\n" + "=" * 40)
        print(f"⏹️ 运动停止，启动扭矩保持")
        print(f"🔧 扭矩限制: {torque_limit}, 时长: {duration}秒")
        print("=" * 40)

        if self.simulation:
            print(f"🎮 模拟模式: 启动 {duration}秒扭矩保持")
            return

        try:
            # ========== 简化方案：直接使用set_motor_positions_direct ==========
            # 获取当前位置
            with self.angles_lock:
                current_angles = self.current_angles.copy()

            # 转换为电机位置字典
            motor_positions = {}
            for joint_name, angle in current_angles.items():
                motor_id = self.joint_to_motor_map.get(joint_name)
                if motor_id and motor_id in motor_ids:
                    motor_positions[motor_id] = angle

            print(f"🔧 锁定 {len(motor_positions)} 个电机的位置")

            # 设置扭矩限制
            self.set_torque_limit_direct(motor_ids, torque_limit)

            # 使用现有的set_motor_positions_direct方法设置位置
            # 这个方法已经经过测试，不会出现格式问题
            if motor_positions:
                success = self.set_motor_positions_direct(motor_positions)
                if success:
                    print("✅ 位置锁定成功，扭矩保持已启动")
                else:
                    print("❌ 位置锁定失败")
            else:
                print("⚠️ 没有找到有效的电机位置")

            print(f"⏱️ 已启动 {duration}秒扭矩保持，扭矩限制 {torque_limit}")

            # 启动定时取消
            self._torque_hold_timer = threading.Timer(duration, self._cancel_torque_hold)
            self._torque_hold_timer.start()

            self._is_torque_holding = True

        except Exception as e:
            print(f"❌ 启动扭矩保持失败: {e}")
            import traceback
            traceback.print_exc()

    def _cancel_torque_hold(self):
        """取消扭矩保持"""
        if not self._is_torque_holding:
            return

        try:
            # 恢复最大扭矩
            self.set_torque_limit_direct(self.motor_ids, 1000)
            self._is_torque_holding = False

            if self._torque_hold_timer:
                self._torque_hold_timer.cancel()
                self._torque_hold_timer = None

            print("🔓 扭矩保持已取消")

        except Exception as e:
            print(f"❌ 取消扭矩保持失败: {e}")

    def set_torque_limit_direct(self, motor_ids: list, torque_limit: int):
        """直接设置扭矩限制值 - 修复版本"""
        if not (0 <= torque_limit <= 1000):
            raise ValueError("Torque limit must be between 0 and 1000")

        if self.simulation:
            # 显示涉及的关节名称
            joint_names = [self.motor_to_joint_map.get(mid, f"未知({mid})") for mid in motor_ids]
            print(f"🎮 模拟模式: 设置关节 {joint_names} 扭矩限制为 {torque_limit}")
            return

        try:
            # 通过hand实例设置扭矩限制
            if self.hand and hasattr(self.hand, '_dxl_client'):
                success_count = 0
                for motor_id in motor_ids:
                    joint_name = self.motor_to_joint_map.get(motor_id, f"未知({motor_id})")
                    try:
                        # 使用Dynamixel协议直接设置扭矩限制
                        result, error = self.hand._dxl_client.protocol.write2ByteTxRx(motor_id, 48, torque_limit)
                        if result == 0:
                            success_count += 1
                            print(f"✅ 设置关节 {joint_name}(电机{motor_id}) 扭矩限制为 {torque_limit}")
                        else:
                            print(f"❌ 设置关节 {joint_name}(电机{motor_id}) 扭矩限制失败: 错误码 {result}")
                    except Exception as e:
                        print(f"❌ 设置电机 {motor_id} 扭矩限制异常: {e}")

                print(f"📊 扭矩设置完成: {success_count}/{len(motor_ids)} 个电机成功")
            else:
                print("❌ 无法访问硬件客户端")
        except Exception as e:
            print(f"❌ 设置扭矩限制失败: {e}")

    def cancel_torque_hold_on_other_actions(self):
        """在其他操作时取消扭矩保持"""
        if self._is_torque_holding:
            print("🔄 检测到其他操作，取消扭矩保持")
            self._cancel_torque_hold()

    def get_motor_pos(self):
        """获取电机位置"""
        if self.simulation:
            # 模拟模式下，将关节角度转换为电机位置
            joint_angles = self.get_current_angles()
            motor_positions = []
            for motor_id in self.motor_ids:
                joint_name = self.motor_to_joint_map.get(motor_id)
                if joint_name and joint_name in joint_angles:
                    motor_positions.append(joint_angles[joint_name])
                else:
                    motor_positions.append(0)  # 默认值
            return motor_positions
        else:
            # 硬件模式下从hand实例获取
            if self.hand and hasattr(self.hand, 'get_motor_pos'):
                positions = self.hand.get_motor_pos()
                # 确保返回的是列表
                if isinstance(positions, list):
                    return positions
                else:
                    print(f"⚠️ get_motor_pos 返回了非列表类型: {type(positions)}")
                    return [0] * len(self.motor_ids)
            else:
                # 备用方案
                return [0] * len(self.motor_ids)

    # ==================== 原有运动控制方法 ====================

    def _grasp_motion(self):
        """抓取运动线程函数 - 逐步闭合到目标角度"""
        step_count = 0

        while (not self.stop_motion.is_set() and
               step_count < self.max_steps and
               self._need_more_closing()):

            try:
                # 计算下一步的角度
                new_angles = self._calculate_next_step('closing')

                # 应用角度变化
                self._set_joint_angles(new_angles)

                # 更新进度显示
                progress = self._calculate_progress()
                if step_count % 10 == 0:  # 每10步打印一次进度，减少输出频率
                    print(f"📈 抓取进度: {progress:.1%} (步骤 {step_count}/{self.max_steps})")

                step_count += 1
                time.sleep(0.1 / self.motion_speed)

            except Exception as e:
                print(f"❌ 抓取运动出错: {e}")
                break

        self.is_moving = False
        if not self.stop_motion.is_set():
            print("✅ 抓取运动完成")
        else:
            print("⏹️ 抓取运动被中断")

    def _release_motion(self):
        """释放运动线程函数 - 逐步张回到初始位置"""
        step_count = 0

        while (not self.stop_motion.is_set() and
               step_count < self.max_steps and
               self._need_more_opening()):

            try:
                # 计算下一步的角度
                new_angles = self._calculate_next_step('opening')

                # 应用角度变化
                self._set_joint_angles(new_angles)

                # 更新进度显示
                progress = self._calculate_progress()
                if step_count % 10 == 0:  # 每10步打印一次进度，减少输出频率
                    print(f"📈 释放进度: {progress:.1%} (步骤 {step_count}/{self.max_steps})")

                step_count += 1
                time.sleep(0.1 / self.motion_speed)

            except Exception as e:
                print(f"❌ 释放运动出错: {e}")
                break

        self.is_moving = False
        if not self.stop_motion.is_set():
            print("✅ 释放运动完成")
        else:
            print("⏹️ 释放运动被中断")

    def _calculate_next_step(self, direction: str) -> Dict[str, float]:
        """计算下一步的角度 - 16关节版本"""
        with self.angles_lock:
            current_angles = self.current_angles.copy()
            target_angles = self.target_angles.copy()
            initial_angles = self.initial_angles.copy()

        new_angles = current_angles.copy()

        for joint, current_angle in current_angles.items():
            if direction == 'closing':
                target_angle = target_angles.get(joint, current_angle)
                # 向目标角度移动
                if current_angle < target_angle:
                    new_angles[joint] = min(current_angle + self.step_size, target_angle)
                elif current_angle > target_angle:
                    # 如果当前角度超过目标，稍微回退
                    new_angles[joint] = max(current_angle - self.step_size, target_angle)
                else:
                    new_angles[joint] = current_angle

            elif direction == 'opening':
                initial_angle = initial_angles.get(joint, 0)
                # 向初始角度移动
                if current_angle > initial_angle:
                    new_angles[joint] = max(current_angle - self.step_size, initial_angle)
                elif current_angle < initial_angle:
                    # 如果当前角度小于初始，稍微前进
                    new_angles[joint] = min(current_angle + self.step_size, initial_angle)
                else:
                    new_angles[joint] = current_angle

        return new_angles

    def _need_more_closing(self) -> bool:
        """检查是否需要继续闭合 - 16关节版本"""
        with self.angles_lock:
            current_angles = self.current_angles.copy()
            target_angles = self.target_angles.copy()

        total_diff = 0
        for joint, current_angle in current_angles.items():
            target_angle = target_angles.get(joint, current_angle)
            total_diff += abs(current_angle - target_angle)

        # 当所有关节的平均差异小于阈值时停止
        avg_diff = total_diff / len(current_angles)
        return avg_diff > 0.5  # 平均差异大于0.5度继续运动

    def _need_more_opening(self) -> bool:
        """检查是否需要继续张开 - 16关节版本"""
        with self.angles_lock:
            current_angles = self.current_angles.copy()
            initial_angles = self.initial_angles.copy()

        total_diff = 0
        for joint, current_angle in current_angles.items():
            initial_angle = initial_angles.get(joint, 0)
            total_diff += abs(current_angle - initial_angle)

        # 当所有关节的平均差异小于阈值时停止
        avg_diff = total_diff / len(current_angles)
        return avg_diff > 0.5  # 平均差异大于0.5度继续运动

    def _calculate_progress(self) -> float:
        """计算运动进度 (0-1) - 16关节版本"""
        with self.angles_lock:
            current_angles = self.current_angles.copy()
            target_angles = self.target_angles.copy()
            initial_angles = self.initial_angles.copy()

        if self.motion_direction == 'closing':
            total_distance = 0
            current_distance = 0

            for joint in current_angles:
                initial = initial_angles.get(joint, 0)
                target = target_angles.get(joint, initial)
                current = current_angles.get(joint, initial)

                total_distance += abs(target - initial)
                current_distance += abs(current - initial)

            if total_distance == 0:
                return 1.0
            return min(1.0, current_distance / total_distance)

        elif self.motion_direction == 'opening':
            total_distance = 0
            current_distance = 0

            for joint in current_angles:
                initial = initial_angles.get(joint, 0)
                target = target_angles.get(joint, initial)
                current = current_angles.get(joint, initial)

                total_distance += abs(current_angles.get(joint, 0) - initial)
                current_distance += abs(current - initial)

            if total_distance == 0:
                return 1.0
            return min(1.0, current_distance / total_distance)

        return 1.0

    def _set_joint_angles(self, angles: Dict[str, float]):
        """
        设置关节角度 - 使用直接电机控制版本 (修正：移除冗余步进打印)
        """

        # 验证角度安全性
        is_valid, message = self.mode_manager.validate_angles(angles)
        if not is_valid:
            print(f"⚠️ 角度安全性警告: {message}")
            return

        # 更新当前角度
        with self.angles_lock:
            self.current_angles = angles.copy()

        if self.simulation:
            return

        try:
            # 将关节角度转换为电机位置
            motor_positions = {}
            for joint, angle in angles.items():
                motor_id = self.joint_to_motor_map.get(joint)
                if motor_id:
                    motor_positions[motor_id] = angle

            if motor_positions:
                # print(f"🔌 直接控制 {len(motor_positions)} 个电机: {motor_positions}") # <-- 原始冗长打印已注释/删除
                success = self.set_motor_positions_direct(motor_positions)
                if success:
                    pass  # 移除原始代码中的延迟和验证打印
                else:
                    print("❌ 直接电机控制失败")
            else:
                print("❌ 没有找到对应的电机映射")

        except Exception as e:
            print(f"❌ 直接电机控制异常: {e}")
            import traceback
            traceback.print_exc()

    def get_current_angles(self) -> Dict[str, float]:
        """获取当前16关节角度"""
        if not self.simulation and self.hand:
            try:
                # 从硬件读取实际角度
                hardware_angles = self.hand.get_joint_pos(as_list=False)
                if hardware_angles and all(v is not None for v in hardware_angles.values()):
                    with self.angles_lock:
                        self.current_angles = hardware_angles
            except Exception as e:
                print(f"⚠️ 无法从硬件读取角度: {e}")

        with self.angles_lock:
            return self.current_angles.copy()

    def get_status(self) -> Dict:
        """获取控制器状态"""
        current_angles = self.get_current_angles()

        return {
            'simulation': self.simulation,
            'current_mode': self.current_mode,
            'is_moving': self.is_moving,
            'motion_direction': self.motion_direction,
            'current_angles': current_angles,
            'target_angles': self.target_angles.copy(),
            'initial_angles': self.initial_angles.copy(),
            'progress': self._calculate_progress(),
            'is_torque_holding': self._is_torque_holding
        }

    def get_motor_status_info(self):
        """获取电机状态信息"""
        status_info = {}

        if self.simulation:
            # 模拟模式返回假数据
            for motor_id in self.motor_ids:
                joint_name = self.motor_to_joint_map.get(motor_id, "未知")
                status_info[motor_id] = {
                    'joint': joint_name,
                    'torque_limit': 1000,  # 模拟值
                    'position': 0,  # 模拟值
                    'temperature': 25,  # 模拟值
                    'current': 0,  # 模拟值
                    'voltage': 12.0,  # 模拟值
                    'moving': False  # 模拟值
                }
        else:
            # 硬件模式获取真实数据
            try:
                if self.hand:
                    # 获取扭矩限制
                    torque_limits = {}
                    for motor_id in self.motor_ids:
                        torque_value, result, error = self.hand._dxl_client.protocol.read2ByteTxRx(motor_id, 48)
                        if result == 0:
                            torque_limits[motor_id] = torque_value
                        else:
                            torque_limits[motor_id] = 0

                    # 获取位置
                    positions = self.hand.get_motor_pos()

                    # 获取温度
                    temperatures = self.hand.get_motor_temp()

                    # 获取电流（如果支持）
                    currents = {}
                    for motor_id in self.motor_ids:
                        try:
                            current_value, result, error = self.hand._dxl_client.protocol.read2ByteTxRx(motor_id, 69)
                            if result == 0:
                                currents[motor_id] = current_value * 6.5  # 转换为mA
                            else:
                                currents[motor_id] = 0
                        except:
                            currents[motor_id] = 0

                    # 获取电压
                    voltages = {}
                    for motor_id in self.motor_ids:
                        try:
                            voltage_value, result, error = self.hand._dxl_client.protocol.read1ByteTxRx(motor_id, 62)
                            if result == 0:
                                voltages[motor_id] = voltage_value * 0.1  # 转换为V
                            else:
                                voltages[motor_id] = 0
                        except:
                            voltages[motor_id] = 0

                    # 获取移动状态
                    moving_status = {}
                    for motor_id in self.motor_ids:
                        try:
                            moving_value, result, error = self.hand._dxl_client.protocol.read1ByteTxRx(motor_id, 66)
                            if result == 0:
                                moving_status[motor_id] = bool(moving_value)
                            else:
                                moving_status[motor_id] = False
                        except:
                            moving_status[motor_id] = False

                    # 组合所有信息
                    for i, motor_id in enumerate(self.motor_ids):
                        joint_name = self.motor_to_joint_map.get(motor_id, "未知")
                        status_info[motor_id] = {
                            'joint': joint_name,
                            'torque_limit': torque_limits.get(motor_id, 0),
                            'position': positions[i] if i < len(positions) else 0,
                            'temperature': temperatures[i] if i < len(temperatures) else 0,
                            'current': currents.get(motor_id, 0),
                            'voltage': voltages.get(motor_id, 0),
                            'moving': moving_status.get(motor_id, False)
                        }
            except Exception as e:
                print(f"❌ 获取电机状态失败: {e}")
        return status_info

    def switch_to_hardware(self, hand_instance):
        """
        从模拟模式切换到硬件模式

        Args:
            hand_instance: OrcaHand实例
        """
        if self.is_moving:
            self.emergency_stop()
            time.sleep(0.5)

        self.simulation = False
        self.hand = hand_instance
        self._init_hardware()
        print("🔄 已切换到硬件模式")

    def switch_to_simulation(self):
        """从硬件模式切换到模拟模式"""
        if self.is_moving:
            self.emergency_stop()
            time.sleep(0.5)

        self.simulation = True
        self.hand = None
        self._init_simulation()
        print("🔄 已切换到模拟模式")


# 全局控制器实例
_global_controller = None


def get_grasp_controller(simulation=True, hand_instance=None):
    """获取全局抓取控制器实例"""
    global _global_controller
    if _global_controller is None:
        _global_controller = GraspController(simulation=simulation, hand_instance=hand_instance)
    return _global_controller


def print_controller_status():
    """打印控制器状态"""
    controller = get_grasp_controller()
    status = controller.get_status()

    print("\n" + "=" * 50)
    print("           抓取控制器状态")
    print("=" * 50)
    print(f"模式: {'模拟' if status['simulation'] else '硬件'}")
    print(f"当前模式: {status['current_mode']}")
    print(f"运动状态: {'运动中' if status['is_moving'] else '静止'}")
    print(f"运动方向: {status['motion_direction'] or '无'}")
    print(f"扭矩保持: {'激活' if status['is_torque_holding'] else '未激活'}")

    # 只显示关键关节角度
    key_joints = ['thumb_mcp', 'index_mcp', 'middle_mcp']
    current_key = {k: status['current_angles'].get(k, 0) for k in key_joints}
    target_key = {k: status['target_angles'].get(k, 0) for k in key_joints}

    # 转换为度显示
    current_key_deg = {k: math.degrees(v) for k, v in current_key.items()}
    target_key_deg = {k: math.degrees(v) for k, v in target_key.items()}

    print(f"当前关键角度: {current_key_deg}")
    print(f"目标关键角度: {target_key_deg}")
    print(f"进度: {status['progress']:.1%}")
    print("=" * 50)


# 测试代码
if __name__ == "__main__":
    print("🧪 抓取控制器测试...")

    # 创建模拟控制器进行测试
    controller = GraspController(simulation=True)

    # 测试模式切换
    print("\n1. 测试模式切换...")
    controller.set_mode(1)
    print_controller_status()

    # 测试抓取运动（短暂运行）
    print("\n2. 测试抓取运动...")
    controller.start_grasping()
    time.sleep(2)
    controller.stop_and_hold()

    print_controller_status()

    # 测试释放运动
    print("\n3. 测试释放运动...")
    controller.start_releasing()
    time.sleep(2)
    controller.stop_and_hold()

    print_controller_status()

    print("\n✅ 16关节控制器测试完成")