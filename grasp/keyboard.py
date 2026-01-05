import threading
import time
import math
from typing import Optional, Callable
from pynput import keyboard

# 修正导入路径 - 假设从当前包导入controller模块
try:
    from .controller import get_grasp_controller, GraspController
except ImportError as e:
    print(f"错误: 无法导入controller模块: {e}")


    class GraspController:
        def __init__(self, simulation=True):
            self.simulation = simulation
            self._is_torque_holding = False
            self.current_torque_params = {}
            self.motor_ids = list(range(1, 17))
            self.motor_to_joint_map = {}
            self.current_mode = 1  # 添加current_mode属性

        def set_mode(self, mode_id):
            print(f"[Dummy] Mode set to {mode_id}")
            self.current_mode = mode_id
            return True

        def realtime_close_step(self):
            print("[Dummy] Closing step")

        def realtime_open_step(self):
            print("[Dummy] Opening step")

        def emergency_stop(self):
            print("[Dummy] Emergency stop")

        def trigger_calibration(self):
            print("[Dummy] Calibration triggered")

        def get_status(self):
            return {
                'simulation': self.simulation,
                'current_mode': self.current_mode,
                'is_moving': False,
                'motion_direction': None,
                'current_angles': {},
                'target_angles': {},
                'initial_angles': {},
                'progress': 0.5,
                'is_torque_holding': self._is_torque_holding
            }

        def start_timed_torque_hold(self, **kwargs):
            print("[Dummy] Torque hold started")
            self._is_torque_holding = True

        def cancel_torque_hold_on_other_actions(self):
            print("[Dummy] Torque hold cancelled")
            self._is_torque_holding = False

        def get_motor_status_info(self):
            print("[Dummy] Getting motor status")
            return {}


    def get_grasp_controller(controller=None):
        return GraspController()


class KeyboardController:
    def __init__(self, controller=None):
        self.controller: GraspController = controller or get_grasp_controller()
        self.listener: Optional[keyboard.Listener] = None
        self.is_running: bool = False

        # 按键状态
        self.enter_pressed: bool = False
        self.backspace_pressed: bool = False

        # 控制线程
        self.control_thread: Optional[threading.Thread] = None
        self.control_interval: float = 0.1  # 控制循环间隔(秒)

        # 回调函数
        self.control_callbacks: dict = {
            'on_mode_change': None,
            'on_emergency_stop': None,
            'on_exit': None,
            'on_torque_hold_start': None,
            'on_torque_hold_cancel': None
        }

        print("键盘控制器已初始化")

    def start(self) -> bool:
        if self.is_running:
            print("键盘控制器已在运行中。")
            return True

        try:
            self.is_running = True
            self.listener = keyboard.Listener(
                on_press=self._on_key_press,
                on_release=self._on_key_release
            )
            self.listener.start()

            self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
            self.control_thread.start()

            self._print_controls()
            print("键盘控制已启动")
            return True
        except Exception as e:
            print(f"启动键盘控制失败: {e}")
            self.is_running = False
            return False

    def stop(self) -> None:
        if not self.is_running:
            return

        self.is_running = False

        if self.listener:
            self.listener.stop()
            self.listener = None

        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)

        print("键盘控制已停止")

    def _on_key_press(self, key) -> bool:
        try:
            char = key.char
        except AttributeError:
            char = None

        cancel_torque_conditions = [
            key == keyboard.Key.space,
            key == keyboard.Key.backspace,
            char in ['t', 'T', 's', 'S', 'c', 'C']
        ]

        if any(cancel_torque_conditions):
            self.controller.cancel_torque_hold_on_other_actions()
            if self.control_callbacks['on_torque_hold_cancel']:
                self.control_callbacks['on_torque_hold_cancel']()

        if char in ('1', '2', '3', '4','5','6','7'):
            mode_id = int(char)
            print(f"键盘: 检测到数字键 '{char}'，切换到模式 {mode_id}")

            # 添加模式切换前的状态检查
            current_mode_before = self.controller.current_mode
            print(f"切换前当前模式: {current_mode_before}")

            success = self.controller.set_mode(mode_id)

            # 添加模式切换后的状态检查
            current_mode_after = self.controller.current_mode
            print(f"切换后当前模式: {current_mode_after}")

            if success and current_mode_after == mode_id:
                print(f"键盘: 成功切换到模式 {mode_id}")
                if self.control_callbacks['on_mode_change']:
                    self.control_callbacks['on_mode_change'](mode_id)
            else:
                print(f"键盘: 切换到模式 {mode_id} 失败")
            return True

        elif key == keyboard.Key.enter:
            if not self.enter_pressed:
                self.enter_pressed = True
                self.backspace_pressed = False  # 确保互斥
                print("键盘: Enter键按下 - 开始闭合")
            return True

        elif key == keyboard.Key.backspace:
            if not self.backspace_pressed:
                self.backspace_pressed = True
                self.enter_pressed = False  # 确保互斥
                print("键盘: Backspace键按下 - 开始张开")
            return True

        elif key == keyboard.Key.space:
            print("键盘: Space键按下 - 紧急停止")
            self.controller.emergency_stop()
            if self.control_callbacks['on_emergency_stop']:
                self.control_callbacks['on_emergency_stop']()
            return True

        # ==================== 功能按键 ====================
        elif char in ('s', 'S'):
            print("键盘: S键按下 - 显示电机状态")
            self._show_motor_status()
            return True

        elif char in ('t', 'T'):
            print("键盘: T键按下 - 手动启动扭矩保持")
            torque_limit = 400
            duration = 60.0

            # 尝试使用当前模式的扭矩参数
            if hasattr(self.controller, 'current_torque_params'):
                torque_params = self.controller.current_torque_params
                torque_limit = torque_params.get('hold_torque', 400)
                duration = torque_params.get('auto_hold_duration', 60.0)

            self.controller.start_timed_torque_hold(
                torque_limit=torque_limit,
                duration=duration
            )
            if self.control_callbacks['on_torque_hold_start']:
                self.control_callbacks['on_torque_hold_start'](torque_limit, duration)
            return True

        elif char in ('c', 'C'):
            print("键盘: C键按下 - 触发校准")
            if hasattr(self.controller, 'trigger_calibration'):
                self.controller.trigger_calibration()
            return True

        elif key == keyboard.Key.esc or char in ('q', 'Q'):
            print("键盘: 退出键按下")
            self._handle_exit()
            return False  # 停止 pynput 监听

        else:
            # 其他未处理的按键
            key_name = char if char else str(key)
            print(f"键盘: 未处理按键 '{key_name}'")
            return True

    def _on_key_release(self, key) -> bool:
        try:
            if key == keyboard.Key.enter:
                if self.enter_pressed:
                    print("键盘: Enter键释放 - 启动扭矩保持")
                    # Enter键松开时启动一分钟扭矩保持
                    torque_limit = 400  # 默认值
                    duration = 60.0  # 默认60秒

                    # 尝试从控制器获取当前模式的扭矩参数
                    if hasattr(self.controller, 'current_torque_params'):
                        torque_params = self.controller.current_torque_params
                        torque_limit = torque_params.get('hold_torque', 400)
                        duration = torque_params.get('auto_hold_duration', 60.0)

                    # 启动扭矩保持，使用所有电机
                    self.controller.start_timed_torque_hold(
                        torque_limit=torque_limit,
                        duration=duration
                    )

                    if self.control_callbacks['on_torque_hold_start']:
                        self.control_callbacks['on_torque_hold_start'](torque_limit, duration)

                    print(f"⏱️ 已启动 {duration}秒扭矩保持")

                self.enter_pressed = False

            elif key == keyboard.Key.backspace:
                if self.backspace_pressed:
                    print("键盘: Backspace键释放 - 停止张开")
                self.backspace_pressed = False

        except Exception as e:
            print(f"按键释放处理错误: {e}")
            import traceback
            traceback.print_exc()
        return True

    def _control_loop(self):
        step_count = 0
        last_print_time = time.time()
        print_interval = 1  # 打印间隔(秒)

        print("键盘控制循环已启动 - 直接电机控制模式")

        while self.is_running:
            try:
                current_time = time.time()

                # --- 仅在Enter键按下时执行运动和条件打印 ---
                if self.enter_pressed:
                    # 1. 执行运动步
                    moved = self.controller.realtime_close_step()
                    step_count += 1

                    # 2. 状态打印：只在间隔时间到达时打印
                    if current_time - last_print_time > print_interval:
                        print(f"Enter键按下中，执行第 {step_count} 步闭合")
                        last_print_time = current_time

                    # 3. 目标到达逻辑
                    if not moved and step_count > 1:
                        if not hasattr(self, '_target_reached_printed') or not self._target_reached_printed:
                            print("已到达目标抓取位置")
                            self._target_reached_printed = True
                    else:
                        self._target_reached_printed = False

                # --- 仅在Backspace键按下时执行运动和条件打印 ---
                elif self.backspace_pressed:
                    # 1. 执行运动步
                    moved = self.controller.realtime_open_step()
                    step_count += 1

                    # 2. 状态打印：只在间隔时间到达时打印
                    if current_time - last_print_time > print_interval:
                        print(f"Backspace键按下中，执行第 {step_count} 步张开")
                        last_print_time = current_time

                    # 3. 目标到达逻辑
                    if not moved and step_count > 1:
                        if not hasattr(self, '_initial_reached_printed') or not self._initial_reached_printed:
                            print("已完全张开")
                            self._initial_reached_printed = True
                    else:
                        self._initial_reached_printed = False

                else:
                    # 没有按键按下时的处理
                    if step_count > 0:
                        print(f"按键释放，停止运动。总共执行了 {step_count} 步")
                        step_count = 0

                # 控制循环的休眠时间
                time.sleep(self.control_interval)

            except Exception as e:
                print(f"控制循环错误: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)

    def _handle_exit(self):
        self.is_running = False
        print("\n 正在退出...")
        if self.control_callbacks['on_exit']:
            self.control_callbacks['on_exit']()

        self.stop()

    def _show_motor_status(self):
        """显示电机状态信息 - 添加角度显示"""
        try:
            if hasattr(self.controller, 'get_motor_status_info'):
                status_info = self.controller.get_motor_status_info()
                print("\n" + "=" * 80)
                print("                           电机状态信息")
                print("=" * 80)
                print(
                    f"{'电机ID':<6} {'关节名称':<12} {'扭矩限制':<8} {'位置(rad)':<10} {'位置(°)':<8} {'温度':<6} {'电流':<8} {'电压':<6} {'移动'}")
                print("-" * 80)

                for motor_id in sorted(status_info.keys()):
                    info = status_info[motor_id]
                    position_rad = info.get('position', 0)
                    position_deg = math.degrees(position_rad)  # 弧度转角度

                    print(f"{motor_id:<6} {info.get('joint', '未知'):<12} "
                          f"{info.get('torque_limit', 0):<8} "
                          f"{position_rad:<10.3f} "  # 显示弧度值
                          f"{position_deg:<8.1f} "  # 显示角度值
                          f"{info.get('temperature', 0):<6} "
                          f"{info.get('current', 0):<8.1f} "
                          f"{info.get('voltage', 0):<6.1f} "
                          f"{'是' if info.get('moving', False) else '否'}")

                print("=" * 80)

                # 显示扭矩保持状态
                status = self.controller.get_status()
                if status.get('is_torque_holding', False):
                    print("扭矩保持: 激活中")
                else:
                    print("扭矩保持: 未激活")

            else:
                print("控制器不支持获取电机状态信息")

        except Exception as e:
            print(f"获取电机状态失败: {e}")

    def set_callback(self, event: str, callback: Callable):
        if event in self.control_callbacks:
            self.control_callbacks[event] = callback

    def _print_controls(self):
        print("\n" + "=" * 60)
        print("               ORCA Hand 键盘控制")
        print("=" * 60)
        print("📏 单位说明: 所有角度使用弧度制")
        print("             (1 rad ≈ 57.3°, π rad = 180°)")
        print("=" * 60)
        print("实时控制:")
        print("  Enter键     (按住) - 手指逐渐闭合")
        print("                (松开) - 自动启动60秒扭矩保持")
        print("  Backspace键 (按住) - 手指逐渐张开")
        print("  Space键             - 紧急停止所有运动")
        print("\n模式切换:")
        print("  1, 2, 3             - 切换抓取模式 (不会取消扭矩保持)")
        print("\n扭矩控制:")
        print("  T键                 - 手动启动扭矩保持")
        print("  特定按键             - 取消扭矩保持 (Space, Backspace, T, S, C)")
        print("\n状态监控:")
        print("  S键                 - 显示电机状态信息")
        print("\n系统功能:")
        print("  C键                 - 触发系统校准")
        print("  Q / Esc键           - 退出程序")
        print("=" * 60)
        print("💡 提示:")
        print("  • 松开Enter键后自动保持抓取力60秒")
        print("  • 数字键切换模式不会取消扭矩保持")
        print("  • 不同抓取模式使用不同的扭矩设置")
        print("  • 所有角度单位均为弧度")
        print("=" * 60)
        print()


# 测试代码
if __name__ == "__main__":
    print("键盘控制器测试...")

    # 创建模拟控制器
    controller = GraspController(simulation=True)
    keyboard_ctrl = KeyboardController(controller)


    # 设置回调函数示例
    def on_mode_change(mode_id):
        # 可以在这里获取当前角度并显示
        status = controller.get_status()
        current_angles = status['current_angles']
        # 选择关键关节显示
        key_joints = ['thumb_mcp', 'index_mcp', 'middle_mcp']
        for joint in key_joints:
            if joint in current_angles:
                angle_rad = current_angles[joint]
                angle_deg = math.degrees(angle_rad)
                print(f"  关节 {joint}: {angle_rad:.3f} rad ({angle_deg:.1f}°)")

        print(f"回调: 模式已切换到 {mode_id}")


    def on_emergency_stop():
        print("回调: 紧急停止触发")


    def on_torque_hold_start(torque, duration):
        print(f"回调: 扭矩保持启动 - 扭矩:{torque}, 时长:{duration}秒")


    def on_torque_hold_cancel():
        print("回调: 扭矩保持取消")


    def on_exit():
        print("回调: 程序退出")


    keyboard_ctrl.set_callback('on_mode_change', on_mode_change)
    keyboard_ctrl.set_callback('on_emergency_stop', on_emergency_stop)
    keyboard_ctrl.set_callback('on_torque_hold_start', on_torque_hold_start)
    keyboard_ctrl.set_callback('on_torque_hold_cancel', on_torque_hold_cancel)
    keyboard_ctrl.set_callback('on_exit', on_exit)

    # 启动键盘控制
    success = keyboard_ctrl.start()

    if success:
        print("键盘控制测试启动成功")
        print("请尝试按 1/2/3/4 切换模式，按住Enter键抓取，松开Enter键启动扭矩保持")
        print("按 S 键查看电机状态，按 Q 键退出")

        # 等待用户退出
        try:
            while keyboard_ctrl.is_running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n 用户中断")
        finally:
            keyboard_ctrl.stop()
    else:
        print("键盘控制测试启动失败")