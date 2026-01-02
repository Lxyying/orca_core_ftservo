"""
competition_demo.py
功能: ORCA 灵巧手比赛演示系统主程序入口。
职责:
1. 解析命令行参数。
2. 初始化并连接所有核心组件 (Controller, ModeManager, KeyboardController)。
3. 【关键修改】使用硬编码模式，不再加载用户录制的模式文件。
4. 打印用户操作指南。
5. 启动键盘监听线程并保持程序运行。
6. 优雅地进行清理和退出。
"""

import argparse
import os
import sys
import time
import threading

# --- 路径管理 ---
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

print(f"🔧 项目根目录: {PROJECT_ROOT}")
print(f"🔧 Python 路径: {sys.path}")

try:
    # 从 grasp 包导入核心组件
    from grasp.controller import GraspController, get_grasp_controller
    from grasp.mode import get_mode_manager
    from grasp.keyboard import KeyboardController

    print("✅ 核心组件导入成功")
    CORE_IMPORTED = True

except ImportError as e:
    print(f"❌ 核心组件导入失败: {e}")
    import traceback

    traceback.print_exc()
    CORE_IMPORTED = False


def register_competition_modes(mode_manager):
    print("⚙️ 模式加载: 使用 mode.py 中硬编码的预设模式。")

    # 检查 mode.py 中是否已成功加载模式
    modes_info = mode_manager.get_all_modes_info()
    if modes_info:
        print("✅ 预设模式已成功加载:")
        for mode_id, info in modes_info.items():
            print(f"   模式 {mode_id}: {info['name']} - 扭矩限制: {info['torque_limit']}")
    else:
        print("⚠️ 警告: mode.py 中未发现任何模式 (请检查 _init_default_modes)。")


def print_user_guide(controller, mode_manager):
    """打印控制台的用户操作指南"""
    mode = '模拟模式' if controller.simulation else '硬件模式'

    # 获取当前模式的名称
    current_mode_id = controller.current_mode
    if current_mode_id is None:
        # 如果没有设置模式，默认使用模式1
        current_mode_id = 1
        controller.set_mode(current_mode_id)

    current_mode_info = mode_manager.get_mode(current_mode_id)
    current_mode_name = current_mode_info['name'] if current_mode_info else "未定义"

    print("\n" + "=" * 80)
    print(f"🥇 ORCA 灵巧手比赛演示系统 ({mode})")
    print("=" * 80)
    print("⌨️  实时控制指南:")
    print("--------------------------------------------------------------------------------")
    print("  [1], [2], [3], [4]      : 切换抓取模式（使用 mode.py 中硬编码的姿势）")
    print("  [5], [6], [7],          : 切换抓取模式（使用 mode.py 中硬编码的姿势）")
    print("  [Enter] (长按)           : 手指逐渐闭合 (Grab) - 运动到 '目标姿势'")
    print("  [Backspace] (长按)       : 手指逐渐张开 (Release) - 返回 '起手姿势'")
    print("  [Space]                 : 紧急停止所有运动并断开扭矩 (Emergency Stop)")
    print("  [T]                     : 手动启动扭矩保持")
    print("  [S]                     : 显示电机状态信息")
    print("  [C]                     : 触发系统校准流程")
    print("  [Q] 或 [Esc]             : 退出程序")
    print("--------------------------------------------------------------------------------")
    print("💡 高级功能:")
    print("  • 松开 Enter 键后自动启动扭矩保持 (使用当前模式的扭矩参数)")
    print("  • 按任意其他键可取消扭矩保持")
    print("  • 不同抓取模式使用不同的扭矩设置")
    print("--------------------------------------------------------------------------------")
    print(f"✅ 系统已就绪。当前模式: {current_mode_name} (模式 {current_mode_id})")
    print("--------------------------------------------------------------------------------\n")


def main():
    """程序主入口"""
    if not CORE_IMPORTED:
        return

    parser = argparse.ArgumentParser(
        description="ORCA 灵巧手比赛演示系统启动脚本"
    )
    parser.add_argument(
        "--simulation",
        action="store_true",
        help="以模拟模式运行，不连接实际硬件。"
    )
    parser.add_argument(
        "--model-path",
        type=str,
        default=None,
        help="指定 ORCA Hand 模型文件夹的路径 (现已无用，数据已硬码)。"
    )
    args = parser.parse_args()

    # 1. 初始化核心组件 (Mode Manager, Controller, Keyboard)
    try:
        print("🔧 正在初始化系统组件...")

        # 模式管理器必须首先初始化2
        mode_manager = get_mode_manager(args.model_path)

        # Controller 初始化
        print("🎮 初始化抓取控制器...")
        controller = get_grasp_controller(
            simulation=args.simulation,
            hand_instance=None  # 硬件模式下需要提供实际的 hand 实例
        )

        # Keyboard Controller 初始化，并依赖注入 Controller
        print("⌨️ 初始化键盘控制器...")
        kb_controller = KeyboardController(controller)

    except Exception as e:
        print(f"❌ 启动核心组件失败: {e}")
        import traceback
        traceback.print_exc()
        return

    # 2. 注册模式文件（仅打印信息）
    register_competition_modes(mode_manager)

    # 3. 打印用户指南
    print_user_guide(controller, mode_manager)

    # 4. 启动键盘监听
    try:
        print("▶️ 正在启动键盘监听...")
        success = kb_controller.start()
        if not success:
            print("❌ 键盘监听启动失败")
            return

        print("✅ 键盘监听已启动，系统准备就绪！")
        print("💡 提示: 按 1/2/3/4/5/6/7 切换模式，按住 Enter 抓取，松开 Enter 启动扭矩保持")

        # 5. 主循环 (保持程序运行，直到键盘线程停止)
        while kb_controller.is_running:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n捕获到中断信号 (Ctrl+C)。")

    finally:
        # 6. 清理和退出
        print("\n👋 正在清理系统资源，请稍候...")
        kb_controller.stop()  # 停止键盘监听
        if hasattr(controller, 'emergency_stop'):
            controller.emergency_stop()  # 确保硬件扭矩安全关闭
        print("✅ 系统已安全退出。")


if __name__ == "__main__":
    main()