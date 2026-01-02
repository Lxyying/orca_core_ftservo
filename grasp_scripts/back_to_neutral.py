#!/usr/bin/env python3
"""
back_to_neutral.py 
"""

import sys
import os
import time
import math

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

try:
    from grasp.controller import get_grasp_controller
    from grasp.mode import get_mode_manager

    print("✅ 成功导入核心模块")
except ImportError as e:
    print(f"❌ 导入错误: {e}")
    sys.exit(1)


def radians_to_degrees(radians):
    return radians * 180 / math.pi


def main():
    print("🔧 ORCA Hand 回到中立位 - 直接电机控制版本")
    print("==========================================")
    print("完全绕过关节角度转换，直接控制电机")
    print("==========================================")

    confirm = input("🔧 确认让手部回到中立位? (y/N): ").strip().lower()
    if confirm != 'y':
        print("❌ 操作取消")
        return

    try:
        torque_limit = int(input("💪 请输入安全扭矩限制 (推荐 100-300): ") or "200")
        if not (0 <= torque_limit <= 1000):
            print("❌ 扭矩限制必须在 0-1000 范围内")
            return
    except ValueError:
        print("❌ 请输入有效的数字")
        return

    print(f"🔒 使用安全扭矩限制: {torque_limit}")

    try:
        # 初始化控制器
        print("\n🔌 正在连接手部硬件...")
        controller = get_grasp_controller(simulation=False)
        mode_manager = get_mode_manager()

        # 检查连接状态
        status = controller.get_status()
        if status['simulation']:
            print("❌ 错误: 控制器处于模拟模式")
            return

        print("✅ 手部硬件连接成功")

        # 获取系统信息
        motor_ids = mode_manager.get_motor_ids()
        joint_to_motor = mode_manager.get_joint_to_motor_map()
        motor_to_joint = mode_manager.get_motor_to_joint_map()
        neutral_position = mode_manager.default_neutral

        print(f"🔧 发现 {len(motor_ids)} 个电机")
        print("🔄 准备回到中立位...")

        # 设置安全扭矩限制
        print(f"\n⚙️ 设置所有电机扭矩限制为 {torque_limit}...")
        controller.set_torque_limit_direct(motor_ids, torque_limit)
        time.sleep(1.0)

        # 读取当前真实电机位置
        print("\n📡 读取当前电机位置...")
        current_motor_positions = controller.get_motor_positions_dict()

        if not current_motor_positions:
            print("❌ 无法读取电机位置")
            return

        # 显示当前位置
        print("📊 当前真实电机位置:")
        print("-" * 60)
        for motor_id in sorted(motor_ids):
            joint_name = motor_to_joint.get(motor_id, "未知")
            current_pos = current_motor_positions.get(motor_id, 0)
            print(f"  电机{motor_id:2} ({joint_name:<12}): {current_pos:.3f}rad")

        # 创建目标电机位置字典
        # 关键：直接将中立位关节角度作为电机目标位置
        print("\n🎯 设置目标电机位置...")
        target_motor_positions = {}

        for joint_name, target_angle in neutral_position.items():
            motor_id = joint_to_motor.get(joint_name)
            if motor_id:
                target_motor_positions[motor_id] = target_angle
                current_pos = current_motor_positions.get(motor_id, 0)
                error = abs(target_angle - current_pos)
                error_deg = radians_to_degrees(error)

                status = "✅" if error_deg < 5 else "⚠️"
                print(
                    f"  {status} 电机{motor_id:2} ({joint_name:<12}): {current_pos:.3f} -> {target_angle:.3f}rad (移动: {error_deg:.1f}°)")

        # 确认开始移动
        confirm_move = input(f"\n⚠️  确认开始移动? (y/N): ").strip().lower()
        if confirm_move != 'y':
            print("❌ 移动取消")
            return

        print(f"\n🚀 开始移动到中立位...")

        try:
            # 方法1：使用 hand 的直接电机控制（最可靠）
            print("🔧 使用 hand.set_motor_positions_direct 直接控制...")
            success = controller.hand.set_motor_positions_direct(target_motor_positions)

            if success:
                print("✅ 直接电机控制命令发送成功")
                print("⏳ 等待移动完成...")
                time.sleep(3.0)
            else:
                print("❌ 直接电机控制失败")
                return

        except Exception as e:
            print(f"❌ 控制过程中出错: {e}")
            import traceback
            traceback.print_exc()
            return

        # 验证最终位置
        print(f"\n🔍 验证最终位置...")
        final_motor_positions = controller.get_motor_positions_dict()

        if final_motor_positions:
            print("📊 最终位置状态:")
            print("-" * 60)

            success_count = 0
            total_error = 0

            for motor_id in sorted(motor_ids):
                joint_name = motor_to_joint.get(motor_id, "未知")
                final_pos = final_motor_positions.get(motor_id, 0)
                target_pos = target_motor_positions.get(motor_id, final_pos)
                error = abs(final_pos - target_pos)
                error_deg = radians_to_degrees(error)
                total_error += error

                status = "✅" if error_deg < 10 else "⚠️"
                if error_deg < 10:
                    success_count += 1

                print(f"  {status} 电机{motor_id:2} ({joint_name:<12}): {final_pos:.3f}rad (误差: {error_deg:.1f}°)")

            avg_error = total_error / len(motor_ids)
            avg_error_deg = radians_to_degrees(avg_error)

            print(f"\n📏 最终平均误差: {avg_error:.3f}rad ({avg_error_deg:.1f}°)")
            print(f"🎉 中立位移动完成: {success_count}/{len(motor_ids)} 个电机位置正确")
        else:
            print("⚠️  无法读取最终位置")

    except KeyboardInterrupt:
        print(f"\n🛑 用户中断操作")
    except Exception as e:
        print(f"\n❌ 操作过程中出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 恢复默认扭矩限制
        if 'controller' in locals() and 'motor_ids' in locals():
            print(f"\n🔧 恢复默认扭矩限制...")
            controller.set_torque_limit_direct(motor_ids, 1000)
            print("💡 操作完成")


if __name__ == "__main__":
    main()