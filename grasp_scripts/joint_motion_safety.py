#!/usr/bin/env python3
"""
joint_motion_safety.py
关节运动范围动态测试工具 - 直接电机控制版本
功能：连接真实手部，使用直接电机控制方法测试每个关节在安全范围内的运动
"""

import sys
import os
import time
import math

# 添加正确的导入路径
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
    """将弧度转换为角度"""
    return radians * 180 / math.pi


def main():
    """主函数 - 直接电机控制版本"""
    print("🎯 ORCA Hand 关节运动范围动态测试 - 直接电机控制版本")
    print("==========================================")
    print("⚠️  警告: 这个脚本将实际控制硬件运动!")
    print("      使用直接电机控制方法，绕过关节转换问题")
    print("      请确保手部周围没有障碍物")
    print("==========================================")

    # 获取用户确认
    confirm = input("🔧 确认开始硬件运动测试? (y/N): ").strip().lower()
    if confirm != 'y':
        print("❌ 测试取消")
        return

    # 获取扭矩限制
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
        # 初始化控制器（硬件模式）
        print("\n🔌 正在连接手部硬件...")
        controller = get_grasp_controller(simulation=False)
        mode_manager = get_mode_manager()

        # 检查连接状态
        status = controller.get_status()
        if status['simulation']:
            print("❌ 错误: 控制器处于模拟模式，无法进行硬件测试")
            return

        print("✅ 手部硬件连接成功")

        # 获取关节信息
        joints = mode_manager.joint_order
        safety_limits = mode_manager.safety_limits
        joint_to_motor = mode_manager.joint_to_motor_map
        motor_to_joint = mode_manager.motor_to_joint_map
        motor_ids = mode_manager.get_motor_ids()

        print(f"🔧 发现 {len(joints)} 个关节，开始运动测试...")

        # 设置安全扭矩限制
        print(f"\n⚙️ 设置所有电机扭矩限制为 {torque_limit}...")
        controller.set_torque_limit_direct(motor_ids, torque_limit)
        time.sleep(1.0)

        # 测试每个关节
        for joint_name in joints:
            if joint_name not in safety_limits:
                print(f"⚠️  跳过 {joint_name} - 无安全限制数据")
                continue

            limits = safety_limits[joint_name]
            motor_id = joint_to_motor.get(joint_name, "未知")

            # 安全范围（弧度）
            safe_min_rad = limits['safe_min']
            safe_max_rad = limits['safe_max']

            # 计算测试位置（安全范围的30%和70%，更保守）
            test_range = safe_max_rad - safe_min_rad
            target_1_rad = safe_min_rad + test_range * 0.35  # 第35%位置
            target_2_rad = safe_min_rad + test_range * 0.65  # 第65%位置
            safe_mid_rad = (safe_min_rad + safe_max_rad) / 2  # 中间位置

            print(f"\n🧪 测试 {joint_name} (电机{motor_id}):")
            print(f"  安全范围: [{safe_min_rad:.3f}rad, {safe_max_rad:.3f}rad]")
            print(f"           [{radians_to_degrees(safe_min_rad):.1f}°, {radians_to_degrees(safe_max_rad):.1f}°]")

            try:
                # 获取当前位置 - 使用直接电机控制方法
                current_motor_positions = controller.get_motor_positions_dict()
                current_angle_rad = current_motor_positions.get(motor_id, safe_min_rad)
                print(f"  当前位置: {current_angle_rad:.3f}rad ({radians_to_degrees(current_angle_rad):.1f}°)")

                # 测试位置1 - 使用 hand 的直接电机控制
                print(f"  🚀 移动到位置1: {target_1_rad:.3f}rad...")
                success = controller.hand.set_motor_positions_direct({motor_id: target_1_rad})
                if not success:
                    print(f"  ❌ 位置1移动失败")
                time.sleep(2.0)

                # 读取实际位置
                actual_motor_positions = controller.get_motor_positions_dict()
                actual_angle_rad = actual_motor_positions.get(motor_id, 0)
                error = abs(actual_angle_rad - target_1_rad)
                error_deg = radians_to_degrees(error)
                print(f"  📍 实际位置: {actual_angle_rad:.3f}rad")
                print(f"  📏 误差: {error:.3f}rad ({error_deg:.1f}°)")

                # 测试位置2 - 使用 hand 的直接电机控制
                print(f"  🚀 移动到位置2: {target_2_rad:.3f}rad...")
                success = controller.hand.set_motor_positions_direct({motor_id: target_2_rad})
                if not success:
                    print(f"  ❌ 位置2移动失败")
                time.sleep(2.0)

                # 读取实际位置
                actual_motor_positions = controller.get_motor_positions_dict()
                actual_angle_rad = actual_motor_positions.get(motor_id, 0)
                error = abs(actual_angle_rad - target_2_rad)
                error_deg = radians_to_degrees(error)
                print(f"  📍 实际位置: {actual_angle_rad:.3f}rad")
                print(f"  📏 误差: {error:.3f}rad ({error_deg:.1f}°)")

                # 返回安全位置 - 使用 hand 的直接电机控制
                print(f"  🔄 返回安全位置: {safe_mid_rad:.3f}rad...")
                success = controller.hand.set_motor_positions_direct({motor_id: safe_mid_rad})
                if not success:
                    print(f"  ❌ 返回安全位置失败")
                time.sleep(2.0)

                print(f"  ✅ {joint_name} 测试完成")

            except Exception as e:
                print(f"  ❌ 测试失败: {e}")
                # 紧急停止并尝试恢复
                controller.emergency_stop()
                time.sleep(1.0)
                continue

        print(f"\n🎉 所有关节运动测试完成!")

        # 最后回到中立位
        print(f"\n🏠 回到中立位...")
        neutral_position = mode_manager.default_neutral

        # 创建目标电机位置字典
        target_motor_positions = {}
        for joint_name, target_angle in neutral_position.items():
            motor_id = joint_to_motor.get(joint_name)
            if motor_id:
                target_motor_positions[motor_id] = target_angle

        # 使用 hand 的直接电机控制回到中立位
        try:
            success = controller.hand.set_motor_positions_direct(target_motor_positions)
            if success:
                print("✅ 回到中立位命令发送成功")
                time.sleep(3.0)
            else:
                print("❌ 回到中立位失败")
        except Exception as e:
            print(f"❌ 回到中立位过程中出错: {e}")

        # 验证最终位置
        final_positions = controller.get_motor_positions_dict()
        print("📊 最终位置验证:")
        success_count = 0
        for motor_id in motor_ids:
            joint_name = motor_to_joint.get(motor_id, "未知")
            final_pos = final_positions.get(motor_id, 0)
            target_pos = target_motor_positions.get(motor_id, final_pos)
            error = abs(final_pos - target_pos)
            error_deg = radians_to_degrees(error)

            status = "✅" if error_deg < 10 else "⚠️"
            if error_deg < 10:
                success_count += 1

            print(f"  {status} {joint_name:<12}: {final_pos:.3f}rad (误差: {error_deg:.1f}°)")

        print(f"\n📈 测试总结: {success_count}/{len(motor_ids)} 个关节回到中立位")

    except KeyboardInterrupt:
        print(f"\n🛑 用户中断测试")
    except Exception as e:
        print(f"\n❌ 测试过程中出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 恢复默认扭矩限制
        if 'controller' in locals() and 'motor_ids' in locals():
            print(f"\n🔧 恢复默认扭矩限制...")
            controller.set_torque_limit_direct(motor_ids, 1000)
            print("💡 测试完成，建议检查手部实际运动情况")


if __name__ == "__main__":
    main()