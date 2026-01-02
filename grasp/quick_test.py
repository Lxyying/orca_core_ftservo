#!/usr/bin/env python3
"""
快速测试抓取系统核心功能
"""

import sys
import os
import time

# 添加项目根目录到路径，确保可以导入 orca_core 和其他模块
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)


def test_basic_imports():
    """测试基本导入"""
    print("🧪 测试模块导入...")
    try:
        # 修改1: 使用绝对导入而不是相对导入
        from grasp.controller import GraspController
        from grasp.mode import GraspModeManager
        print("✅ 模块导入成功")
        return True
    except Exception as e:
        print(f"❌ 模块导入失败: {e}")

        # 尝试替代导入方式
        try:
            print("🔄 尝试替代导入方式...")
            # 直接导入当前目录的模块
            import controller
            import mode
            print("✅ 替代导入方式成功")
            print(f"controller 中的类: {[name for name in dir(controller) if 'Controller' in name]}")
            print(f"mode 中的类: {[name for name in dir(mode) if 'Manager' in name]}")
            return True
        except Exception as e2:
            print(f"❌ 替代导入方式也失败: {e2}")
            return False


def test_mode_manager():
    """测试模式管理器"""
    print("\n🧪 测试模式管理器...")
    try:
        # 修改2: 在函数内部导入，避免全局导入问题
        from grasp.mode import GraspModeManager
        mode_manager = GraspModeManager()

        # 检查模式
        for mode_id in [1, 2, 3]:
            mode = mode_manager.get_mode(mode_id)
            if mode:
                print(f"✅ 模式 {mode_id}: {mode['name']}")
            else:
                print(f"❌ 模式 {mode_id} 不存在")

        # 检查电机映射
        motor_ids = mode_manager.get_motor_ids()
        print(f"✅ 电机ID列表: {motor_ids}")

        return True
    except Exception as e:
        print(f"❌ 模式管理器测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_controller_basic():
    """测试控制器基本功能"""
    print("\n🧪 测试控制器基本功能...")
    try:
        # 修改3: 在函数内部导入
        from grasp.controller import GraspController
        controller = GraspController(simulation=True)

        # 测试模式切换
        for mode_id in [1, 2, 3]:
            success = controller.set_mode(mode_id)
            if success:
                print(f"✅ 切换到模式 {mode_id} 成功")
            else:
                print(f"❌ 切换到模式 {mode_id} 失败")

        # 测试状态获取
        status = controller.get_status()
        print(f"✅ 控制器状态获取成功")
        print(f"   模拟模式: {status['simulation']}")
        print(f"   当前模式: {status['current_mode']}")

        return True
    except Exception as e:
        print(f"❌ 控制器测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_torque_functions():
    """测试扭矩功能"""
    print("\n🧪 测试扭矩功能...")
    try:
        # 修改4: 在函数内部导入
        from grasp.controller import GraspController
        controller = GraspController(simulation=True)
        controller.set_mode(1)

        # 测试扭矩设置
        controller.set_torque_limit_direct([1, 2, 3], 400)
        print("✅ 扭矩设置功能正常")

        # 测试扭矩保持
        controller.start_timed_torque_hold(duration=3)  # 只保持3秒测试
        print("✅ 扭矩保持启动成功")

        time.sleep(1)

        # 测试取消
        controller.cancel_torque_hold_on_other_actions()
        print("✅ 扭矩保持取消成功")

        return True
    except Exception as e:
        print(f"❌ 扭矩功能测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """主测试函数"""
    print("=" * 60)
    print("           快速功能测试")
    print("=" * 60)

    tests = [
        test_basic_imports,
        test_mode_manager,
        test_controller_basic,
        test_torque_functions
    ]

    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"❌ 测试异常: {e}")
            results.append(False)

    print("\n" + "=" * 60)
    print("测试结果汇总:")
    passed = sum(results)
    total = len(results)
    print(f"通过: {passed}/{total}")

    if passed == total:
        print("🎉 所有测试通过！系统基本功能正常")
        print("💡 现在可以尝试使用键盘控制进行完整测试")
    else:
        print("❌ 部分测试失败，请检查上述错误信息")

    print("=" * 60)

    return passed == total


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)