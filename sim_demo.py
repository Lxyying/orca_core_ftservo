import numpy as np
import time
import math
import os
from sim_adapter import MujocoOrcaHand  # 确保 sim_adapter.py 在旁边


# 1. 辅助函数：数值映射
# 作用：把“真实舵机数值”（比如 3.14）转换成“仿真数值”（比如 0.5）
def map_range(val, in_min, in_max, out_min, out_max):
    # 防止除以0
    if in_max == in_min:
        return out_min
    # 计算比例并映射
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def main():
    # --- A. 初始化仿真环境 ---
    # 自动寻找模型路径，防止找不到文件
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mjcf_path = os.path.join(current_dir, "orcahand_description", "scene_combined.xml")

    print(f"正在加载模型: {mjcf_path}")
    hand = MujocoOrcaHand(mjcf_path, hand_prefix="right")

    # 连接并启动窗口
    hand.connect()
    hand.start_viewer()

    # --- B. 定义参数 (直接从您的 main_demo.py 搬运) ---

    # 1. 真实舵机的关节范围 (Real World Limits)
    real_limits = {
        'thumb_mcp': [2.241, 4.295], 'thumb_abd': [2.147, 3.375],
        'thumb_pip': [2.837, 4.371], 'thumb_dip': [1.963, 4.142],
        'index_abd': [1.687, 3.375], 'index_mcp': [2.546, 4.142], 'index_pip': [2.837, 3.757],
        'middle_abd': [1.994, 2.837], 'middle_mcp': [2.300, 3.375], 'middle_pip': [3.298, 4.525],
        'ring_abd': [2.531, 3.605], 'ring_mcp': [3.139, 3.605], 'ring_pip': [2.991, 4.294],
        'pinky_abd': [2.991, 4.448], 'pinky_mcp': [2.991, 4.448], 'pinky_pip': [2.991, 4.448],
        'wrist': [math.radians(-50), math.radians(30)],
    }

    # 2. 仿真模型的关节范围 (Simulation Limits, 从官方 XML 文件提取)
    # 这里的数值通常较小，0 是中间位置
    sim_limits = {
        'thumb_mcp': [-0.87, 0.87], 'thumb_abd': [-1.08, 0.0], 'thumb_pip': [-0.79, 1.23], 'thumb_dip': [-0.85, 1.45],
        'index_abd': [-1.04, 0.24], 'index_mcp': [-0.35, 1.65], 'index_pip': [-0.35, 1.88],
        'middle_abd': [-0.64, 0.64], 'middle_mcp': [-0.35, 1.58], 'middle_pip': [-0.35, 1.86],
        'ring_abd': [-0.47, 0.80], 'ring_mcp': [-0.35, 1.58], 'ring_pip': [-0.35, 1.86],
        'pinky_abd': [-0.12, 1.16], 'pinky_mcp': [-0.35, 1.71], 'pinky_pip': [-0.35, 1.88],
        'wrist': [-0.67, 0.89]
    }

    # 3. 手指结构定义
    fingers = [
        {'name': 'index', 'joints': ['index_mcp', 'index_pip']},
        {'name': 'middle', 'joints': ['middle_mcp', 'middle_pip']},
        {'name': 'ring', 'joints': ['ring_mcp', 'ring_pip']},
        {'name': 'pinky', 'joints': ['pinky_mcp', 'pinky_pip']},
    ]

    # 4. 运动参数
    period = 2.0  # 周期改慢一点(2秒)，看得更清楚
    step_time = 0.01  # 仿真步长
    amplitude = 0.8  # 运动幅度 (0.0~1.0)
    thumb_amplitude = 0.4
    phase_shift_factor = period / 4

    print(">>> 开始仿真循环！")
    print(">>> 请在弹出的窗口中观察 (按 Ctrl+C 停止)")

    # --- C. 主循环 ---
    try:
        start_time = time.time()
        while True:
            # 1. 检查窗口是否关闭
            if not hand.viewer.is_running():
                print("窗口已关闭，退出程序。")
                break

            # 2. 计算当前时间进度
            t = (time.time() - start_time) % period

            # --- 下面这部分逻辑和您的实机代码完全一样 ---
            # 这里的 calculated_pos 字典里存的都是 "实机的大数值"
            calculated_pos = {}

            # (1) 计算四指位置（由pahse_shift定义不同手指的相位差，以此达到波浪的效果）
            for i, finger in enumerate(fingers):
                phase_shift = i * phase_shift_factor
                for joint in finger['joints']:
                    r_min, r_max = real_limits[joint]
                    center = (r_min + r_max) / 2
                    range_val = (r_max - r_min) / 2
                    # 正弦波计算
                    val = center + amplitude * range_val * np.sin(2 * np.pi * (t - phase_shift) / period)
                    calculated_pos[joint] = val

            # (2) 计算拇指位置
            thumb_joints = ['thumb_mcp', 'thumb_pip', 'thumb_dip']
            # 为了简化，这里稍微偷懒用通用的正弦波，只为展示效果
            for joint in thumb_joints:
                r_min, r_max = real_limits[joint]
                center = (r_min + r_max) / 2
                range_val = (r_max - r_min) / 2
                calculated_pos[joint] = center + thumb_amplitude * range_val * np.sin(2 * np.pi * t / period)

            # 添加其他关节的静态值 (防止报错,显式地定义保持不动)
            calculated_pos['thumb_abd'] = (real_limits['thumb_abd'][0] + real_limits['thumb_abd'][1]) / 2
            calculated_pos['wrist'] = real_limits['wrist'][0]
            # 侧摆关节保持不动
            for j in ['index_abd', 'middle_abd', 'ring_abd', 'pinky_abd']:
                calculated_pos[j] = (real_limits[j][0] + real_limits[j][1]) / 2

            # --- D. 关键步骤：把实机数值映射到仿真数值 ---
            sim_commands = {}
            for joint_name, real_val in calculated_pos.items():
                if joint_name in sim_limits and joint_name in real_limits:
                    # 获取这一关节的【实机】范围
                    rmin, rmax = real_limits[joint_name]
                    # 获取这一关节的【仿真】范围
                    smin, smax = sim_limits[joint_name]

                    # 核心映射！
                    sim_val = map_range(real_val, rmin, rmax, smin, smax)
                    sim_commands[joint_name] = sim_val

            # --- E. 发送指令并步进 ---
            # 1. 设置关节目标
            hand.set_joint_pos(sim_commands)

            # 2. 物理引擎推算一步 (让手真正动起来)
            hand.step()

            # 3. 控制循环速度 (大约 60Hz)
            time.sleep(0.016)

    except KeyboardInterrupt:
        print("\n程序已停止。")


if __name__ == "__main__":
    main()