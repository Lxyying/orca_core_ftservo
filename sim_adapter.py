import time
import numpy as np
import mujoco
import mujoco.viewer


class MujocoOrcaHand:
    def __init__(self, mjcf_path, hand_prefix="right"):
        # 1. 加载 MuJoCo 模型
        try:
            print(f"正在加载模型: {mjcf_path}")
            self.model = mujoco.MjModel.from_xml_path(mjcf_path)
            self.data = mujoco.MjData(self.model)
        except ValueError as e:
            print(f"错误: 无法加载模型文件 {mjcf_path}。请检查路径是否正确，或者文件夹是否放对了位置。")
            raise e

        self.hand_prefix = hand_prefix

        # 2. 建立映射：您的代码关节名 -> MuJoCo 执行器 ID
        self.joint_to_actuator_id = {}

        # 定义您代码中使用的关节列表
        self.joint_ids = [
            'thumb_mcp', 'thumb_abd', 'thumb_pip', 'thumb_dip',
            'index_abd', 'index_mcp', 'index_pip',
            'middle_abd', 'middle_mcp', 'middle_pip',
            'ring_abd', 'ring_mcp', 'ring_pip',
            'pinky_abd', 'pinky_mcp', 'pinky_pip',
            'wrist'
        ]

        print("正在初始化仿真关节映射...")
        for name in self.joint_ids:
            # 官方模型的命名规则是：前缀 + 关节名 + _actuator
            mj_actuator_name = f"{self.hand_prefix}_{name}_actuator"

            # 在模型中查找这个名字对应的 ID
            id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, mj_actuator_name)
            if id != -1:
                self.joint_to_actuator_id[name] = id
            # else:
            # print(f"  [提示] 仿真模型中找不到执行器: {mj_actuator_name} (如果是无关关节可忽略)")

        self.viewer = None

    def connect(self):
        """模拟真实连接"""
        print(">>> 仿真环境已连接")
        return (True, "Simulation Ready")

    def start_viewer(self):
        """启动可视化窗口"""
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        print(">>> 可视化窗口已启动")

    def set_joint_pos(self, joint_pos_dict):
        """核心控制：把目标位置写入仿真模型"""
        for joint_name, target_rad in joint_pos_dict.items():
            if joint_name in self.joint_to_actuator_id:
                actuator_id = self.joint_to_actuator_id[joint_name]
                # data.ctrl 对应真实世界中的电机指令
                self.data.ctrl[actuator_id] = target_rad

    def step(self):
        """物理步进：这是仿真特有的，必须手动调用"""
        # 计算物理模拟 (前进一小步时间)
        mujoco.mj_step(self.model, self.data)

        # 刷新画面
        if self.viewer is not None:
            self.viewer.sync()