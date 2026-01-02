"""
抓取模式配置定义 (mode.py) - 纯手动定义模式与安全限制版本

功能：管理抓取模式库，所有数据（关节顺序、安全限制、起手/闭合角度）均在代码中硬编码。
"""
import math
import os
from typing import Dict, List, Optional

# 强制设置为不可用，因为不再依赖任何文件工具
UTILS_AVAILABLE = False


class  GraspModeManager:
    """
    智能抓取模式管理器
    职责：
    1. 手动定义硬件真理 (关节顺序、ROM)
    2. 管理模式库 (Modes)，每个模式包含 'initial_angles'(起手) 和 'target_angles'(闭合)
    """

    def __init__(self, model_path_arg=None):
        self.modes = {}
        self.safety_limits = {}
        self.joint_order = []  # 关键：用于将录制文件的List映射回Dict
        self.config_data = {}
        self.default_neutral = {} # 全局默认中立位

        # 1. 初始化硬件配置 (现在是手动硬编码)
        self._load_hardware_config(model_path_arg)

        # 2. 初始化默认预设 (所有模式现在都在这里定义)
        self._init_default_modes()

    def _load_hardware_config(self, model_path_arg):

        print("⚙️ 加载硬件配置: 使用硬编码数据")

        # A. 手动定义关节顺序 (根据 controller.py 中的模拟手部结构)
        # 16关节：拇指4，食中无名小指各3 (总共 4 + 4*3 = 16)
        self.joint_order = [
            'thumb_mcp', 'thumb_abd', 'thumb_pip', 'thumb_dip',
            'index_abd', 'index_mcp', 'index_pip',
            'middle_abd', 'middle_mcp', 'middle_pip',
            'ring_abd', 'ring_mcp', 'ring_pip',
            'pinky_abd', 'pinky_mcp', 'pinky_pip'
        ]

        # B. 添加电机ID与关节的映射关系
        self.joint_to_motor_map = {
            'thumb_mcp': 4,
            'thumb_abd': 3,
            'thumb_pip': 1,
            'thumb_dip': 2,
            'index_abd': 14,
            'index_mcp': 15,
            'index_pip': 16,
            'middle_abd': 13,
            'middle_mcp': 8,
            'middle_pip': 9,
            'ring_abd': 5,
            'ring_mcp': 7,
            'ring_pip': 6,
            'pinky_abd': 12,
            'pinky_mcp': 10,
            'pinky_pip': 11
        }

        # 创建反向映射（电机ID到关节名称）
        self.motor_to_joint_map = {v: k for k, v in self.joint_to_motor_map.items()}

        # 电机ID列表（所有电机的ID）
        self.motor_ids = list(self.joint_to_motor_map.values())

        # 创建电机ID到索引的映射
        self.motor_id_to_idx_dict = {motor_id: i for i, motor_id in enumerate(self.motor_ids)}

        # C. 手动定义默认中立位 (用于 '张开'姿态，参考 controller.py 的 _init_simulation)
        self.default_neutral = {
            'thumb_mcp': 1.994,  # 电机4 - 舵机4: 1300步
            'thumb_abd': 2.684,  # 电机3 - 舵机3: 1750步
            'thumb_pip': 3.220,  # 电机1 - 舵机1: 2100步
            'thumb_dip': 2.531,  # 电机2 - 舵机2: 1650步

            'index_abd': 1.381,  # 电机14 - 舵机14: 900步
            'index_mcp': 1.841,  # 电机15 - 舵机15: 1200步
            'index_pip': 2.608,  # 电机16 - 舵机16: 1700步

            'middle_abd': 2.915,  # 电机13 - 舵机13: 1900步
            'middle_mcp': 2.454,  # 电机8 - 舵机8: 1600步
            'middle_pip': 2.071,  # 电机9 - 舵机9: 1350步

            'ring_abd': 2.915,  # 电机5 - 舵机5: 1900步
            'ring_mcp': 3.220,  # 电机7 - 舵机7: 2100步
            'ring_pip': 3.067,  # 电机6 - 舵机6: 2000步

            'pinky_abd': 3.034,  # 电机12 - 舵机12: 1950步
            'pinky_mcp': 2.837,  # 电机10 - 舵机10: 1850步
            'pinky_pip': 3.375  # 电机11 - 舵机11: 2200步
        }

        # D. 手动定义安全限制 (Hard/Safe Limits)
        # 【手动输入范围】: 根据实际手部设置
        margin_ratio = 0.05 # 5% 安全缓冲区

        # 定义原始物理范围 (ROMs) - 使用实际测得的弧度范围
        # 定义原始物理范围 (ROMs) - 使用最新测得的弧度范围
        raw_roms = {
            # 大拇指关节
            'thumb_mcp': [1.764, 2.837],  # 电机4 (thumb_mcp) - 舵机4: 1150-1850步 (大拇指4)
            'thumb_abd': [2.224, 3.145],  # 电机3 (thumb_abd) - 舵机3: 1550-2050步 (大拇指3)
            'thumb_pip': [3.067, 4.447],  # 电机1 (thumb_pip) - 舵机1: 2000-2900步 (大拇指2)
            'thumb_dip': [2.224, 3.835],  # 电机2 (thumb_dip) - 舵机2: 1450-2500步 (大拇指1)

            # 食指关节
            'index_abd': [1.074, 1.994],  # 电机14 (index_abd) - 舵机14: 700-1300步 (食指3)
            'index_mcp': [1.381, 2.608],  # 电机15 (index_mcp) - 舵机15: 900-1700步 (食指2)
            'index_pip': [2.378, 3.835],  # 电机16 (index_pip) - 舵机16: 1550-2500步 (食指1)

            # 中指关节
            'middle_abd': [2.531, 3.298],  # 电机13 (middle_abd) - 舵机13: 1650-2150步 (中指3)
            'middle_mcp': [2.300, 3.605],  # 电机8 (middle_mcp) - 舵机8: 1500-2300步 (中指2)
            'middle_pip': [1.994, 3.451],  # 电机9 (middle_pip) - 舵机9: 1300-2250步 (中指1)

            # 无名指关节
            'ring_abd': [2.761, 3.526],  # 电机5 (ring_abd) - 舵机5: 1800-2300步 (4指3)
            'ring_mcp': [2.991, 4.371],  # 电机7 (ring_mcp) - 舵机7: 1950-2850步 (无名指2)
            'ring_pip': [2.837, 4.294],  # 电机6 (ring_pip) - 舵机6: 1850-2800步 (4指1)

            # 小指关节
            'pinky_abd': [2.608, 3.298],  # 电机12 (pinky_abd) - 舵机12: 1700-2150步 (小指3)
            'pinky_mcp': [2.684, 4.142],  # 电机10 (pinky_mcp) - 舵机10: 1750-2700步 (小指2)
            'pinky_pip': [3.067, 4.525]  # 电机11 (pinky_pip) - 舵机11: 2000-2900步 (小指1)
        }

        for joint, limits in raw_roms.items():
            if len(limits) == 2:
                min_val, max_val = limits
                range_span = max_val - min_val
                margin = range_span * margin_ratio

                self.safety_limits[joint] = {
                    'hard_min': min_val,
                    'hard_max': max_val,
                    'safe_min': min_val + margin,
                    'safe_max': max_val - margin
                }

        print(f"✅ 已手动配置 {len(self.joint_order)} 个关节的映射与安全限制")
        print(f"🔌 电机映射: {self.joint_to_motor_map}")

    def get_mode(self, mode_id):
        """获取模式数据"""
        return self.modes.get(mode_id)

    def validate_angles(self, angles):
        """验证角度安全性"""
        if not self.safety_limits:
            return True, "无安全限制数据 (Pass)"

        warnings = []
        for joint, angle in angles.items():
            if joint in self.safety_limits:
                limits = self.safety_limits[joint]
                # 检查物理极限
                if angle < limits['hard_min'] or angle > limits['hard_max']:
                    return False, f"❌ 关节 {joint} ({angle:.1f}) 超出物理极限 [{limits['hard_min']}, {limits['hard_max']}]"
                # 检查软限位
                elif angle < limits['safe_min'] or angle > limits['safe_max']:
                    warnings.append(f"{joint}")

        msg = "验证通过"
        if warnings:
            msg += f" (警告: {', '.join(warnings)} 接近极限)"
        return True, msg

    # ==================== 电机映射相关方法 ====================

    def get_motor_ids(self):
        """获取所有电机ID列表"""
        return self.motor_ids

    def get_joint_to_motor_map(self):
        """获取关节到电机的映射"""
        return self.joint_to_motor_map

    def get_motor_to_joint_map(self):
        """获取电机到关节的映射"""
        return self.motor_to_joint_map

    def get_motor_id_to_idx_dict(self):
        """获取电机ID到索引的映射"""
        return self.motor_id_to_idx_dict

    def get_motor_id_for_joint(self, joint_name):
        """获取指定关节对应的电机ID"""
        return self.joint_to_motor_map.get(joint_name)

    def get_joint_for_motor_id(self, motor_id):
        """获取指定电机ID对应的关节名称"""
        return self.motor_to_joint_map.get(motor_id)

    def _init_default_modes(self):
        """
        初始化所有模式 (现在是模式的唯一来源)
        """
        # 模式1：圆形抓握
        self.modes[1] = {
            'name': '圆形抓握',
            'description': '哑铃、水瓶',
            'initial_angles': self.default_neutral.copy(),
            'target_angles': {
                'thumb_mcp': 1.841,  # 电机4 - 舵机4: 1200步
                'thumb_abd': 2.684,  # 电机3 - 舵机3: 1750步
                'thumb_pip': 3.220,  # 电机1 - 舵机1: 2100步
                'thumb_dip': 2.531,  # 电机2 - 舵机2: 1650步

                'index_abd': 1.381,  # 电机14 - 舵机14: 1000步
                'index_mcp': 2.608,  # 电机15 - 舵机15: 1700步
                'index_pip': 3.375,  # 电机16 - 舵机16: 2200步

                'middle_abd': 2.915,  # 电机13 - 舵机13: 1900步
                'middle_mcp': 3.526,  # 电机8 - 舵机8: 2300步
                'middle_pip': 2.915,  # 电机9 - 舵机9: 1900步

                'ring_abd': 2.915,  # 电机5 - 舵机4: 步
                'ring_mcp': 4.371,  # 电机7 - 舵机7: 2850步
                'ring_pip': 4.294,  # 电机6 - 舵机6: 2800步

                'pinky_abd': 3.034,  # 电机12 - 舵机12: 1900步
                'pinky_mcp': 3.835,  # 电机10 - 舵机10: 2500步
                'pinky_pip': 4.525,  # 电机11 - 舵机11: 2950步
            },


            'motion_params': {
                'step_size': math.radians(2.0),
                'approach_speed': 0.7,
                'max_steps': 50
            },
            # 扭矩参数
            'torque_params': {
                'hold_torque': 400,       # 扭矩保持时的扭矩限制
                'auto_hold_duration': 60  # 自动保持时间(秒)
            }
        }

        # 模式2：捏取
        self.modes[2] = {
            'name': '捏取',
            'description': '拇指食指对捏，适合针、布等物体',
            'initial_angles': self.default_neutral.copy(),
            'target_angles': {
                'thumb_mcp': 2.071,  # 电机4 - 舵机4: 1350步
                'thumb_abd': 2.531,  # 电机3 - 舵机3: 1650步
                'thumb_pip': 3.221,  # 电机1 - 舵机1: 2100步
                'thumb_dip': 2.531,  # 电机2 - 舵机2: 1650步

                'index_abd': 1.534,  # 电机14 - 舵机14: 1000步
                'index_mcp': 2.454,  # 电机15 - 舵机15: 1600步
                'index_pip': 2.684,  # 电机16 - 舵机16: 1750步

                'middle_abd': 2.915,  # 电机13 - 舵机13: 1900步
                'middle_mcp': 2.454,  # 电机8 - 舵机8: 1600步
                'middle_pip': 2.071, # 电机9 - 舵机9: 1350步

                'ring_abd': 2.915,  # 电机5 - 舵机5: 1900步
                'ring_mcp': 3.220,  # 电机7 - 舵机7: 2100步
                'ring_pip': 3.067,  # 电机6 - 舵机6: 2000步

                'pinky_abd': 2.991,  # 电机12 - 舵机12: 1950步
                'pinky_mcp': 2.838,  # 电机10 - 舵机10: 1850步
                'pinky_pip': 3.375, # 电机11 - 舵机11: 2200步
            },
            'motion_params': {
                'step_size': math.radians(1.0),
                'approach_speed': 0.4,
                'max_steps': 50
            },
            # 扭矩参数
            'torque_params': {
                'hold_torque': 400,       # 精细操作需要较小扭矩
                'auto_hold_duration': 60
            }
        }

        # 模式3：包络抓取
        self.modes[3] = {
            'name': '包络抓取',
            'description': '乒乓球、魔方、绳、布',
            'initial_angles': self.default_neutral.copy(),
            'target_angles': {
                'thumb_mcp': 2.148,  # 电机4 - 舵机4: 1250步
                'thumb_abd': 2.224,  # 电机3 - 舵机3: 1450步
                'thumb_pip': 3.528,  # 电机1 - 舵机1: 2300步
                'thumb_dip': 3.2218,  # 电机2 - 舵机2: 2100步

                'index_abd': 1.534,  # 电机14 - 舵机14: 1000步
                'index_mcp': 2.608,  # 电机15 - 舵机15: 1700步
                'index_pip': 2.877,  # 电机16 - 舵机16: 1850步

                'middle_abd': 2.915,  # 电机13 - 舵机13: 1900步
                'middle_mcp': 3.526,  # 电机8 - 舵机8: 2300步
                'middle_pip': 2.301, # 电机9 - 舵机9: 1500步

                'ring_abd': 3.067,  # 电机5 - 舵机5: 2200步
                'ring_mcp': 4.295,  # 电机7 - 舵机7: 2750步
                'ring_pip': 3.682,  # 电机6 - 舵机6: 2350步

                'pinky_abd': 3.031,  # 电机12 - 舵机12: 1950步
                'pinky_mcp': 2.838,  # 电机10 - 舵机10: 1850步
                'pinky_pip': 3.416, # 电机11 - 舵机11: 2200步
            },

            'motion_params': {
                'step_size': math.radians(1.5),
                'approach_speed': 0.6,
                'max_steps': 50
            },
            # 扭矩参数
            'torque_params': {
                'hold_torque': 500,       # 包络抓取需要较大扭矩
                'auto_hold_duration': 60
            }
        }

        # 模式4：抓卡预备位置
        self.modes[4] = {
            'name': '抓卡预备位置',
            'description': '食指大拇指抵住卡',
            'initial_angles': self.default_neutral.copy(),
            'target_angles': {
                'thumb_mcp': 1.994,  # 电机4 - 舵机4: 1300步
                'thumb_abd': 2.454,  # 电机3 - 舵机3: 1600步
                'thumb_pip': 3.375,  # 电机1 - 舵机1: 2200步
                'thumb_dip': 2.454,  # 电机2 - 舵机2: 1600步

                'index_abd': 1.687,  # 电机14 - 舵机14: 1100步
                'index_mcp': 1.687,  # 电机15 - 舵机15: 1100步
                'index_pip': 2.761,  # 电机16 - 舵机16: 1800步

                'middle_abd': 2.915,  # 电机13 - 舵机13: 1900步
                'middle_mcp': 2.608,  # 电机8 - 舵机8: 1700步
                'middle_pip': 2.071,  # 电机9 - 舵机9: 1350步

                'ring_abd': 2.915,  # 电机5 - 舵机5: 1900步
                'ring_mcp': 3.220,  # 电机7 - 舵机7: 2100步
                'ring_pip': 3.067,  # 电机6 - 舵机6: 2000步

                'pinky_abd': 3.034,  # 电机12 - 舵机12: 1950步
                'pinky_mcp': 2.837,  # 电机10 - 舵机10: 1850步
                'pinky_pip': 3.375  # 电机11 - 舵机11: 2200步
            },


            'motion_params': {
                'step_size': math.radians(1.5),
                'approach_speed': 0.6,
                'max_steps': 50
            },
            # 扭矩参数
            'torque_params': {
                'hold_torque': 500,  # 包络抓取需要较大扭矩
                'auto_hold_duration': 60
            }
        }

        # 模式5：抓卡动作
        self.modes[5] = {
            'name': '抓卡动作',
            'description': '无名指靠过来',
            'initial_angles': self.default_neutral.copy(),
            'target_angles': {
                'thumb_mcp': 1.994,  # 电机4 - 舵机4: 1300步
                'thumb_abd': 2.454,  # 电机3 - 舵机3: 1600步
                'thumb_pip': 3.375,  # 电机1 - 舵机1: 2200步
                'thumb_dip': 2.454,  # 电机2 - 舵机2: 1600步

                'index_abd': 1.687,  # 电机14 - 舵机14: 1100步
                'index_mcp': 1.687,  # 电机15 - 舵机15: 1100步
                'index_pip': 2.761,  # 电机16 - 舵机16: 1800步

                'middle_abd': 2.915,  # 电机13 - 舵机13: 1900步
                'middle_mcp': 3.375,  # 电机8 - 舵机8: 2200步
                'middle_pip': 2.071,  # 电机9 - 舵机9: 1350步

                'ring_abd': 2.915,  # 电机5 - 舵机5: 1900步
                'ring_mcp': 3.220,  # 电机7 - 舵机7: 2100步
                'ring_pip': 3.067,  # 电机6 - 舵机6: 2000步

                'pinky_abd': 3.034,  # 电机12 - 舵机12: 1950步
                'pinky_mcp': 2.837,  # 电机10 - 舵机10: 1850步
                'pinky_pip': 3.375  # 电机11 - 舵机11: 2200步
            },


            'motion_params': {
                'step_size': math.radians(1.5),
                'approach_speed': 0.6,
                'max_steps': 50
            },
            # 扭矩参数
            'torque_params': {
                'hold_torque': 500,  # 包络抓取需要较大扭矩
                'auto_hold_duration': 60
            }
        }

        # 模式6：拧瓶盖预备位置
        self.modes[6] = {
            'name': '拧瓶盖预备位置',
            'description': '拧瓶盖预备位置',
            'initial_angles': self.default_neutral.copy(),
            'target_angles': {
                'thumb_mcp': 2.117,  # 电机4 - 舵机4: 1380步
                'thumb_abd': 2.554,  # 电机3 - 舵机3: 1665步
                'thumb_pip': 3.334,  # 电机1 - 舵机1: 2180步
                'thumb_dip': 2.531,  # 电机2 - 舵机2: 1650步

                'index_abd': 1.534,  # 电机14 - 舵机14: 1000步
                'index_mcp': 1.381,  # 电机15 - 舵机15: 900步
                'index_pip': 2.608,  # 电机16 - 舵机16: 1700步

                'middle_abd': 2.915,  # 电机13 - 舵机13: 1900步
                'middle_mcp': 2.454,  # 电机8 - 舵机8: 1600步
                'middle_pip': 2.071,  # 电机9 - 舵机9: 1350步

                'ring_abd': 2.915,  # 电机5 - 舵机5: 1900步
                'ring_mcp': 3.220,  # 电机7 - 舵机7: 2100步
                'ring_pip': 3.067,  # 电机6 - 舵机6: 2000步

                'pinky_abd': 3.034,  # 电机12 - 舵机12: 1950步
                'pinky_mcp': 2.837,  # 电机10 - 舵机10: 1850步
                'pinky_pip': 3.375  # 电机11 - 舵机11: 2200步
            },



            'motion_params': {
                'step_size': math.radians(1.5),
                'approach_speed': 0.6,
                'max_steps': 50
            },
            # 扭矩参数
            'torque_params': {
                'hold_torque': 500,  # 包络抓取需要较大扭矩
                'auto_hold_duration': 60
            }
        }

        # 模式7：拧瓶盖动作
        self.modes[7] = {
            'name': '拧瓶盖动作',
            'description': '食指中指夹紧',
            'initial_angles': self.default_neutral.copy(),
            'target_angles': {
                'thumb_mcp': 2.117,  # 电机4 - 舵机4: 1380步
                'thumb_abd': 2.554,  # 电机3 - 舵机3: 1665步
                'thumb_pip': 3.334,  # 电机1 - 舵机1: 2180步
                'thumb_dip': 2.531,  # 电机2 - 舵机2: 1650步

                'index_abd': 1.534,  # 电机14 - 舵机14: 1000步
                'index_mcp': 1.381,  # 电机15 - 舵机15: 900步
                'index_pip': 2.608,  # 电机16 - 舵机16: 1700步

                'middle_abd': 2.915,  # 电机13 - 舵机13: 1900步
                'middle_mcp': 2.454,  # 电机8 - 舵机8: 1600步
                'middle_pip': 2.071, # 电机9 - 舵机9: 1350步

                'ring_abd': 2.915,  # 电机5 - 舵机5: 1900步
                'ring_mcp': 3.220,  # 电机7 - 舵机7: 2100步
                'ring_pip': 3.067,  # 电机6 - 舵机6: 2000步

                'pinky_abd': 3.034,  # 电机12 - 舵机12: 1950步
                'pinky_mcp': 2.837,  # 电机10 - 舵机10: 1850步
                'pinky_pip': 4.449,  # 电机11 - 舵机11: 2900步
            },


            'motion_params': {
                'step_size': math.radians(1.5),
                'approach_speed': 0.6,
                'max_steps': 50
            },
            # 扭矩参数
            'torque_params': {
                'hold_torque': 500,  # 包络抓取需要较大扭矩
                'auto_hold_duration': 60
            }
        }

        print(f"✅ 已初始化 {len(self.modes)} 个抓取模式")
        for mode_id, mode_info in self.modes.items():
            torque_params = mode_info.get('torque_params', {})
            print(f"  模式 {mode_id}: {mode_info['name']} - 扭矩限制: {torque_params.get('hold_torque', 400)}")

    def get_all_modes_info(self):
        """获取所有模式的信息"""
        modes_info = {}
        for mode_id, mode_data in self.modes.items():
            modes_info[mode_id] = {
                'name': mode_data['name'],
                'description': mode_data['description'],
                'torque_limit': mode_data.get('torque_params', {}).get('hold_torque', 400),
                'duration': mode_data.get('torque_params', {}).get('auto_hold_duration', 60)
            }
        return modes_info

    def get_mode_torque_params(self, mode_id):
        """获取指定模式的扭矩参数"""
        mode = self.get_mode(mode_id)
        if mode:
            return mode.get('torque_params', {})
        return {}

    def update_mode_torque_params(self, mode_id, torque_params):
        """更新指定模式的扭矩参数"""
        if mode_id in self.modes:
            self.modes[mode_id]['torque_params'] = torque_params
            print(f"✅ 已更新模式 {mode_id} 的扭矩参数: {torque_params}")
            return True
        else:
            print(f"❌ 模式 {mode_id} 不存在")
            return False


# 全局单例
_manager = None

def get_mode_manager(model_path_arg=None):
    global _manager
    if _manager is None:
        _manager = GraspModeManager(model_path_arg)
    return _manager


# 测试代码
if __name__ == "__main__":
    print("🧪 模式管理器测试...")

    # 创建模式管理器
    mode_manager = GraspModeManager()

    # 测试模式获取
    print("\n1. 测试模式获取...")
    for mode_id in [1, 2, 3]:
        mode = mode_manager.get_mode(mode_id)
        if mode:
            print(f"✅ 模式 {mode_id}: {mode['name']}")
            torque_params = mode.get('torque_params', {})
            print(f"   扭矩设置: {torque_params}")
        else:
            print(f"❌ 模式 {mode_id} 不存在")

    # 测试电机映射
    print("\n2. 测试电机映射...")
    print(f"电机ID列表: {mode_manager.get_motor_ids()}")
    print(f"关节到电机映射: {mode_manager.get_joint_to_motor_map()}")

    # 测试关节映射
    print("\n3. 测试关节映射...")
    test_joints = ['thumb_mcp', 'index_mcp', 'middle_mcp']
    for joint in test_joints:
        motor_id = mode_manager.get_motor_id_for_joint(joint)
        print(f"关节 {joint} -> 电机 {motor_id}")

    # 测试扭矩参数获取
    print("\n4. 测试扭矩参数获取...")
    torque_params = mode_manager.get_mode_torque_params(1)
    print(f"模式1扭矩参数: {torque_params}")

    # 测试所有模式信息
    print("\n5. 测试所有模式信息...")
    all_modes = mode_manager.get_all_modes_info()
    for mode_id, info in all_modes.items():
        print(f"模式 {mode_id}: {info['name']} - 扭矩: {info['torque_limit']}")

    print("\n✅ 模式管理器测试完成")