from orca_core.hardware.ftservo_client import DynamixelClient
import time
import argparse
import numpy as np
import serial.tools.list_ports

def main():
    parser = argparse.ArgumentParser(description="Check a motor connected to the Feite Servo client.")
    parser.add_argument("--port", type=str, default="COM9", help="The port to connect to the Feite Servo client.")
    parser.add_argument("--baudrate", type=int, default=1000000, help="The baudrate for the Feite Servo client.")
    parser.add_argument("--motor_id", type=int, default=1, help="The ID of the motor to check.")
    parser.add_argument("--wrist", action="store_true", help="Set if checking a wrist motor.")
    parser.add_argument("--reverse", action="store_true",
                        help="If set, subtracts 0.1 from position, otherwise adds 0.1.")

    args = parser.parse_args()

    print(f"Connecting to motor ID {args.motor_id} on port {args.port} with baudrate {args.baudrate}")

    try:
        # 1. 创建客户端
        dxl_client = DynamixelClient([args.motor_id], args.port, args.baudrate)

        # 2. 连接硬件（必须这一步先执行）
        dxl_client.connect()
        print("Successfully connected!")

        # 3. 设置运行模式
        operating_mode = 0  # STS3215位置伺服模式
        if args.wrist:
            operating_mode = 0
            print(f"Operating in position servo mode (0) for wrist motor ID {args.motor_id}.")
        else:
            print(f"Operating in position servo mode (0) for motor ID {args.motor_id}.")

        dxl_client.set_operating_mode([args.motor_id], operating_mode)

        # 4. 使能扭矩
        dxl_client.set_torque_enabled([args.motor_id], True)
        print("Torque enabled. Starting position test...")

        # 5. 开始测试
        while True:
            pos, vel, cur = dxl_client.read_pos_vel_cur()
            current_pos = pos[0]

            increment = -0.1 if args.reverse else 0.1
            new_pos = current_pos + increment

            dxl_client.write_desired_pos([args.motor_id], np.array([new_pos]))

            print(f"Position: {current_pos:.3f} rad, Target: {new_pos:.3f} rad")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # 确保安全断开
        if 'dxl_client' in locals():
            dxl_client.set_torque_enabled([args.motor_id], False)
            dxl_client.disconnect()
            print("Disconnected.")


if __name__ == "__main__":
    main()