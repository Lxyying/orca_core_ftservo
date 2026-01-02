from orca_core import OrcaHand
import numpy as np
import argparse
import math


def main():
    parser = argparse.ArgumentParser(
        description="Run a demo of the ORCA Hand. Specify the path to the orcahand model folder."
    )
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1)"
    )
    args = parser.parse_args()

    # Initialize the hand
    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    # Ensure the hand is connected
    if not status[0]:
        print("Failed to connect to the hand.")
        exit()

    # 使用实际的关节弧度范围
    joint_roms = {
        'thumb_mcp': [2.241, 4.295],
        'thumb_abd': [2.147, 3.375],
        'thumb_pip': [2.837, 4.371],
        'thumb_dip': [1.963, 4.142],
        'index_abd': [1.687, 3.375],
        'index_mcp': [2.546, 4.142],
        'index_pip': [2.837, 3.757],
        'middle_abd': [1.994, 2.837],
        'middle_mcp': [2.300, 3.375],
        'middle_pip': [3.298, 4.525],
        'ring_abd': [2.531, 3.605],
        'ring_mcp': [3.139, 3.605],
        'ring_pip': [2.991, 4.294],
        'pinky_abd': [2.991, 4.448],
        'pinky_mcp': [2.991, 4.448],
        'pinky_pip': [2.991, 4.448],
        # 手腕关节保持原来的范围（转换为弧度）
        'wrist': [math.radians(-50), math.radians(30)],
    }

    # Define the fingers and their joints
    fingers = [
        {'name': 'index', 'joints': ['index_mcp', 'index_pip']},
        {'name': 'middle', 'joints': ['middle_mcp', 'middle_pip']},
        {'name': 'ring', 'joints': ['ring_mcp', 'ring_pip']},
        {'name': 'pinky', 'joints': ['pinky_mcp', 'pinky_pip']},
    ]

    # Movement parameters
    period = 0.4  # Total time for one cycle (seconds)
    step_time = 0.005  # Time between updates (seconds)
    amplitude = 0.7  # Fraction of the ROM to use for finger movement
    thumb_amplitude = 0.4  # Fraction of the ROM to use for thumb movement
    phase_shift_factor = period / 20  # Phase shift between fingers (0 for no shift, period/4 for equal spacing)

    # Precompute the joint positions for each time step
    time_steps = np.arange(0, period, step_time)
    joint_positions = {finger['name']: [] for finger in fingers}
    thumb_positions = []

    for t in time_steps:
        # Compute positions for fingers
        for i, finger in enumerate(fingers):
            phase_shift = i * phase_shift_factor  # Apply the phase shift factor
            positions = {}
            for joint in finger['joints']:
                rom_min, rom_max = joint_roms[joint]
                center = (rom_min + rom_max) / 2
                range_ = (rom_max - rom_min) / 2
                positions[joint] = center + amplitude * range_ * np.sin(2 * np.pi * (t - phase_shift) / period)
            joint_positions[finger['name']].append(positions)

        # Compute positions for the thumb
        thumb_pos = {
            'thumb_mcp': (joint_roms['thumb_mcp'][0] + joint_roms['thumb_mcp'][1]) / 2
                         + thumb_amplitude * (joint_roms['thumb_mcp'][1] - joint_roms['thumb_mcp'][0]) / 2
                         * np.sin(2 * np.pi * t / period),
            'thumb_pip': (joint_roms['thumb_pip'][0] + joint_roms['thumb_pip'][1]) / 2
                         + thumb_amplitude / 2 * (joint_roms['thumb_pip'][1] - joint_roms['thumb_pip'][0]) / 2
                         * np.sin(2 * np.pi * t / period),
            'thumb_dip': (joint_roms['thumb_dip'][0] + joint_roms['thumb_dip'][1]) / 2
                         + thumb_amplitude * (joint_roms['thumb_dip'][1] - joint_roms['thumb_dip'][0]) / 2
                         * np.sin(2 * np.pi * t / period),
            'thumb_abd': (joint_roms['thumb_abd'][0] + joint_roms['thumb_abd'][1]) / 2,  # 使用范围中心
            'wrist': joint_roms['wrist'][0],  # 使用最小位置
            'pinky_abd': (joint_roms['pinky_abd'][0] + joint_roms['pinky_abd'][1]) / 2,  # 使用范围中心
            'ring_abd': (joint_roms['ring_abd'][0] + joint_roms['ring_abd'][1]) / 2,  # 使用范围中心
            'index_abd': (joint_roms['index_abd'][0] + joint_roms['index_abd'][1]) / 2,  # 使用范围中心
        }
        thumb_positions.append(thumb_pos)

    # Perform the movement in a loop
    try:
        while True:
            for t_idx, t in enumerate(time_steps):
                # Combine all joint positions for this time step
                current_positions = {}
                for finger in fingers:
                    current_positions.update(joint_positions[finger['name']][t_idx])
                current_positions.update(thumb_positions[t_idx])  # Add thumb, wrist, and abduction positions

                # Send the positions to the hand
                hand.set_joint_pos(current_positions)

    except KeyboardInterrupt:
        # Reset the hand to the neutral position on exit
        hand.set_joint_pos({joint: 0 for joint in hand.joint_ids})
        print("Demo stopped and hand reset.")


if __name__ == "__main__":
    main()