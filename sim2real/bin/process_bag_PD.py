import sys
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.spatial.transform import Rotation as R

import rosbag2_py
import rclpy
import rclpy.serialization
from jirl_interfaces.msg import CommandCTBR
from nav_msgs.msg import Odometry

rclpy.init()


def quaternion_to_euler(q):
    r = R.from_quat([q.x, q.y, q.z, q.w])
    return r.as_euler('xyz', degrees=False)


def analyze_ros2_bag(bag_path, namespace, t0, tf):
    if bag_path.endswith('.mcap') or bag_path.endswith('.db3'):
        uri = str(Path(bag_path).parent)
    else:
        uri = str(Path(bag_path))

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    topic_cmd = '/cf/ctbr_command'
    topic_odom = f'/{namespace}/odom'

    ctbr_times, roll_ref, pitch_ref, yawrate_ref = [], [], [], []
    odom_times = []
    roll_meas, pitch_meas, yaw_meas = [], [], []
    rollrate_meas, pitchrate_meas, yawrate_meas = [], [], []
    vx_list, vy_list, vz_list = [], [], []

    first_time = None

    while reader.has_next():
        topic, data, t_nsec = reader.read_next()

        if topic not in [topic_cmd, topic_odom]:
            continue

        time = t_nsec * 1e-9
        if first_time is None:
            first_time = time
        t_rel = time - first_time

        if t_rel < t0 or t_rel > tf:
            continue

        if topic == topic_cmd:
            msg = rclpy.serialization.deserialize_message(data, CommandCTBR)
            print(msg)
            ctbr_times.append(t_rel)
            roll_ref.append(msg.roll_rate)
            pitch_ref.append(-msg.pitch_rate)
            yawrate_ref.append(-msg.yaw_rate)
        elif topic == topic_odom:
            msg = rclpy.serialization.deserialize_message(data, Odometry)

            roll, pitch, yaw = quaternion_to_euler(msg.pose.pose.orientation)
            roll_meas.append(roll * 180.0 / np.pi)
            pitch_meas.append(pitch * 180.0 / np.pi)
            yaw_meas.append(yaw * 180.0 / np.pi)

            quat = [msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w]
            rot_world_to_body = R.from_quat(quat).as_matrix().T
            omega_world = np.array([
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ])
            omega_body = rot_world_to_body @ omega_world

            rollrate_meas.append(omega_body[0] * 180.0 / np.pi)
            pitchrate_meas.append(omega_body[1] * 180.0 / np.pi)
            yawrate_meas.append(omega_body[2] * 180.0 / np.pi)

            vx_list.append(msg.twist.twist.linear.x)
            vy_list.append(msg.twist.twist.linear.y)
            vz_list.append(msg.twist.twist.linear.z)

            odom_times.append(t_rel)

    if not ctbr_times or not odom_times:
        print("No data found in the selected time window.")
        return

    ctbr_times = np.array(ctbr_times)
    odom_times = np.array(odom_times)

    fig, axs = plt.subplots(nrows=4, ncols=2, figsize=(14, 12), sharex='col')

    # --- ROLL ---
    axs[0, 0].plot(ctbr_times, roll_ref, label="Roll rate ref [deg/s]")
    axs[0, 0].plot(odom_times, rollrate_meas, label="Roll rate meas [deg/s]")
    axs[0, 0].set_ylabel("Roll rate")
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    axs[0, 1].plot(odom_times, roll_meas, label="Roll [deg]", color='tab:orange')
    axs[0, 1].set_ylabel("Roll")
    axs[0, 1].legend()
    axs[0, 1].grid(True)

    # --- PITCH ---
    axs[1, 0].plot(ctbr_times, pitch_ref, label="Pitch rate ref [deg/s]")
    axs[1, 0].plot(odom_times, pitchrate_meas, label="Pitch rate meas [deg/s]")
    axs[1, 0].set_ylabel("Pitch rate")
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    axs[1, 1].plot(odom_times, pitch_meas, label="Pitch [deg]", color='tab:orange')
    axs[1, 1].set_ylabel("Pitch")
    axs[1, 1].legend()
    axs[1, 1].grid(True)

    # --- YAW ---
    axs[2, 0].plot(ctbr_times, yawrate_ref, label="Yaw rate ref [deg/s]")
    axs[2, 0].plot(odom_times, yawrate_meas, label="Yaw rate meas [deg/s]")
    axs[2, 0].set_ylabel("Yaw rate")
    axs[2, 0].legend()
    axs[2, 0].grid(True)

    axs[2, 1].plot(odom_times, yaw_meas, label="Yaw [deg]", color='tab:orange')
    axs[2, 1].set_ylabel("Yaw")
    axs[2, 1].legend()
    axs[2, 1].grid(True)

    # --- Linear Velocities ---
    axs[3, 0].plot(odom_times, vx_list, label="v_x [m/s]")
    axs[3, 0].plot(odom_times, vy_list, label="v_y [m/s]")
    axs[3, 0].plot(odom_times, vz_list, label="v_z [m/s]")
    axs[3, 0].set_xlabel("Time [s]")
    axs[3, 0].set_ylabel("Linear vel")
    axs[3, 0].legend()
    axs[3, 0].grid(True)

    axs[3, 1].axis('off')

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("\nUsage: python plot_command_vs_odom.py <bag_path_or_dir> <namespace> [t0] [tf]")
        sys.exit(1)
    else:
        bag_path_arg = sys.argv[1]
        namespace_arg = sys.argv[2]
        t0_arg = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
        tf_arg = float(sys.argv[4]) if len(sys.argv) > 4 else float('inf')

        if tf_arg <= t0_arg:
            print(f"Error: End time (tf={tf_arg}) must be greater than start time (t0={t0_arg}).")
            sys.exit(1)

        analyze_ros2_bag(bag_path_arg, namespace_arg, t0_arg, tf_arg)
