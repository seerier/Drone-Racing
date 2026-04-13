import sys
import csv
from pathlib import Path
import rclpy
import rosbag2_py
import rclpy.serialization
from jirl_interfaces.msg import CommandCTBR
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import numpy as np

# Conversion constants for PWM to grams
c1 = -0.6709
c2 = 0.1932
c3 = 13.0652
thrust_pwm_min = 10001
thrust_pwm_max = 60000

def pwm_to_grams(thrust_pwm, c1, c2, c3, thrust_pwm_min, thrust_pwm_max):
    thrust_pwm = max(thrust_pwm_min, min(thrust_pwm, thrust_pwm_max))
    x = (thrust_pwm - thrust_pwm_min) / (thrust_pwm_max - thrust_pwm_min)
    sqrt_term = (x - c1) / c2
    thrust_grams = sqrt_term**2 - c3
    return max(thrust_grams, 0.0)

def pwm_to_newtons(thrust_pwm):
    g = pwm_to_grams(thrust_pwm, c1, c2, c3, thrust_pwm_min, thrust_pwm_max)
    return g * 9.81 / 1000

# Automatically find the first topic with Odometry messages
def find_odom_topic(bag_path: Path) -> str:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    while reader.has_next():
        topic, data, _ = reader.read_next()
        try:
            msg = rclpy.serialization.deserialize_message(data, Odometry)
            return topic  # found the odometry topic
        except Exception:
            continue

    print("❌ No Odometry message found in the bag.")
    sys.exit(1)

rclpy.init()

def extract_actions_to_csv(bag_dir: str, output_dir: str):
    bag_dir_path = Path(bag_dir)
    output_dir_path = Path(output_dir)
    output_dir_path.mkdir(parents=True, exist_ok=True)

    bag_files = list(bag_dir_path.glob("*.mcap")) + list(bag_dir_path.glob("*.db3"))
    if not bag_files:
        print(f"❌ No .mcap or .db3 files found in: {bag_dir}")
        sys.exit(1)

    bag_file = bag_files[0]
    uri = str(bag_dir_path)
    bag_name = bag_file.stem
    output_csv = output_dir_path / f"{bag_name}.csv"

    # Identify the odometry topic
    odom_topic = find_odom_topic(bag_dir_path)
    print(f"✅ Found odometry topic: {odom_topic}")

    # First pass: collect all Odometry messages
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    odom_data = {}
    while reader.has_next():
        topic, data, t_nsec = reader.read_next()
        if topic != odom_topic:
            continue
        msg = rclpy.serialization.deserialize_message(data, Odometry)
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        vel = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        quat = [ori.x, ori.y, ori.z, ori.w]
        roll, pitch, yaw = euler_from_quaternion(quat)

        r2d = 180.0 / np.pi

        odom_data[t_nsec] = {
            "x": pos.x, "y": pos.y, "z": pos.z,
            "roll": roll * r2d, "pitch": pitch * r2d, "yaw": yaw * r2d,
            "vx": vel.x, "vy": vel.y, "vz": vel.z,
            "wx": ang.x * r2d, "wy": ang.y * r2d, "wz": ang.z * r2d
        }

    # Second pass: extract commands and sync with closest Odometry
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_cmd = "/cf/ctbr_command"
    first_time = None

    with open(output_csv, mode="w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "time",
            "thrust_pwm",
            "thrust_grams",
            "thrust_newton",
            "roll_rate",
            "pitch_rate",
            "yaw_rate",
            "x", "y", "z",
            "roll", "pitch", "yaw",
            "vx", "vy", "vz",
            "wx", "wy", "wz"
        ])

        while reader.has_next():
            topic, data, t_nsec = reader.read_next()
            if topic != topic_cmd:
                continue

            time = t_nsec * 1e-9
            if first_time is None:
                first_time = time
            t_rel = time - first_time

            msg = rclpy.serialization.deserialize_message(data, CommandCTBR)
            pwm = msg.thrust_pwm
            grams = pwm_to_grams(pwm, c1, c2, c3, thrust_pwm_min, thrust_pwm_max)
            newtons = grams * 9.81 / 1000

            # Find closest odometry timestamp
            if not odom_data:
                print("⚠️ No odometry data found for synchronization.")
                break
            closest_t = min(odom_data.keys(), key=lambda k: abs(k - t_nsec))
            state = odom_data[closest_t]

            writer.writerow([
                f"{t_rel:.6f}",
                pwm,
                f"{grams:.3f}",
                f"{newtons:.4f}",
                msg.roll_rate,
                msg.pitch_rate,
                msg.yaw_rate,
                state["x"], state["y"], state["z"],
                state["roll"], state["pitch"], state["yaw"],
                state["vx"], state["vy"], state["vz"],
                state["wx"], state["wy"], state["wz"]
            ])

    print(f"✅ Saved to: {output_csv}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("\nUsage: python extract_ctbr_to_csv.py <bag_directory> <output_directory>")
        sys.exit(1)

    bag_dir = sys.argv[1]
    output_dir = sys.argv[2]
    extract_actions_to_csv(bag_dir, output_dir)
