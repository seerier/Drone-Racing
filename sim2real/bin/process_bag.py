import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from scipy.spatial.transform import Rotation as R
import os
import sys
from pathlib import Path # Using pathlib for more modern path manipulation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import yaml

# Ensure Pillow is installed for saving GIFs
try:
    import PIL
except ImportError:
    print("Pillow library not found. Please install it for saving GIFs: pip install Pillow")
    # Optionally exit if Pillow is strictly required
    # sys.exit(1)

figsize = (10, 8)

def set_axes_equal(ax):
  """Set equal scale for all axes."""
  x_limits = ax.get_xlim()
  y_limits = ax.get_ylim()
  z_limits = ax.get_zlim()

  x_range = abs(x_limits[1] - x_limits[0])
  y_range = abs(y_limits[1] - y_limits[0])
  z_range = abs(z_limits[1] - z_limits[0])

  # Handle cases where range is zero
  x_range = x_range if x_range > 1e-6 else 1.0
  y_range = y_range if y_range > 1e-6 else 1.0
  z_range = z_range if z_range > 1e-6 else 1.0


  max_range = max(x_range, y_range, z_range) / 2.0

  mid_x = np.mean(x_limits)
  mid_y = np.mean(y_limits)
  mid_z = np.mean(z_limits)

  ax.set_xlim(mid_x - max_range, mid_x + max_range)
  ax.set_ylim(mid_y - max_range, mid_y + max_range)
  ax.set_zlim(mid_z - max_range, mid_z + max_range)

def analyze_ros2_bag(bag_path, namespace, t0=0, tf=float('inf')):
  yaml_file = '/home/neo/workspace/src/jirl_bringup/config/config.yaml'
  with open(yaml_file, 'r') as f:
    data = yaml.safe_load(f)
  waypoints = np.array(data["/*/controller"]["ros__parameters"]["policy"]["waypoints"]).reshape(-1, 6)

  print("Waypoints:", waypoints)

  d = 0.5
  local_square = np.array([
      [0,  d,  d],
      [0, -d,  d],
      [0, -d, -d],
      [0,  d, -d]
  ])  # shape (4, 3)

  wp_pos = waypoints[:, :3]             # shape (N, 3)
  wp_euler = waypoints[:, 3:]           # shape (N, 3)

  rotations = R.from_euler('xyz', wp_euler).as_matrix()  # shape (N, 3, 3)
  # Use rot.T: einsum 'ij,nkj->nik' contracts j with the transposed rotation (same as local_square @ rot.T)
  verts_all = np.einsum('ij,nkj->nik', local_square, rotations) + wp_pos[:, np.newaxis, :]  # shape (N, 4, 3)

  # --- Path setup for saving plots ---
  bag_path_obj = Path(bag_path).resolve() # Get absolute path

  # Determine the base experiment name (directory name)
  # Handle based on whether input path is file or dir
  if bag_path_obj.is_file():
      experiment_dir = bag_path_obj.parent
  elif bag_path_obj.is_dir():
      experiment_dir = bag_path_obj
  else:
      # Fallback if the path doesn't exist yet (use input path structure)
      experiment_dir = Path(bag_path) # Use the input path directly

  experiment_name = experiment_dir.name
  if not experiment_name: # Handle root path case
      print(f"Warning: Cannot determine experiment name from path '{bag_path}'. Using 'unknown_experiment'.")
      experiment_name = "unknown_experiment"
      # Decide where to save plots if base path is weird
      if experiment_dir.parent == experiment_dir: # Check if it's root
           plots_parent_dir = Path("./plots") # Save in current dir subfolder
      else:
           plots_parent_dir = experiment_dir.parent / 'plots'
  else:
      # Go up one level from experiment_dir to find the base 'logs' dir
      logs_base_dir = experiment_dir.parent
      plots_parent_dir = logs_base_dir / 'plots'

  output_plot_dir = plots_parent_dir / experiment_name
  output_plot_dir.mkdir(parents=True, exist_ok=True) # Create directory if it doesn't exist
  print(f"Saving plots and animation to: {output_plot_dir}")
  # --- End Path setup ---

  reader = rosbag2_py.SequentialReader()

  # --- Improved Determine storage_id and uri based on bag_path ---
  resolved_bag_path = Path(bag_path).resolve()
  bag_uri = str(resolved_bag_path) # Default URI is the resolved input path
  storage_id = None # Start with None, force explicit detection

  if not resolved_bag_path.exists():
      # Path doesn't exist. Guess based on expected suffix or directory structure.
      print(f"Warning: Bag path '{bag_path}' does not exist. Guessing storage format based on name.")
      if bag_path.endswith('.mcap'):
          storage_id = 'mcap'
          bag_uri = bag_path # Use the original path string as URI
      elif bag_path.endswith('.db3'):
           storage_id = 'sqlite3'
           bag_uri = bag_path # Use the original path string as URI
      elif '.' not in Path(bag_path).name: # Assume it's intended as a directory for sqlite3
          storage_id = 'sqlite3'
          bag_uri = bag_path # Use the original path string as URI
      else:
          print(f"Error: Cannot determine storage type for non-existent path with unknown extension: {bag_path}")
          sys.exit(1)

  elif resolved_bag_path.is_dir():
      print(f"Input path '{bag_path}' is a directory. Checking contents...")
      mcap_files = list(resolved_bag_path.glob('*.mcap'))
      db3_files = list(resolved_bag_path.glob('*.db3'))
      metadata_file = resolved_bag_path / 'metadata.yaml'

      if mcap_files:
          if len(mcap_files) > 1:
              print(f"Warning: Multiple MCAP files found in {resolved_bag_path}. Using the first one: {mcap_files[0].name}")
          print(f"Found MCAP file: {mcap_files[0].name}. Setting storage_id='mcap' and URI to the file path.")
          storage_id = 'mcap'
          bag_uri = str(mcap_files[0].resolve()) # CRITICAL: URI must be the MCAP file itself, resolved
      elif db3_files:
          # If db3 files exist, the URI should be the directory itself.
          print(f"Found db3 file(s): {[f.name for f in db3_files]}. Setting storage_id='sqlite3' and URI to the directory path.")
          storage_id = 'sqlite3'
          bag_uri = str(resolved_bag_path) # URI is the directory for sqlite3
      elif metadata_file.exists():
           # If only metadata exists, assume sqlite3 (common case for ongoing recordings or bags without db3 yet)
           print(f"Found metadata.yaml, assuming 'sqlite3' storage with directory URI.")
           storage_id = 'sqlite3'
           bag_uri = str(resolved_bag_path)
      else:
           # No mcap, no db3, no metadata... ambiguous.
           print(f"Error: Could not determine storage type for directory {resolved_bag_path}. No .mcap, .db3, or metadata.yaml found.")
           sys.exit(1)

  elif resolved_bag_path.is_file():
      if resolved_bag_path.suffix == '.mcap':
          print("Input path is an MCAP file. Setting storage_id='mcap'.")
          storage_id = 'mcap'
          bag_uri = str(resolved_bag_path) # URI is the file
      elif resolved_bag_path.suffix == '.db3':
          print("Input path is a DB3 file. Setting storage_id='sqlite3'.")
          storage_id = 'sqlite3'
          # For sqlite3, rosbag2_py generally expects the *directory* containing the db3.
          # Using the file path directly *might* work in some versions, but using the parent dir is safer.
          bag_uri = str(resolved_bag_path.parent)
          print(f"Note: For .db3 file, using parent directory as URI: {bag_uri}")
      else:
          print(f"Error: Input path is a file with unrecognized suffix: {resolved_bag_path.suffix}")
          sys.exit(1)

  # Final check if detection failed somehow
  if storage_id is None:
      print("Error: Could not determine storage ID after checking path and contents.")
      sys.exit(1)

  print(f"Using storage ID: '{storage_id}' for URI: '{bag_uri}'")
  storage_options = rosbag2_py.StorageOptions(uri=bag_uri, storage_id=storage_id)
  converter_options = rosbag2_py.ConverterOptions()
  # --- End storage_id determination ---

  try:
    reader.open(storage_options, converter_options)
  except Exception as e:
    print(f"Error opening bag file: {e}")
    print("Please ensure the bag path and format are correct, and the bag is not corrupted.")
    print(f"Attempted Path/URI: {bag_uri}")
    print(f"Attempted Storage ID: {storage_id}")
    # Print available storage plugins hint
    try:
        from rosbag2_py import get_registered_readers
        print(f"Available storage readers: {get_registered_readers()}")
    except Exception:
        pass # Ignore if this fails
    return # Exit function if bag cannot be opened

  topic_types = reader.get_all_topics_and_types()
  type_map = {topic.name: topic.type for topic in topic_types}

  print("\nAvailable topics:")
  for topic, msg_type in type_map.items():
    print(f"- {topic}: {msg_type}")

  # Check if required topics exist
  required_topics = [f"/{namespace}/observations", f"/{namespace}/odom", f"/{namespace}/ctbr_cmd", f"/{namespace}/trajectory", "/ctbr_cmd"]
  missing_topics = []
  # Check topics with and without leading slash for robustness
  for req_topic in required_topics:
      base_topic = req_topic.lstrip('/')
      if req_topic not in type_map and f"/{base_topic}" not in type_map:
          # Check if maybe the namespace itself has a leading slash issue
           if req_topic.startswith(f"/{namespace}/") and f"{namespace}/{req_topic.split(f'/{namespace}/', 1)[1]}" not in type_map:
               missing_topics.append(req_topic)
           elif not req_topic.startswith(f"/{namespace}/"): # Global topics like /ctbr_cmd
                 if req_topic not in type_map and req_topic.lstrip('/') not in type_map:
                      missing_topics.append(req_topic)


  if missing_topics:
      print("\nWarning: The following required topics (or variants) were not found in the bag:")
      for t in missing_topics:
          print(f"- {t}")
      # Decide whether to continue or exit - let's continue for now

  timestamps = []
  timestamps_cmd = []
  timestamps_traj = []
  timestamps_obs = []
  gt_pos = {"x": [], "y": [], "z": []}
  gt_quat = {"x": [], "y": [], "z": [], "w": []}
  gt_euler = {"roll": [], "pitch": [], "yaw": []}
  gt_lin_vel = {"x": [], "y": [], "z": []}
  gt_ang_vel = {"x": [], "y": [], "z": []}
  thrust_pwm = []
  thrust_N = []
  roll_rate = []
  pitch_rate = []
  yaw_rate = []
  dist_next_gate = []
  traj = {"x": [], "x_dot": [], "x_ddot": [], "x_dddot": [], "x_ddddot": [],
          "yaw": [], "yaw_dot": [], "yaw_ddot": []}

  first_timestamp = None

  print("\nReading bag data...")
  message_count = 0
  processed_count = 0
  while reader.has_next():
    try:
        (topic, data, timestamp_ns) = reader.read_next()
        message_count += 1
        timestamp_sec = timestamp_ns * 1e-9 # nanoseconds to seconds

        if first_timestamp is None:
          first_timestamp = timestamp_sec

        rel_time = timestamp_sec - first_timestamp
        if rel_time < t0 or rel_time > tf:
          continue # Skip message if outside time range

        processed_count += 1
        # Normalize topic name (remove leading slash if present for comparison)
        normalized_topic = topic.lstrip('/')

        if topic not in type_map:
            # This might happen with late-discovered topics?
            # Try to get type dynamically if possible, otherwise skip
            print(f"Warning: Skipping message from topic '{topic}' not initially listed in type map.")
            continue

        msg_type = type_map[topic]
        try:
            msg_class = get_message(msg_type)
            message = deserialize_message(data, msg_class)
        except Exception as e:
            print(f"Error deserializing message for topic {topic} ({msg_type}): {e}")
            continue # Skip this message

        # --- Topic processing ---
        odom_topic = f"{namespace}/odom".lstrip('/')
        cmd_topic = f"/{namespace}/ctbr_cmd".lstrip('/')
        traj_topic = f"{namespace}/trajectory".lstrip('/')
        obs_topic = f"/{namespace}/observations".lstrip('/')

        if normalized_topic == odom_topic:
          timestamps.append(rel_time)
          gt_pos["x"].append(message.pose.pose.position.x)
          gt_pos["y"].append(message.pose.pose.position.y)
          gt_pos["z"].append(message.pose.pose.position.z)
          gt_quat["x"].append(message.pose.pose.orientation.x)
          gt_quat["y"].append(message.pose.pose.orientation.y)
          gt_quat["z"].append(message.pose.pose.orientation.z)
          gt_quat["w"].append(message.pose.pose.orientation.w)

          quat = [
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w
          ]
          rot = R.from_quat(quat)
          rot_T = rot.as_matrix().T

          lin_vel_world = np.array([
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z
          ])
          ang_vel_world = np.array([
            message.twist.twist.angular.x,
            message.twist.twist.angular.y,
            message.twist.twist.angular.z
          ])

          lin_vel_body = rot_T @ lin_vel_world
          ang_vel_body = rot_T @ ang_vel_world * 180.0 / np.pi

          gt_lin_vel["x"].append(lin_vel_body[0])
          gt_lin_vel["y"].append(lin_vel_body[1])
          gt_lin_vel["z"].append(lin_vel_body[2])
          gt_ang_vel["x"].append(ang_vel_body[0])
          gt_ang_vel["y"].append(ang_vel_body[1])
          gt_ang_vel["z"].append(ang_vel_body[2])




        elif normalized_topic == cmd_topic:
           # Check if message has crazyflie_name (assuming specific message type)
           # Make the check more robust in case the attribute doesn't exist
           cf_name_in_msg = getattr(message, 'crazyflie_name', None) # Returns None if not present
           # Check if the attribute exists AND if the namespace is in the list/string
           if cf_name_in_msg is not None and namespace in cf_name_in_msg:
                timestamps_cmd.append(rel_time)
                # Check for attributes before accessing to prevent errors on malformed msgs
                thrust_pwm.append(getattr(message, 'thrust_pwm', float('nan')))
                thrust_N.append(getattr(message, 'thrust_n', float('nan')))
                roll_rate.append(getattr(message, 'roll_rate', float('nan')))
                pitch_rate.append(getattr(message, 'pitch_rate', float('nan')))
                yaw_rate.append(getattr(message, 'yaw_rate', float('nan')))
           # If the attribute doesn't exist, maybe it's a generic command? Or log warning.
           elif cf_name_in_msg is None:
                # Decide: skip, process anyway, or warn? Let's warn and process.
                # print(f"Warning: /ctbr_cmd message at {rel_time:.2f}s lacks 'crazyflie_name'. Assuming it applies to {namespace}.")
                timestamps_cmd.append(rel_time)
                thrust_pwm.append(getattr(message, 'thrust_pwm', float('nan')))
                thrust_N.append(getattr(message, 'thrust_n', float('nan')))
                roll_rate.append(getattr(message, 'roll_rate', float('nan')))
                pitch_rate.append(getattr(message, 'pitch_rate', float('nan')))
                yaw_rate.append(getattr(message, 'yaw_rate', float('nan')))

        elif normalized_topic == traj_topic:
          timestamps_traj.append(rel_time)
          # Check for attributes before accessing
          traj["x"].append(getattr(message, 'x', [float('nan')]*3)) # Default to NaN array if missing
          traj["x_dot"].append(getattr(message, 'x_dot', [float('nan')]*3))
          traj["x_ddot"].append(getattr(message, 'x_ddot', [float('nan')]*3))
          traj["x_dddot"].append(getattr(message, 'x_dddot', [float('nan')]*3))
          traj["x_ddddot"].append(getattr(message, 'x_ddddot', [float('nan')]*3))
          traj["yaw"].append(getattr(message, 'yaw', float('nan')))
          traj["yaw_dot"].append(getattr(message, 'yaw_dot', float('nan')))
          traj["yaw_ddot"].append(getattr(message, 'yaw_ddot', float('nan')))

        elif normalized_topic == 'ctbr_cmd':
            # Check if message has crazyflie_name (assuming specific message type)
            # Make the check more robust in case the attribute doesn't exist
            cf_name_in_msg = getattr(message, 'crazyflie_name', None) # Returns None if not present
            # Check if the attribute exists AND if the namespace is in the list/string
            if cf_name_in_msg is not None and namespace in cf_name_in_msg:
                timestamps_cmd.append(rel_time)
                # Check for attributes before accessing to prevent errors on malformed msgs
                thrust_pwm.append(getattr(message, 'thrust_pwm', float('nan')))
                thrust_N.append(getattr(message, 'thrust_n', float('nan')))
                roll_rate.append(getattr(message, 'roll_rate', float('nan')))
                pitch_rate.append(getattr(message, 'pitch_rate', float('nan')))
                yaw_rate.append(getattr(message, 'yaw_rate', float('nan')))
            # If the attribute doesn't exist, maybe it's a generic command? Or log warning.
            elif cf_name_in_msg is None:
                # Decide: skip, process anyway, or warn? Let's warn and process.
                # print(f"Warning: /ctbr_cmd message at {rel_time:.2f}s lacks 'crazyflie_name'. Assuming it applies to {namespace}.")
                timestamps_cmd.append(rel_time)
                thrust_pwm.append(getattr(message, 'thrust_pwm', float('nan')))
                thrust_N.append(getattr(message, 'thrust_n', float('nan')))
                roll_rate.append(getattr(message, 'roll_rate', float('nan')))
                pitch_rate.append(getattr(message, 'pitch_rate', float('nan')))
                yaw_rate.append(getattr(message, 'yaw_rate', float('nan')))

        elif normalized_topic == obs_topic:
            timestamps_obs.append(rel_time)
            corners = getattr(message, 'corners_pos_b_curr', float('nan')).reshape(4,3)
            mean_point = corners.mean(axis=0)
            dist_next_gate.append(np.linalg.norm(mean_point))

    except StopIteration: # This is expected when the reader finishes
        break
    except Exception as e:
        # Catch other potential errors during reading/processing loop
        print(f"\nAn error occurred while reading or processing message #{message_count}: {e}")
        print("Topic:", topic)
        print("Timestamp (ns):", timestamp_ns)
        # Decide whether to stop or continue (continuing might lead to incomplete data)
        # break
        continue # Try processing next message

  print(f"Finished reading bag data. Read {message_count} messages total, processed {processed_count} within time range [{t0}, {tf}].")

  # -- Print initial pose ---
  quat = [gt_quat["x"][0], gt_quat["y"][0], gt_quat["z"][0], gt_quat["w"][0]]
  rpy = R.from_quat(quat).as_euler('xyz', degrees=True)  # 'xyz' = roll, pitch, yaw

  print("Initial pose:")
  print(f"Position: x = {gt_pos['x'][0]:.3f}, y = {gt_pos['y'][0]:.3f}, z = {gt_pos['z'][0]:.3f}")
  print(f"Orientation (RPY): roll = {rpy[0]:.3f}°, pitch = {rpy[1]:.3f}°, yaw = {rpy[2]:.3f}°")

  # --- Data processing and Plotting ---

  # Check if essential data was loaded
  if not timestamps:
      print("\nError: No odometry data (topic: /{}/odom) found for the specified namespace and time range.".format(namespace))
      print("Cannot generate plots.")
      return

  # Convert trajectory lists to numpy arrays if they contain data
  if traj["x"]:
      try:
          traj["x"] = np.vstack(traj["x"])
          traj["x_dot"] = np.vstack(traj["x_dot"])
          traj["x_ddot"] = np.vstack(traj["x_ddot"])
          traj["x_dddot"] = np.vstack(traj["x_dddot"])
          traj["x_ddddot"] = np.vstack(traj["x_ddddot"])
      except ValueError as e:
          print(f"\nWarning: Could not stack trajectory arrays, likely due to inconsistent shapes or NaNs: {e}")
          print("Trajectory plots might be incorrect or fail.")
          # Set to empty arrays to prevent downstream errors if stacking fails
          for k in traj: traj[k] = np.array([])
      # Check if trajectory array is actually usable after stacking
      if not traj["x"].size:
           print("Warning: Trajectory data was found but resulted in empty arrays after processing.")

  else:
      print("\nWarning: No trajectory data (topic: /{}/trajectory) found or processed.".format(namespace))
      # Ensure keys exist but are empty numpy arrays for consistency downstream
      for k in traj: traj[k] = np.array([])


  # Convert quaternion to Euler angles
  if gt_quat["x"]:
      quaternions = np.column_stack((gt_quat["x"], gt_quat["y"], gt_quat["z"], gt_quat["w"]))
      # Check for invalid quaternions (e.g., all zeros, NaNs) before conversion
      valid_quat_mask = np.all(np.isfinite(quaternions), axis=1) & (np.linalg.norm(quaternions, axis=1) > 1e-6)
      if not np.all(valid_quat_mask):
          print(f"\nWarning: Found {len(quaternions) - np.sum(valid_quat_mask)} invalid quaternions (NaNs or zero norm). Replacing with identity/NaN.")
          # Option 1: Replace invalid ones with identity (0,0,0,1) -> (0,0,0) Euler
          # quaternions[~valid_quat_mask] = [0, 0, 0, 1]
          # Option 2: Keep them as NaN Euler angles (perhaps better to show data issues)
          euler_angles = np.full((len(quaternions), 3), np.nan) # Initialize with NaNs
          try:
               if np.any(valid_quat_mask): # Only convert if there are valid ones
                    valid_quats = quaternions[valid_quat_mask]
                    euler_angles[valid_quat_mask] = R.from_quat(valid_quats).as_euler('xyz', degrees=True)
          except ValueError as e:
               print(f"Error converting valid quaternions to Euler angles: {e}. Euler angles will contain NaNs.")
               # Keep euler_angles as initialized with NaNs
      else:
            # All quaternions are valid
            try:
                euler_angles = R.from_quat(quaternions).as_euler('xyz', degrees=True)
            except ValueError as e:
                print(f"Error converting quaternions to Euler angles: {e}. Filling with NaNs.")
                euler_angles = np.full((len(quaternions), 3), np.nan)

      gt_euler["roll"] = euler_angles[:, 0].tolist()
      gt_euler["pitch"] = euler_angles[:, 1].tolist()
      gt_euler["yaw"] = euler_angles[:, 2].tolist()

  else:
      print("\nWarning: No quaternion data found for Euler conversion.")
      # Ensure gt_euler lists have the same length as timestamps if needed elsewhere, filled with NaN
      nan_list = [np.nan] * len(timestamps)
      gt_euler = {"roll": nan_list, "pitch": nan_list, "yaw": nan_list}


  print("\nGenerating plots...")

  # --- Ground truth data plot ---
  fig_gt, axs_gt = plt.subplots(4, 1, figsize=figsize, sharex=True)
  fig_gt.suptitle(f"Ground Truth Data ({namespace})")

  axs_gt[0].plot(timestamps, gt_pos["x"], label="$x$")
  axs_gt[0].plot(timestamps, gt_pos["y"], label="$y$")
  axs_gt[0].plot(timestamps, gt_pos["z"], label="$z$")
  axs_gt[0].set_ylabel("Position [m]")
  axs_gt[0].legend()
  axs_gt[0].grid(True)

  axs_gt[1].plot(timestamps, gt_euler["roll"], label="Roll")
  axs_gt[1].plot(timestamps, gt_euler["pitch"], label="Pitch")
  axs_gt[1].plot(timestamps, gt_euler["yaw"], label="Yaw")
  axs_gt[1].set_ylabel("Euler Angles [deg]")
  axs_gt[1].legend()
  axs_gt[1].grid(True)

  axs_gt[2].plot(timestamps, gt_lin_vel["x"], label="$v_{x}$")
  axs_gt[2].plot(timestamps, gt_lin_vel["y"], label="$v_{y}$")
  axs_gt[2].plot(timestamps, gt_lin_vel["z"], label="$v_{z}$")
  axs_gt[2].set_ylabel("Linear Velocity [m/s]")
  axs_gt[2].legend()
  axs_gt[2].grid(True)

  axs_gt[3].plot(timestamps, gt_ang_vel["x"], label=r"$\omega_{x}$")
  axs_gt[3].plot(timestamps, gt_ang_vel["y"], label=r"$\omega_{y}$")
  axs_gt[3].plot(timestamps, gt_ang_vel["z"], label=r"$\omega_{z}$")
  axs_gt[3].set_xlabel("Time [s]")
  axs_gt[3].set_ylabel("Angular Velocity [deg/s]")
  axs_gt[3].legend()
  axs_gt[3].grid(True)

  fig_gt.tight_layout(rect=[0, 0.03, 1, 0.97]) # Adjust layout to prevent title overlap
  gt_filename = output_plot_dir / f"{namespace}_ground_truth"
  try:
      print(f"Saving ground truth plot to {gt_filename}")
      fig_gt.savefig(gt_filename.with_suffix('.eps'), format='eps', bbox_inches='tight', dpi=300, facecolor=fig_gt.get_facecolor())
      fig_gt.savefig(gt_filename.with_suffix('.png'), format='png', bbox_inches='tight', dpi=300, facecolor=fig_gt.get_facecolor())
  except Exception as e:
      print(f"Error saving ground truth plot: {e}")

  # --- Positions comparison plot ---
  if timestamps_traj and traj["x"].ndim == 2 and traj["x"].shape[0] > 0: # Check if traj data is valid 2D array
    fig_pos, axs_pos = plt.subplots(3, 1, figsize=figsize, sharex=True)
    fig_pos.suptitle(f"Position Comparison ({namespace})")
    axs_pos[0].plot(timestamps, gt_pos["x"], label="Actual")
    axs_pos[0].plot(timestamps_traj, traj["x"][:, 0], label="Desired", linestyle='--')
    axs_pos[0].set_ylabel("x [m]")
    axs_pos[0].legend()
    axs_pos[0].grid(True)

    axs_pos[1].plot(timestamps, gt_pos["y"], label="Actual")
    axs_pos[1].plot(timestamps_traj, traj["x"][:, 1], label="Desired", linestyle='--')
    axs_pos[1].set_ylabel("y [m]")
    axs_pos[1].legend()
    axs_pos[1].grid(True)

    axs_pos[2].plot(timestamps, gt_pos["z"], label="Actual")
    axs_pos[2].plot(timestamps_traj, traj["x"][:, 2], label="Desired", linestyle='--')
    axs_pos[2].set_xlabel("Time [s]")
    axs_pos[2].set_ylabel("z [m]")
    axs_pos[2].legend()
    axs_pos[2].grid(True)

    fig_pos.tight_layout(rect=[0, 0.03, 1, 0.97])
    pos_filename = output_plot_dir / f"{namespace}_position_comparison"
    try:
        print(f"Saving position comparison plot to {pos_filename}")
        fig_pos.savefig(pos_filename.with_suffix('.eps'), format='eps', bbox_inches='tight', dpi=300, facecolor=fig_pos.get_facecolor())
        fig_pos.savefig(pos_filename.with_suffix('.png'), format='png', bbox_inches='tight', dpi=300, facecolor=fig_pos.get_facecolor())
    except Exception as e:
        print(f"Error saving position comparison plot: {e}")
  else:
    print("Skipping position comparison plot (no valid trajectory data found/processed).")


  # --- Angular velocities comparison plot ---
  if timestamps_cmd: # Check if there's command data to compare
    fig_rates, axs_rates = plt.subplots(3, 1, figsize=figsize, sharex=True)
    fig_rates.suptitle(f"Angular Velocity Comparison ({namespace})")

    axs_rates[0].plot(timestamps, gt_ang_vel["x"], label="Actual")
    axs_rates[0].plot(timestamps_cmd, roll_rate, label="Desired", linestyle='--')
    axs_rates[0].set_ylabel("Roll Rate [deg/s]")
    axs_rates[0].legend()
    axs_rates[0].grid(True)

    axs_rates[1].plot(timestamps, gt_ang_vel["y"], label="Actual")
    axs_rates[1].plot(timestamps_cmd, pitch_rate, label="Desired", linestyle='--')
    axs_rates[1].set_ylabel("Pitch Rate [deg/s]")
    axs_rates[1].legend()
    axs_rates[1].grid(True)

    axs_rates[2].plot(timestamps, gt_ang_vel["z"], label="Actual")
    axs_rates[2].plot(timestamps_cmd, yaw_rate, label="Desired", linestyle='--')
    axs_rates[2].set_xlabel("Time [s]")
    axs_rates[2].set_ylabel("Yaw Rate [deg/s]")
    axs_rates[2].legend()
    axs_rates[2].grid(True)

    fig_rates.tight_layout(rect=[0, 0.03, 1, 0.97])
    rates_filename = output_plot_dir / f"{namespace}_angular_velocity_comparison"
    try:
        print(f"Saving angular velocity comparison plot to {rates_filename}")
        fig_rates.savefig(rates_filename.with_suffix('.eps'), format='eps', bbox_inches='tight', dpi=300, facecolor=fig_rates.get_facecolor())
        fig_rates.savefig(rates_filename.with_suffix('.png'), format='png', bbox_inches='tight', dpi=300, facecolor=fig_rates.get_facecolor())
    except Exception as e:
        print(f"Error saving angular velocity plot: {e}")
  else:
      print("Skipping angular velocity comparison plot (no command data found/processed).")

  # --- CTBR data plot ---
  if timestamps_cmd:
    fig_ctbr, axs_ctbr = plt.subplots(3, 1, figsize=figsize, sharex=True)
    fig_ctbr.suptitle(f"Commanded CTBR Data ({namespace})")

    axs_ctbr[0].plot(timestamps_cmd, thrust_pwm)
    axs_ctbr[0].set_ylabel("Thrust PWM")
    axs_ctbr[0].grid(True)

    axs_ctbr[1].plot(timestamps_cmd, thrust_N)
    axs_ctbr[1].set_ylabel("Thrust [N]")
    axs_ctbr[1].grid(True)

    axs_ctbr[2].plot(timestamps_cmd, roll_rate, label="Roll Rate Cmd")
    axs_ctbr[2].plot(timestamps_cmd, pitch_rate, label="Pitch Rate Cmd")
    axs_ctbr[2].plot(timestamps_cmd, yaw_rate, label="Yaw Rate Cmd")
    axs_ctbr[2].set_xlabel("Time [s]")
    axs_ctbr[2].set_ylabel("Commanded Rates [deg/s]")
    axs_ctbr[2].legend()
    axs_ctbr[2].grid(True)

    fig_ctbr.tight_layout(rect=[0, 0.03, 1, 0.97])
    ctbr_filename = output_plot_dir / f"{namespace}_ctbr_commands"
    try:
        print(f"Saving CTBR command plot to {ctbr_filename}")
        fig_ctbr.savefig(ctbr_filename.with_suffix('.eps'), format='eps', bbox_inches='tight', dpi=300, facecolor=fig_ctbr.get_facecolor())
        fig_ctbr.savefig(ctbr_filename.with_suffix('.png'), format='png', bbox_inches='tight', dpi=300, facecolor=fig_ctbr.get_facecolor())
    except Exception as e:
        print(f"Error saving CTBR plot: {e}")
  else:
    print("Skipping CTBR command plot (no command data found/processed).")

  # --- Observations ---
  times_pass_gates = []
  if timestamps_obs:
    fig_obs, axs_obs = plt.subplots(2, 1, figsize=figsize, sharex=True)
    fig_obs.suptitle(f"Distance from next gate ({namespace})")

    axs_obs[0].plot(timestamps_obs, dist_next_gate)
    axs_obs[0].set_ylabel("m")
    axs_obs[0].grid(True)

    fig_obs.tight_layout(rect=[0, 0.03, 1, 0.97])
    obs_filename = output_plot_dir / f"{namespace}_obs_commands"

    # Detect gate passing times
    for i in range(1, len(dist_next_gate)):
        if abs(dist_next_gate[i-1] - dist_next_gate[i]) > 0.3:
            times_pass_gates.append(timestamps_obs[i])

    try:
        print(f"Saving obs command plot to {obs_filename}")
        fig_obs.savefig(obs_filename.with_suffix('.eps'), format='eps', bbox_inches='tight', dpi=300, facecolor=fig_obs.get_facecolor())
        fig_obs.savefig(obs_filename.with_suffix('.png'), format='png', bbox_inches='tight', dpi=300, facecolor=fig_obs.get_facecolor())
    except Exception as e:
        print(f"Error saving obs plot: {e}")
  else:
    print("Skipping obs command plot (no command data found/processed).")

#   # --- CTBR and position
#   def normalize_to_unit_range(x):
#     """Scala l’array x tra -1 e 1."""
#     x = np.array(x, dtype=float)
#     x_min, x_max = np.min(x), np.max(x)
#     if x_max == x_min:
#         return np.zeros_like(x)
#     return 2 * (x - x_min) / (x_max - x_min) - 1

#   thrust_n    = normalize_to_unit_range(thrust_pwm)
#   roll_n      = normalize_to_unit_range(roll_rate)
#   pitch_n     = normalize_to_unit_range(pitch_rate)
#   yaw_n       = normalize_to_unit_range(yaw_rate)

#   fig_ctbr_pos, ax = plt.subplots(2, 1, figsize=figsize)
#   fig_ctbr_pos.suptitle(f"CTBR vs Position ({namespace})")

#   gt_pos_x = np.array(gt_pos["x"])
#   gt_pos_y = np.array(gt_pos["y"])
#   gt_pos_z = np.array(gt_pos["z"])

#   # Filtro: solo dati tra 3 e 4 secondi
#   mask_cmd = (np.array(timestamps_cmd) >= 0.0) & (np.array(timestamps_cmd) <= 6.0)
#   mask_gt = (np.array(timestamps) >= 0.0) & (np.array(timestamps) <= 6.0)

#   # Plot normalized commands
#   ax[0].plot(np.array(timestamps_cmd)[mask_cmd], thrust_n[mask_cmd], label="Thrust PWM")
#   ax[0].plot(np.array(timestamps_cmd)[mask_cmd], roll_n[mask_cmd],   label="Roll Rate Cmd")
#   ax[0].plot(np.array(timestamps_cmd)[mask_cmd], pitch_n[mask_cmd],  label="Pitch Rate Cmd")
#   ax[0].plot(np.array(timestamps_cmd)[mask_cmd], yaw_n[mask_cmd],    label="Yaw Rate Cmd")

#   ax[0].set_xlabel("Time [s]")
#   ax[0].set_ylabel("Normalized Commands")
#   ax[0].set_ylim(-1.1, 1.1)
#   ax[0].legend()
#   ax[0].grid(True)

#   # Plot position
#   print(gt_pos_x[0], gt_pos_y[0], gt_euler['yaw'][0])
#   input()
#   n = 180
#   ax[1].plot(np.array(timestamps)[mask_gt][n:], gt_pos_x[mask_gt][n:], label="X")
#   ax[1].plot(np.array(timestamps)[mask_gt][n:], gt_pos_y[mask_gt][n:], label="Y")
#   ax[1].plot(np.array(timestamps)[mask_gt][n:], gt_pos_z[mask_gt][n:], label="Z")

#   ax[1].set_xlabel("Time [s]")
#   ax[1].set_ylabel("Position")
#   ax[1].legend()
#   ax[1].grid(True)

#   fig_ctbr_pos.tight_layout(rect=[0, 0.03, 1, 0.97])
#   # ctbr_filename = output_plot_dir / f"{namespace}_ctbr_commands"
#   # try:
#   #     print(f"Saving CTBR command plot to {ctbr_filename}")
#   #     fig_ctbr_pos.savefig(ctbr_filename.with_suffix('.eps'), format='eps', bbox_inches='tight', dpi=300, facecolor=fig_ctbr.get_facecolor())
#   #     fig_ctbr_pos.savefig(ctbr_filename.with_suffix('.png'), format='png', bbox_inches='tight', dpi=300, facecolor=fig_ctbr.get_facecolor())
#   # except Exception as e:
#   #     print(f"Error saving CTBR plot: {e}")
#   # else:
#   #   print("Skipping CTBR command plot (no command data found/processed).")


  # --- Trajectory 3D plot ---
  fig3d = plt.figure(figsize=figsize)
  ax3d = fig3d.add_subplot(111, projection='3d')
  # Plot actual trajectory only if position data exists
  if gt_pos["x"]:
       ax3d.plot(gt_pos["x"], gt_pos["y"], gt_pos["z"], label="Actual Trajectory")

  # Plot desired trajectory if available and valid
  if timestamps_traj and traj["x"].ndim == 2 and traj["x"].shape[0] > 0:
       ax3d.plot(traj["x"][:, 0], traj["x"][:, 1], traj["x"][:, 2], label="Desired Trajectory", linestyle='--', color='orange')

  # Plot red points at gate intersection times
  if gt_pos["x"] and timestamps and times_pass_gates:
      t_array = np.array(timestamps)
      x_array = np.array(gt_pos["x"])
      y_array = np.array(gt_pos["y"])
      z_array = np.array(gt_pos["z"])

      for i, t_gate in enumerate(times_pass_gates):
          idx = np.argmin(np.abs(t_array - t_gate))
          ax3d.scatter(x_array[idx], y_array[idx], z_array[idx], color='red', s=50, marker='o',
                       label="Gate pass" if i == 0 else None)

  ax3d.set_xlabel("x [m]")
  ax3d.set_ylabel("y [m]")
  ax3d.set_zlabel("z [m]")
  ax3d.set_title(f"3D Position Trajectory ({namespace})")
  ax3d.legend()
  # Apply equal scaling only if there's data plotted
  if gt_pos["x"] or (timestamps_traj and traj["x"].ndim == 2 and traj["x"].shape[0] > 0):
      set_axes_equal(ax3d) # Apply equal scaling after plotting

  traj3d_filename = output_plot_dir / f"{namespace}_trajectory_3d"
  try:
      print(f"Saving 3D trajectory plot to {traj3d_filename}")
      fig3d.savefig(traj3d_filename.with_suffix('.eps'), format='eps', bbox_inches='tight', dpi=300, facecolor=fig3d.get_facecolor())
      fig3d.savefig(traj3d_filename.with_suffix('.png'), format='png', bbox_inches='tight', dpi=300, facecolor=fig3d.get_facecolor())
  except Exception as e:
      print(f"Error saving 3D trajectory plot: {e}")


  # --- Animation ---
  print("\nGenerating animation...")
  # Need timestamps AND valid euler angles (check if list isn't just NaNs)
  if not timestamps or not gt_euler["roll"] or all(np.isnan(gt_euler["roll"])):
        print("Skipping animation generation due to missing timestamps or valid orientation data.")
  else:
      def rotate_points(points, roll, pitch, yaw):
        # Ensure angles are valid numbers before attempting rotation
        if any(np.isnan([roll, pitch, yaw])):
            # print(f"Warning: NaN Euler angle encountered. Using identity rotation.") # Optional warning
            return points # Return original points if rotation is not possible
        try:
            # Filter out potentially huge angle values if they are unrealistic outliers
            # if abs(roll)>360*5 or abs(pitch)>360*5 or abs(yaw)>360*5:
            #     print(f"Warning: Large Euler angle encountered ({roll:.1f}, {pitch:.1f}, {yaw:.1f}). Using identity.")
            #     return points

            r = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
            return r.apply(points)
        except ValueError as e:
             # This might happen with extreme gimbal lock angles, though 'xyz' is usually robust
             print(f"Warning: Skipping rotation due to invalid Euler angles for SciPy: roll={roll}, pitch={pitch}, yaw={yaw} ({e})")
             return points # Return original points if rotation fails

      frame_idx = 0

      def update_animation(_):
        nonlocal paused, frame_idx, drone_quiver

        if paused or frame_idx >= len(timestamps):
            return drone_x, drone_y, trail, current_pos_marker, drone_quiver

        x = gt_pos["x"][frame_idx]
        y = gt_pos["y"][frame_idx]
        z = gt_pos["z"][frame_idx]
        roll = gt_euler["roll"][frame_idx]
        pitch = gt_euler["pitch"][frame_idx]
        yaw = gt_euler["yaw"][frame_idx]

        drone_size = 0.1
        base_points = np.array([
            [-drone_size, -drone_size, 0], [drone_size, drone_size, 0],
            [-drone_size, drone_size, 0], [drone_size, -drone_size, 0]
        ])
        rotated_points = rotate_points(base_points, roll, pitch, yaw)

        drone_x.set_data(
            [x + rotated_points[0, 0], x + rotated_points[1, 0]],
            [y + rotated_points[0, 1], y + rotated_points[1, 1]]
        )
        drone_x.set_3d_properties(
            [z + rotated_points[0, 2], z + rotated_points[1, 2]]
        )

        drone_y.set_data(
            [x + rotated_points[2, 0], x + rotated_points[3, 0]],
            [y + rotated_points[2, 1], y + rotated_points[3, 1]]
        )
        drone_y.set_3d_properties(
            [z + rotated_points[2, 2], z + rotated_points[3, 2]]
        )

        trail.set_data(gt_pos["x"][:frame_idx + 1], gt_pos["y"][:frame_idx + 1])
        trail.set_3d_properties(gt_pos["z"][:frame_idx + 1])

        current_pos_marker.set_data([x], [y])
        current_pos_marker.set_3d_properties([z])

        x_dir_body = rotate_points(np.array([[0.2, 0, 0]]), roll, pitch, yaw)[0]
        drone_quiver.remove()
        drone_quiver = anim_ax.quiver(
            x, y, z,
            x_dir_body[0], x_dir_body[1], x_dir_body[2],
            color='red', length=0.2, normalize=True
        )

        anim_ax.set_title(f"Drone Trajectory Animation ({namespace}) - Time: {timestamps[frame_idx]:.2f}s")

        frame_idx += 1
        return drone_x, drone_y, trail, current_pos_marker, drone_quiver


      # --- Set up Animation Plot ---
      anim_fig = plt.figure(figsize=figsize)
      def on_key(event):
        nonlocal paused
        if event.key == ' ':
          paused = not paused
          print("Paused" if paused else "Resumed")
      anim_fig.canvas.mpl_connect('key_press_event', on_key)

      anim_ax = anim_fig.add_subplot(111, projection='3d')

      # Determine axis limits from the *entire* dataset (actual and desired if present)
      all_x = list(gt_pos["x"])
      all_y = list(gt_pos["y"])
      all_z = list(gt_pos["z"])
      if timestamps_traj and traj["x"].ndim == 2 and traj["x"].shape[0] > 0:
            all_x.extend(traj["x"][:, 0])
            all_y.extend(traj["x"][:, 1])
            all_z.extend(traj["x"][:, 2])

      # Filter out NaNs before finding min/max
      all_x = [v for v in all_x if np.isfinite(v)]
      all_y = [v for v in all_y if np.isfinite(v)]
      all_z = [v for v in all_z if np.isfinite(v)]

      if not all_x or not all_y or not all_z:
          print("Error: No valid position data available for setting animation axes limits.")
          # Set default limits if no data is valid
          min_x, max_x, min_y, max_y, min_z, max_z = -1, 1, -1, 1, 0, 1
      else:
          min_x, max_x = min(all_x), max(all_x)
          min_y, max_y = min(all_y), max(all_y)
          min_z, max_z = min(all_z), max(all_z)

      # Add padding to limits
      x_range = max(max_x - min_x, 0.1) # Ensure range is not zero
      y_range = max(max_y - min_y, 0.1)
      z_range = max(max_z - min_z, 0.1)
      padding_x = x_range * 0.1
      padding_y = y_range * 0.1
      padding_z = z_range * 0.1

      anim_ax.set_xlim(min_x - padding_x, max_x + padding_x)
      anim_ax.set_ylim(min_y - padding_y, max_y + padding_y)
      anim_ax.set_zlim(max(0, min_z - padding_z), max_z + padding_z) # Ensure z starts >= 0

      # Calculate a reasonable size for the drone marker based on overall trajectory size
      drone_size = max(x_range, y_range, z_range) # Use max range as a scale factor

      set_axes_equal(anim_ax) # Apply equal scaling

      anim_ax.set_xlabel("x [m]")
      anim_ax.set_ylabel("y [m]")
      anim_ax.set_zlabel("z [m]")

      # Plot desired trajectory statically if available
      if timestamps_traj and traj["x"].ndim == 2 and traj["x"].shape[0] > 0:
            anim_ax.plot(traj["x"][:, 0], traj["x"][:, 1], traj["x"][:, 2], label="Desired Traj.", linestyle='--', color='orange', alpha=0.7)

      # Plot the full actual trajectory statically and faintly
      anim_ax.plot(gt_pos["x"], gt_pos["y"], gt_pos["z"], 'b-', linewidth=1, alpha=0.3, label="Full Actual Traj.")

      # Initialize animated elements (lines/markers)
      # Trail will grow, others will move/rotate
      trail, = anim_ax.plot([], [], [], 'b-', linewidth=1.5, label="Actual Trail") # Growing trail
      drone_x, = anim_ax.plot([], [], [], 'r-', linewidth=3) # Drone X marker
      drone_y, = anim_ax.plot([], [], [], 'g-', linewidth=3) # Drone Y marker
      current_pos_marker, = anim_ax.plot([], [], [], 'ko', markersize=4, label='Current Pos') # Current position dot
      drone_quiver = anim_ax.quiver(0, 0, 0, 0, 0, 0, color='red', length=0.2)
      anim_ax.legend(fontsize='small')

      # --- Calculate Animation Timing ---
      if len(timestamps) > 1:
          # Calculate median time difference between frames
          median_dt = np.median(np.diff(timestamps))
          # Ensure dt is positive and non-zero; default if calculation fails
          if not np.isfinite(median_dt) or median_dt <= 0:
              median_dt = 0.05 # Default to 50ms (20 FPS)
          interval_ms = max(1, int(median_dt * 1000)) # Interval in milliseconds
          fps = min(30, 1000.0 / interval_ms) # Calculate FPS, cap at 30 for GIF
      else:
          interval_ms = 50 # Default interval for single frame
          fps = 20 # Default FPS

      print(f"Animation settings: interval={interval_ms}ms, target save fps={fps:.1f}")

      num_frames = len(timestamps)

      paused = False

      # Create the animation object
      _ = animation.FuncAnimation(
          anim_fig,
          update_animation,
          frames=range(num_frames),
          interval=interval_ms,
          blit=False
      )

  # --- Show plots interactively at the end ---
  print("\nDisplaying plots...")
  plt.show()
  print("\nAnalysis complete.")
  plt.close('all') # Close all figures


if __name__ == "__main__":
  if len(sys.argv) < 3:
    print("\nUsage: python process_bag.py <bag_path_or_dir> <namespace> [t0] [tf]")
    print("  <bag_path_or_dir>: Path to the ROS2 bag directory or a specific .mcap/.db3 file.")
    print("  <namespace>:       The namespace of the vehicle (e.g., 'cf1', 'crazy_jirl_02').")
    print("  [t0]:              Optional start time in seconds relative to the bag start (default: 0).")
    print("  [tf]:              Optional end time in seconds relative to the bag start (default: inf).")
    print("\nExample (directory): python process_bag.py logs/20250403_test1 cf1")
    print("Example (mcap file): python process_bag.py logs/my_run.mcap cf2 5 25")
    print("Example (db3 file):  python process_bag.py logs/old_bag/data.db3 drone_A\n")
    sys.exit(1) # Exit if not enough arguments
  else:
    bag_path_arg = sys.argv[1]
    namespace_arg = sys.argv[2]
    t0_arg = float(sys.argv[3]) if len(sys.argv) > 3 else 0
    tf_arg = float(sys.argv[4]) if len(sys.argv) > 4 else float('inf')

    # --- Basic Validation ---
    # We now handle non-existent paths inside the main function's detection logic
    # if not Path(bag_path_arg).exists():
    #      print(f"Error: Bag path '{bag_path_arg}' not found.")
    #      sys.exit(1)

    if tf_arg <= t0_arg:
        print(f"Error: End time (tf={tf_arg}) must be greater than start time (t0={t0_arg}).")
        sys.exit(1)

    # Call the main analysis function
    analyze_ros2_bag(bag_path_arg, namespace_arg, t0_arg, tf_arg)