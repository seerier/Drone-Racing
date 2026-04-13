#!/usr/bin/env python3
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
from pathlib import Path
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import yaml

# IEEE publication formatting
# Use serif font - will fall back to available serif fonts if Times New Roman not installed
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman', 'DejaVu Serif', 'Liberation Serif', 'serif']
plt.rcParams['font.size'] = 10  # IEEE standard base font size
plt.rcParams['axes.labelsize'] = 10
plt.rcParams['axes.titlesize'] = 11
plt.rcParams['xtick.labelsize'] = 9
plt.rcParams['ytick.labelsize'] = 9
plt.rcParams['legend.fontsize'] = 9
plt.rcParams['figure.dpi'] = 300
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['text.usetex'] = False  # Set to True if LaTeX is available
plt.rcParams['axes.linewidth'] = 0.8
plt.rcParams['grid.linewidth'] = 0.5

# IEEE two-column width is 3.5 inches, single column is 7.16 inches
# For 3D plots, use square aspect ratio for better visibility
figsize = (3.5, 3.5)  # Single column width in inches

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

    # Find the maximum range
    max_range = max(x_range, y_range, z_range)

    # Calculate midpoints
    mid_x = np.mean(x_limits)
    mid_y = np.mean(y_limits)
    mid_z = np.mean(z_limits)

    # Set all axes to have the same range
    ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
    ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
    ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)

    # Force aspect ratio to be equal
    ax.set_box_aspect([1,1,1])

def process_drone_trajectory(bag_path, namespace, t0=0, tf=float('inf')):
    """Extract trajectory data for a single drone from a bag file."""

    reader = rosbag2_py.SequentialReader()

    # Determine storage format
    resolved_bag_path = Path(bag_path).resolve()
    bag_uri = str(resolved_bag_path)
    storage_id = None

    if not resolved_bag_path.exists():
        print(f"Warning: Bag path '{bag_path}' does not exist.")
        return None, None, None

    if resolved_bag_path.is_dir():
        mcap_files = list(resolved_bag_path.glob('*.mcap'))
        db3_files = list(resolved_bag_path.glob('*.db3'))
        metadata_file = resolved_bag_path / 'metadata.yaml'

        if mcap_files:
            storage_id = 'mcap'
            bag_uri = str(mcap_files[0].resolve())
        elif db3_files:
            storage_id = 'sqlite3'
            bag_uri = str(resolved_bag_path)
        elif metadata_file.exists():
            storage_id = 'sqlite3'
            bag_uri = str(resolved_bag_path)
        else:
            print(f"Error: Could not determine storage type for {resolved_bag_path}")
            return None, None, None

    elif resolved_bag_path.is_file():
        if resolved_bag_path.suffix == '.mcap':
            storage_id = 'mcap'
            bag_uri = str(resolved_bag_path)
        elif resolved_bag_path.suffix == '.db3':
            storage_id = 'sqlite3'
            bag_uri = str(resolved_bag_path.parent)
        else:
            print(f"Error: Unrecognized file type: {resolved_bag_path.suffix}")
            return None, None, None

    if storage_id is None:
        print("Error: Could not determine storage ID")
        return None, None, None

    storage_options = rosbag2_py.StorageOptions(uri=bag_uri, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions()

    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag file: {e}")
        return None, None, None

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # Data storage
    timestamps = []
    positions = {"x": [], "y": [], "z": []}
    timestamps_obs = []
    dist_next_gate = []

    first_timestamp = None

    print(f"Reading trajectory data for {namespace}...")

    while reader.has_next():
        try:
            (topic, data, timestamp_ns) = reader.read_next()
            timestamp_sec = timestamp_ns * 1e-9

            if first_timestamp is None:
                first_timestamp = timestamp_sec

            rel_time = timestamp_sec - first_timestamp
            if rel_time < t0 or rel_time > tf:
                continue

            normalized_topic = topic.lstrip('/')
            odom_topic = f"{namespace}/odom".lstrip('/')
            obs_topic = f"{namespace}/observations".lstrip('/')

            if normalized_topic == odom_topic:
                msg_type = type_map[topic]
                msg_class = get_message(msg_type)
                message = deserialize_message(data, msg_class)

                timestamps.append(rel_time)
                positions["x"].append(message.pose.pose.position.x)
                positions["y"].append(message.pose.pose.position.y)
                positions["z"].append(message.pose.pose.position.z)

            elif normalized_topic == obs_topic:
                msg_type = type_map[topic]
                msg_class = get_message(msg_type)
                message = deserialize_message(data, msg_class)

                timestamps_obs.append(rel_time)
                corners = getattr(message, 'corners_pos_b_curr', float('nan')).reshape(4, 3)
                mean_point = corners.mean(axis=0)
                dist_next_gate.append(np.linalg.norm(mean_point))

        except StopIteration:
            break
        except Exception as e:
            continue

    # Detect gate passing times
    times_pass_gates = []
    if timestamps_obs and dist_next_gate:
        for i in range(1, len(dist_next_gate)):
            if abs(dist_next_gate[i-1] - dist_next_gate[i]) > 0.3:
                times_pass_gates.append(timestamps_obs[i])

    return positions, timestamps, times_pass_gates

def plot_multiple_trajectories(bag_path, drone_configs, save_path=None):
    """Plot trajectories for multiple drones on the same 3D plot.

    Args:
        bag_path: Path to the bag file
        drone_configs: List of tuples (namespace, t0, tf) for each drone
        save_path: Optional path to save the plots
    """

    # Load waypoints/gates from config
    yaml_file = '/home/neo/workspace/src/jirl_bringup/config/config.yaml'
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    waypoints = np.array(data["/*/controller"]["ros__parameters"]["policy"]["waypoints"]).reshape(-1, 6)

    print(f"Loaded {len(waypoints)} waypoints/gates")

    # Create gate geometry
    d = 0.5
    local_square = np.array([
        [0,  d,  d],
        [0, -d,  d],
        [0, -d, -d],
        [0,  d, -d]
    ])

    wp_pos = waypoints[:, :3]
    wp_euler = waypoints[:, 3:]
    rotations = R.from_euler('xyz', wp_euler).as_matrix()
    verts_all = np.einsum('ij,njk->nik', local_square, rotations) + wp_pos[:, np.newaxis, :]

    # Create 3D plot
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')

    # Define colors for different drones
    colors = ['darkred', 'darkblue', 'green', 'purple', 'orange', 'brown', 'pink', 'gray']

    # Process and plot each drone's trajectory
    all_positions = []
    namespaces = []
    for idx, (namespace, t0, tf) in enumerate(drone_configs):
        namespaces.append(namespace)
        color = colors[idx % len(colors)]
        print(f"\nProcessing {namespace} with time range [{t0}, {tf}]")
        positions, timestamps, times_pass_gates = process_drone_trajectory(bag_path, namespace, t0, tf)

        if positions and positions["x"]:
            # Label drones for legend
            if idx == 0:
                legend_label = "ego"
            elif idx == 1:
                legend_label = "adversary"
            else:
                legend_label = f"drone_{idx+1}"

            # Plot trajectory with IEEE-appropriate line width
            ax.plot(positions["x"], positions["y"], positions["z"],
                   label=legend_label, color=color, linewidth=1.5, alpha=0.9)

            # Add directional arrows along the trajectory using quiver
            arrow_spacing = max(1, len(positions["x"]) // 12)  # Show ~12 arrows
            arrow_indices = list(range(arrow_spacing, len(positions["x"]) - 1, arrow_spacing))

            if False: #arrow_indices:
                arrow_x = []
                arrow_y = []
                arrow_z = []
                arrow_dx = []
                arrow_dy = []
                arrow_dz = []

                for i in arrow_indices:
                    # Get direction vector
                    if i > 0:
                        dx = positions["x"][i+1] - positions["x"][i-1]
                        dy = positions["y"][i+1] - positions["y"][i-1]
                        dz = positions["z"][i+1] - positions["z"][i-1]
                    else:
                        dx = positions["x"][i+1] - positions["x"][i]
                        dy = positions["y"][i+1] - positions["y"][i]
                        dz = positions["z"][i+1] - positions["z"][i]

                    # Normalize and scale
                    length = np.sqrt(dx**2 + dy**2 + dz**2)
                    if length > 0:
                        scale = 0.15  # Larger arrow length for better visibility
                        arrow_x.append(positions["x"][i])
                        arrow_y.append(positions["y"][i])
                        arrow_z.append(positions["z"][i])
                        arrow_dx.append(dx / length * scale)
                        arrow_dy.append(dy / length * scale)
                        arrow_dz.append(dz / length * scale)

                # Plot all arrows at once with larger head ratio
                if arrow_x:
                    ax.quiver(arrow_x, arrow_y, arrow_z,
                             arrow_dx, arrow_dy, arrow_dz,
                             color=color, alpha=0.9, arrow_length_ratio=0.4,
                             linewidth=2)

            # # Plot starting point
            # ax.scatter(positions["x"][0], positions["y"][0], positions["z"][0],
            #           color=color, s=100, marker='o', edgecolors='black', linewidth=2,
            #           label=f"{namespace} start")

            # Plot ending point with IEEE-appropriate marker size
            # Only show "crash" label for the second drone
            if idx == 1:
                end_label = "crash"
            else:
                end_label = None
            ax.scatter(positions["x"][-1], positions["y"][-1], positions["z"][-1],
                      color=color, s=60, marker='X', edgecolors='black', linewidth=1,
                      label=end_label)

            # Plot gate intersection points
            if timestamps and times_pass_gates:
                t_array = np.array(timestamps)
                x_array = np.array(positions["x"])
                y_array = np.array(positions["y"])
                z_array = np.array(positions["z"])

                for i, t_gate in enumerate(times_pass_gates):
                    idx_gate = np.argmin(np.abs(t_array - t_gate))
                    # Add label only for the first gate intersection to avoid duplicate legend entries
                    label = "successful gate pass" if idx == 0 and i == 0 else None
                    ax.scatter(x_array[idx_gate], y_array[idx_gate], z_array[idx_gate],
                              color='green', s=15, marker='o', edgecolors='darkgreen', linewidth=0.5,
                              label=label)

            all_positions.append(positions)
            print(f"✓ Loaded trajectory for {namespace}: {len(positions['x'])} points")
        else:
            print(f"✗ No trajectory data found for {namespace}")

    # Plot gates with borders (transparent faces, only edges visible)
    # Create extended gates for visual border (5cm = 0.05m larger)
    border_extension = 0.05
    d_extended = 0.5 + border_extension  # Original d is 0.5, extend by 5cm
    local_square_extended = np.array([
        [0,  d_extended,  d_extended],
        [0, -d_extended,  d_extended],
        [0, -d_extended, -d_extended],
        [0,  d_extended, -d_extended]
    ])
    verts_extended = np.einsum('ij,njk->nik', local_square_extended, rotations) + wp_pos[:, np.newaxis, :]

    for idx, (square, square_extended) in enumerate(zip(verts_all, verts_extended)):
        # Plot the extended border gate first (behind) - IEEE publication style
        poly_border = Poly3DCollection([square_extended],
                                      facecolors=(1, 1, 1, 0),  # RGBA with alpha=0
                                      edgecolors='gray',
                                      linewidths=0.8)
        ax.add_collection3d(poly_border)

        # Plot the actual gate - transparent face with visible black edge
        poly = Poly3DCollection([square],
                               facecolors=(1, 1, 1, 0),  # RGBA with alpha=0
                               edgecolors='black',
                               linewidths=1.0)
        ax.add_collection3d(poly)

    # Set labels and title (IEEE formatting) with adjusted label padding
    ax.set_xlabel("X", fontsize=10, fontweight='normal', labelpad=0)
    ax.set_ylabel("Y", fontsize=10, fontweight='normal', labelpad=0)
    ax.set_zlabel("Z", fontsize=10, fontweight='normal', labelpad=0)
    # ax.set_title("SM vs SM", fontsize=11, fontweight='normal', pad=5)

    # Remove tick labels (measurements) from all axes
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_zticklabels([])

    # Add legend with IEEE formatting - horizontal layout
    ax.legend(loc='upper center', fontsize=9, framealpha=0.9, edgecolor='black',
              ncol=3, columnspacing=1.0, handletextpad=0.5, bbox_to_anchor=(0.5, 1.15))

    # Set equal aspect ratio
    set_axes_equal(ax)

    # Remove grid
    ax.grid(False)

    # Focus camera on the last waypoint
    if len(waypoints) > 0:
        last_wp = waypoints[-1]
        last_wp_pos = last_wp[:3]

        # Set view to focus on the last waypoint
        # view_distance controls how zoomed in we are on the last waypoint
        view_distance = 0.7  # Distance from the waypoint in meters

        # Set the camera position
        # Azimuth and elevation for a nice 3D view
        ax.view_init(elev=40, azim=-48)

        # Set limits centered around the last waypoint using view_distance
        ax.set_xlim(last_wp_pos[0] - view_distance, last_wp_pos[0] + view_distance)
        ax.set_ylim(last_wp_pos[1] - view_distance, last_wp_pos[1] + view_distance)
        ax.set_zlim(last_wp_pos[2] - view_distance, last_wp_pos[2] + view_distance)

    # Save if requested
    if save_path:
        output_dir = Path(save_path)
        output_dir.mkdir(parents=True, exist_ok=True)

        # Generate filename based on drone names
        filename = "_".join(namespaces) + "_trajectories"

        # Save in IEEE-recommended formats with high quality
        # EPS for IEEE submissions (vector format)
        filepath_eps = output_dir / (filename + '.eps')
        fig.savefig(filepath_eps, format='eps', bbox_inches='tight', pad_inches=0.05)
        print(f"Saved EPS (IEEE submission) to {filepath_eps}")

        # PDF for review and vector graphics
        filepath_pdf = output_dir / (filename + '.pdf')
        fig.savefig(filepath_pdf, format='pdf', bbox_inches='tight', pad_inches=0.05, dpi=600)
        print(f"Saved PDF (vector) to {filepath_pdf}")

        # High-res PNG for preview
        filepath_png = output_dir / (filename + '.png')
        fig.savefig(filepath_png, format='png', bbox_inches='tight', pad_inches=0.05, dpi=600)
        print(f"Saved PNG (preview) to {filepath_png}")

    # Show plot
    plt.show()
    plt.close('all')

def main():
    if len(sys.argv) < 3:
        print("\nUsage: python process_bag_trajectory.py <bag_path> <drone_spec1> [drone_spec2] ...")
        print("\nDrone specifications can be:")
        print("  <namespace>           - Use default time range [0, inf]")
        print("  <namespace>:<t0>:<tf> - Use specific time range for this drone")
        print("\nExamples:")
        print("  Single drone:    python process_bag_trajectory.py logs/test1 crazy_jirl_b3")
        print("  Multiple drones: python process_bag_trajectory.py logs/test1 crazy_jirl_b3 crazy_jirl_b4")
        print("  Different times: python process_bag_trajectory.py logs/test1 crazy_jirl_b3:17:22 crazy_jirl_b4:18:23")
        print("  Mixed:          python process_bag_trajectory.py logs/test1 crazy_jirl_b3 crazy_jirl_b4:10:20")
        sys.exit(1)

    bag_path = sys.argv[1]
    drone_specs = sys.argv[2:]

    # Parse drone specifications
    drone_configs = []
    for spec in drone_specs:
        parts = spec.split(':')
        if len(parts) == 1:
            # Just namespace, use default time range
            namespace = parts[0]
            t0 = 0
            tf = float('inf')
        elif len(parts) == 3:
            # namespace:t0:tf format
            namespace = parts[0]
            try:
                t0 = float(parts[1])
                tf = float(parts[2])
            except ValueError:
                print(f"Error: Invalid time values in specification '{spec}'")
                print("Time values must be numbers")
                sys.exit(1)
        else:
            print(f"Error: Invalid drone specification '{spec}'")
            print("Expected format: <namespace> or <namespace>:<t0>:<tf>")
            sys.exit(1)

        drone_configs.append((namespace, t0, tf))

    if not drone_configs:
        print("Error: At least one drone must be specified")
        sys.exit(1)

    print(f"\nProcessing bag: {bag_path}")
    print("Drone configurations:")
    for namespace, t0, tf in drone_configs:
        tf_str = "inf" if tf == float('inf') else str(tf)
        print(f"  {namespace}: [{t0}, {tf_str}]")

    # Determine save path
    bag_path_obj = Path(bag_path).resolve()
    if bag_path_obj.is_file():
        experiment_dir = bag_path_obj.parent
    else:
        experiment_dir = bag_path_obj

    logs_base_dir = experiment_dir.parent
    save_path = logs_base_dir / 'plots' / experiment_dir.name

    # Plot trajectories
    plot_multiple_trajectories(bag_path, drone_configs, save_path)

if __name__ == "__main__":
    main()