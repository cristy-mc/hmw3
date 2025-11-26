#!/usr/bin/env python3
import argparse, os
import rosbag2_py, rclpy
from rclpy.serialization import deserialize_message
from px4_msgs.msg import VehicleLocalPosition, ManualControlSetpoint
import matplotlib.pyplot as plt


def read_bag(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )

    t_z, z_vals, t_thr, thr_vals = [], [], [], []
    first_stamp = None

    while reader.has_next():
        topic, data, t = reader.read_next()
        first_stamp = first_stamp or t
        t_sec = (t - first_stamp) * 1e-9

        if topic == '/fmu/out/vehicle_local_position_v1':
            msg = deserialize_message(data, VehicleLocalPosition)
            t_z.append(t_sec); z_vals.append(msg.z)

        elif topic == '/fmu/out/manual_control_setpoint':
            msg = deserialize_message(data, ManualControlSetpoint)
            t_thr.append(t_sec); thr_vals.append(msg.throttle)

    return t_z, z_vals, t_thr, thr_vals


def make_plot(t_z, z_vals, t_thr, thr_vals, out_file):
    plt.figure(figsize=(10, 5))
    plt.plot(t_z, [-z for z in z_vals], label='Altitude z [m]')
    plt.plot(t_thr, thr_vals, label='Manual throttle setpoint')
    plt.xlabel('Time [s]'); plt.ylabel('Value')
    plt.title('Force land test: altitude vs throttle')
    plt.grid(True); plt.legend(); plt.tight_layout()
    plt.savefig(out_file, dpi=200)
    print(f"Plot saved to: {out_file}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bag_dir', default='force_land_test',
                   help="Directory containing the .db3 file")
    p.add_argument('--out', default='force_land_plot.png',
                   help="Output plot filename")
    args = p.parse_args()

    bag_dir = os.path.abspath(args.bag_dir)
    if not os.path.isdir(bag_dir):
        raise SystemExit(f"Bag directory not found: {bag_dir}")

    rclpy.init()
    try: 
        t_z, z_vals, t_thr, thr_vals = read_bag(bag_dir)
    finally:
        rclpy.shutdown()

    if not (t_z and t_thr):
        print("Warning: missing required topics in bag.")
        return

    make_plot(t_z, z_vals, t_thr, thr_vals, args.out)


if __name__ == '__main__':
    main()

