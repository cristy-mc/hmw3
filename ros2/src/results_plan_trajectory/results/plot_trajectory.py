#!/usr/bin/env python3
import sys
import math

import numpy as np
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message

from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, TrajectorySetpoint
    

#yaw from quaternion
def quat_to_yaw(w, x, y, z):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = w * w + x * x - y * y - z * z
    return math.atan2(siny_cosp, cosy_cosp)


def read_bag(bag_path):
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    t_pos = []
    x = []; y = []; z = []
    vx = []; vy = []; vz = []
    ax = []; ay = []; az = []

    t_att = []
    yaw = []

    t_ref = []
    xref = []; yref = []; zref = []
    vxref = []; vyref = []; vzref = []
    axref = []; ayref = []; azref = []
    yawref = []

    first_timestamp = None

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if first_timestamp is None:
            first_timestamp = timestamp
        t_sec = (timestamp - first_timestamp) * 1e-9

        # VehicleLocalPosition 
        if topic == "/fmu/out/vehicle_local_position_v1":
            msg = deserialize_message(data, VehicleLocalPosition)
            t_pos.append(t_sec)

            x.append(msg.x)
            y.append(msg.y)
            z.append(msg.z)   # NED

            vx.append(msg.vx)
            vy.append(msg.vy)
            vz.append(msg.vz)

            ax.append(msg.ax)
            ay.append(msg.ay)
            az.append(msg.az)

        # VehicleAttitude 
        elif topic == "/fmu/out/vehicle_attitude":
            msg = deserialize_message(data, VehicleAttitude)
            t_att.append(t_sec)
            yaw_val = quat_to_yaw(msg.q[0], msg.q[1], msg.q[2], msg.q[3])
            yaw.append(yaw_val)

        # TrajectorySetpoint 
        elif topic == "/fmu/in/trajectory_setpoint":
            msg = deserialize_message(data, TrajectorySetpoint)
            t_ref.append(t_sec)

            xref.append(msg.position[0])
            yref.append(msg.position[1])
            zref.append(msg.position[2])  # NED

            vxref.append(msg.velocity[0])
            vyref.append(msg.velocity[1])
            vzref.append(msg.velocity[2])

            axref.append(msg.acceleration[0])
            ayref.append(msg.acceleration[1])
            azref.append(msg.acceleration[2])

            yawref.append(msg.yaw)

    return {
        "t_pos": np.array(t_pos),
        "x": np.array(x), "y": np.array(y), "z": np.array(z),
        "vx": np.array(vx), "vy": np.array(vy), "vz": np.array(vz),
        "ax": np.array(ax), "ay": np.array(ay), "az": np.array(az),

        "t_att": np.array(t_att),
        "yaw": np.array(yaw),

        "t_ref": np.array(t_ref),
        "xref": np.array(xref), "yref": np.array(yref), "zref": np.array(zref),
        "vxref": np.array(vxref), "vyref": np.array(vyref), "vzref": np.array(vzref),
        "axref": np.array(axref), "ayref": np.array(ayref), "azref": np.array(azref),
        "yawref": np.array(yawref)
    }


def make_plots(data):

    # measured
    t_pos = data["t_pos"]
    x = data["x"]; y = data["y"]; z = data["z"]
    vx = data["vx"]; vy = data["vy"]; vz = data["vz"]
    ax = data["ax"]; ay = data["ay"]; az = data["az"]

    # reference
    t_ref = data["t_ref"]
    xref = data["xref"]; yref = data["yref"]; zref = data["zref"]
    vxref = data["vxref"]; vyref = data["vyref"]; vzref = data["vzref"]
    axref = data["axref"]; ayref = data["ayref"]; azref = data["azref"]

    # yaw
    t_att = data["t_att"]
    yaw = data["yaw"]
    yawref = data["yawref"]

    # velocity 
    v  = np.sqrt(vx**2 + vy**2 + vz**2)
    vref = np.sqrt(vxref**2 + vyref**2 + vzref**2)

    # acceleration
    acc = np.sqrt(ax**2 + ay**2 + az**2)
    acc_ref = np.sqrt(axref**2 + ayref**2 + azref**2)

    # NED -> ENU
    z_plot = -z
    zref_plot = -zref

    # PLOT XY 
    plt.figure()
    plt.plot(x, y, label="measured")
    plt.plot(xref, yref, "--", label="reference")
    plt.xlabel("x [m]"); plt.ylabel("y [m]")
    plt.title("XY Trajectory")
    plt.axis("equal"); plt.grid(True); plt.legend()

    # ALTITUDE 
    plt.figure()
    plt.plot(t_pos, z_plot, label="measured z")
    plt.plot(t_ref, zref_plot, "--", label="reference z")
    plt.xlabel("time [s]"); plt.ylabel("z [m] (up)")
    plt.title("Altitude"); plt.grid(True); plt.legend()

    # YAW
    plt.figure()
    plt.plot(t_att, yaw, label="measured yaw")
    plt.plot(t_ref, yawref, "--", label="reference yaw")
    plt.xlabel("time [s]"); plt.ylabel("yaw [rad]")
    plt.title("Yaw"); plt.grid(True); plt.legend()

    # SPEED 
    plt.figure()
    plt.plot(t_pos, v, label="measured speed")
    plt.plot(t_ref, vref, "--", label="reference speed")
    plt.xlabel("time [s]"); plt.ylabel("speed [m/s]")
    plt.title("Linear speed"); plt.grid(True); plt.legend()


    # ACCELERATION 
    plt.figure()
    plt.plot(t_pos, acc, label="measured acceleration")
    plt.plot(t_ref, acc_ref, "--", label="reference acceleration")
    plt.xlabel("time [s]"); plt.ylabel("acc [m/s^2]")
    plt.title("Acceleration"); plt.grid(True); plt.legend()


    plt.show()



def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_trajectory.py /path/to/bag")
        sys.exit(1)

    data = read_bag(sys.argv[1])
    make_plots(data)


if __name__ == "__main__":
    main()
