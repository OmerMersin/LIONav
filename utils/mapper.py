#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import math

# === Load your CSV ===
csv_path = "/home/orin/rtl_traj_latest.csv"  # <- change to your actual path
df = pd.read_csv(csv_path)

# === Compute yaw from quaternion ===
def yaw_from_quat(qx, qy, qz, qw):
    # Standard ENU yaw extraction
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

df["yaw_rad"] = [yaw_from_quat(qx, qy, qz, qw)
                 for qx, qy, qz, qw in zip(df.qx, df.qy, df.qz, df.qw)]
df["yaw_deg"] = df["yaw_rad"] * 180.0 / math.pi

# === Normalize to start at (0,0) ===
x0, y0 = df.x.iloc[0], df.y.iloc[0]
df["x_rel"] = df.x - x0
df["y_rel"] = df.y - y0

# === Plot bird’s eye view ===
plt.figure(figsize=(8, 8))
plt.plot(df.x_rel, df.y_rel, color='purple', linewidth=2, label="Trajectory")
plt.scatter(df.x_rel.iloc[0], df.y_rel.iloc[0], color='green', s=60, label="Start")
plt.scatter(df.x_rel.iloc[-1], df.y_rel.iloc[-1], color='red', s=60, label="End")

# Optionally add small orientation arrows
step = max(1, len(df)//50)
for i in range(0, len(df), step):
    yaw = df.yaw_rad.iloc[i]
    x, y = df.x_rel.iloc[i], df.y_rel.iloc[i]
    dx, dy = 0.1 * math.cos(yaw), 0.1 * math.sin(yaw)
    plt.arrow(x, y, dx, dy, color='blue', head_width=0.05, alpha=0.5)

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("LIO-SAM Odometry Bird’s-Eye View")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.show()
