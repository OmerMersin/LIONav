#!/usr/bin/env python3
"""
ArduPilot Trajectory Follower - Replay recorded ENU trajectory in NED
Handles: origin normalization, heading alignment, ENU→NED conversion, timing
CSV format: time(s), x, y, z, yaw_deg
"""

from pymavlink import mavutil
import csv
import math
import os
import time
import sys

# ============== CONFIGURATION ==============
TCP_TARGET = os.getenv("MP_TCP", "192.168.1.196:5762")  # Default to localhost SITL
TRAJ_FILE = os.getenv("TRAJ_FILE", "/home/orin/rtl_traj_20251112_120633.csv")

TAKEOFF_ALT_M = float(os.getenv("TAKEOFF_ALT", "5"))      # AGL altitude offset
SEND_HZ_FALLBACK = float(os.getenv("SEND_HZ", "10"))     # Fallback if no timestamps
TIME_SCALE = float(os.getenv("TIME_SCALE", "1.0"))       # 1.0 = real-time
XY_SCALE = float(os.getenv("XY_SCALE", "1.0"))           # Scale XY positions
ALIGN_WITH_FIRST_YAW = os.getenv("ALIGN_YAW", "1") == "1"

# Bitmask flags for SET_POSITION_TARGET_LOCAL_NED
MASK_IGNORE_VX = 1 << 3
MASK_IGNORE_VY = 1 << 4
MASK_IGNORE_VZ = 1 << 5
MASK_IGNORE_AX = 1 << 6
MASK_IGNORE_AY = 1 << 7
MASK_IGNORE_AZ = 1 << 8
MASK_IGNORE_YAW_RATE = 1 << 11


def connect_mavlink(target):
    """Connect to ArduPilot via TCP"""
    print(f"🔌 Connecting to {target}...")
    try:
        conn = mavutil.mavlink_connection(f"tcp:{target}", timeout=10)
        print("⏳ Waiting for heartbeat...")
        conn.wait_heartbeat(timeout=15)
        print(f"✅ Connected! System ID: {conn.target_system}, Component: {conn.target_component}")
        return conn
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        sys.exit(1)


def to_radians(degrees):
    """Convert degrees to radians"""
    return degrees * math.pi / 180.0


def wrap_to_pi(angle_rad):
    """Wrap angle to [-π, π]"""
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi


def enu_to_ned_yaw(yaw_enu_rad):
    """Convert ENU yaw (0=East, CCW+) to NED yaw (0=North, CW+)"""
    return wrap_to_pi(math.pi / 2.0 - yaw_enu_rad)


def rotate_2d(x, y, angle_rad):
    """Rotate point (x,y) by angle_rad counterclockwise"""
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return c * x - s * y, s * x + c * y


def load_trajectory(filepath):
    """Load trajectory from CSV file"""
    if not os.path.exists(filepath):
        print(f"❌ File not found: {filepath}")
        print(f"ℹ️  Creating sample trajectory file...")
        create_sample_trajectory(filepath)
    
    times, xs, ys, zs, yaws = [], [], [], [], []
    
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        
        for row_num, row in enumerate(reader, start=2):
            if len(row) < 4:
                continue
            try:
                t = float(row[0])
                x = float(row[1])
                y = float(row[2])
                z = float(row[3])
                yaw = float(row[4]) if len(row) > 4 and row[4].strip() else 0.0
                
                times.append(t)
                xs.append(x)
                ys.append(y)
                zs.append(z)
                yaws.append(yaw)
            except (ValueError, IndexError) as e:
                print(f"⚠️  Skipping invalid row {row_num}: {row}")
                continue
    
    if not times:
        print("❌ No valid trajectory data found")
        sys.exit(1)
    
    print(f"✅ Loaded {len(times)} waypoints from {filepath}")
    return times, xs, ys, zs, yaws


def create_sample_trajectory(filepath):
    """Create a sample square trajectory for testing"""
    print("📝 Creating sample square trajectory...")
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'x', 'y', 'z', 'yaw_deg'])
        
        # Square pattern: 10m x 10m, 40 points
        for i in range(41):
            t = i * 0.5
            if i <= 10:  # Move forward (North in ENU)
                x, y = i, 0
                yaw = 90
            elif i <= 20:  # Move right (East in ENU)
                x, y = 10, i - 10
                yaw = 0
            elif i <= 30:  # Move back (South in ENU)
                x, y = 30 - i, 10
                yaw = -90
            else:  # Move left (West in ENU)
                x, y = 0, 40 - i
                yaw = 180
            
            z = 0.0  # Constant altitude
            writer.writerow([t, x, y, z, yaw])
    
    print(f"✅ Sample trajectory created: {filepath}")


def normalize_trajectory(times, xs, ys, zs, yaws_deg):
    """Normalize trajectory to start at origin with optional heading alignment"""
    # Shift to origin
    x0, y0, z0 = xs[0], ys[0], zs[0]
    xs_norm = [x - x0 for x in xs]
    ys_norm = [y - y0 for y in ys]
    zs_norm = [z - z0 for z in zs]
    
    # Align heading
    if ALIGN_WITH_FIRST_YAW:
        yaw0_rad = to_radians(yaws_deg[0])
        rotated = [rotate_2d(x, y, -yaw0_rad) for x, y in zip(xs_norm, ys_norm)]
        xs_norm = [r[0] for r in rotated]
        ys_norm = [r[1] for r in rotated]
        yaws_aligned = [yaw - yaws_deg[0] for yaw in yaws_deg]
    else:
        yaws_aligned = yaws_deg[:]
    
    # Print trajectory stats
    dx = xs_norm[-1] - xs_norm[0]
    dy = ys_norm[-1] - ys_norm[0]
    dz = zs_norm[-1] - zs_norm[0]
    duration = times[-1] - times[0]
    
    print(f"ℹ️  Trajectory stats (ENU, aligned):")
    print(f"   Δx={dx:.2f}m, Δy={dy:.2f}m, Δz={dz:.2f}m")
    print(f"   Duration: {duration:.1f}s, Points: {len(times)}")
    
    return times, xs_norm, ys_norm, zs_norm, yaws_aligned


def wait_for_mode(conn, mode_name, timeout=5):
    """Wait for mode change confirmation"""
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            current_mode = conn.mode_mapping().get(msg.custom_mode, 'UNKNOWN')
            if current_mode == mode_name:
                return True
    return False


def arm_and_takeoff(conn, altitude_m):
    """Arm and takeoff to specified altitude"""
    print("🟢 Setting GUIDED mode...")
    conn.set_mode('GUIDED')
    time.sleep(1)
    
    print("🔓 Arming motors...")
    conn.arducopter_arm()
    conn.motors_armed_wait()
    print("✅ Armed!")
    
    print(f"🛫 Taking off to {altitude_m}m...")
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0, 0, 0, 0,  # params 1-4
        0, 0,  # lat, lon (use current)
        altitude_m  # altitude
    )
    
    # Wait for takeoff
    print("⏳ Waiting for takeoff...")
    time.sleep(8)
    
    # Check altitude
    msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg:
        alt = msg.relative_alt / 1000.0
        print(f"📍 Current altitude: {alt:.1f}m")


def land_and_disarm(conn):
    """Land and disarm"""
    print("🛬 Landing...")
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0,
        0, 0, 0
    )
    time.sleep(5)
    
    print("🔒 Disarming...")
    conn.arducopter_disarm()
    print("✅ Disarmed")


def send_ned_position_target(conn, x, y, z, yaw_rad, time_boot_ms):
    """Send position target in NED frame"""
    # Build type_mask: position + yaw only, ignore velocities and accelerations
    type_mask = (MASK_IGNORE_VX | MASK_IGNORE_VY | MASK_IGNORE_VZ |
                 MASK_IGNORE_AX | MASK_IGNORE_AY | MASK_IGNORE_AZ |
                 MASK_IGNORE_YAW_RATE)
    
    conn.mav.set_position_target_local_ned_send(
        time_boot_ms,
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, z,  # position in meters
        0, 0, 0,  # velocity (ignored)
        0, 0, 0,  # acceleration (ignored)
        yaw_rad,  # yaw in radians
        0.0       # yaw_rate (ignored)
    )


def follow_trajectory(conn, times, xs, ys, zs, yaws_deg):
    """Execute trajectory following"""
    print("🚁 Starting trajectory following...")
    
    # Calculate time deltas
    dt_list = []
    for i in range(len(times) - 1):
        dt = max(times[i + 1] - times[i], 0.001)
        dt_list.append(dt)
    
    # If timestamps are missing or invalid, use fallback rate
    if sum(dt_list) <= 0.0 or not dt_list:
        dt_fallback = 1.0 / SEND_HZ_FALLBACK
        dt_list = [dt_fallback] * (len(times) - 1)
        print(f"⚠️  Using fallback rate: {SEND_HZ_FALLBACK} Hz")
    
    # Build bitmask
    type_mask = (MASK_IGNORE_VX | MASK_IGNORE_VY | MASK_IGNORE_VZ |
                 MASK_IGNORE_AX | MASK_IGNORE_AY | MASK_IGNORE_AZ |
                 MASK_IGNORE_YAW_RATE)
    
    start_time = time.time()
    
    for i in range(len(times)):
        # Apply XY scaling
        x_enu = xs[i] * XY_SCALE
        y_enu = ys[i] * XY_SCALE
        z_enu = zs[i]
        
        # Convert ENU to NED
        x_ned = y_enu
        y_ned = x_enu
        z_ned = -(z_enu + TAKEOFF_ALT_M)
        
        # Convert yaw
        yaw_enu_rad = to_radians(yaws_deg[i])
        yaw_ned_rad = enu_to_ned_yaw(yaw_enu_rad)
        
        # Time since start in milliseconds
        time_boot_ms = int((time.time() - start_time) * 1000)
        
        # Send position target
        send_ned_position_target(conn, x_ned, y_ned, z_ned, yaw_ned_rad, time_boot_ms)
        
        # Progress feedback
        if i % 5 == 0 or i == len(times) - 1:
            print(f"[{i+1:03d}/{len(times)}] NED: ({x_ned:+.2f}, {y_ned:+.2f}, {z_ned:+.2f})m, "
                  f"yaw: {math.degrees(yaw_ned_rad):+.1f}°")
        
        # Sleep for next waypoint (time-scaled)
        if i < len(times) - 1:
            sleep_time = dt_list[i] / TIME_SCALE
            time.sleep(max(sleep_time, 0.001))
    
    print("✅ Trajectory complete!")


def main():
    """Main execution"""
    print("=" * 60)
    print("🚁 ArduPilot Trajectory Follower")
    print("=" * 60)
    
    # Connect
    conn = connect_mavlink(TCP_TARGET)
    
    # Load trajectory
    times, xs, ys, zs, yaws = load_trajectory(TRAJ_FILE)
    
    # Normalize
    times, xs, ys, zs, yaws = normalize_trajectory(times, xs, ys, zs, yaws)
    
    # Execute mission
    try:
        arm_and_takeoff(conn, TAKEOFF_ALT_M)
        follow_trajectory(conn, times, xs, ys, zs, yaws)
        land_and_disarm(conn)
    except Exception as e:
        print(f"❌ Error during flight: {e}")
        print("🛑 Attempting emergency landing...")
        try:
            land_and_disarm(conn)
        except:
            pass
        raise
    
    print("=" * 60)
    print("✅ Mission complete!")
    print("=" * 60)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⏹️  Interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
