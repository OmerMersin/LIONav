#!/usr/bin/env python3
"""
ArduPilot REVERSE Trajectory Follower
Assumes drone is at the END of trajectory and traces back to START
- Reverses the waypoint order
- Reverses time sequence
- Handles ENU→NED conversion
- Reverses yaw angles appropriately
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

CURRENT_ALT_M = float(os.getenv("CURRENT_ALT", "5"))     # Current flight altitude
SEND_HZ_FALLBACK = float(os.getenv("SEND_HZ", "10"))    # Fallback if no timestamps
TIME_SCALE = float(os.getenv("TIME_SCALE", "1.0"))      # 1.0 = real-time
XY_SCALE = float(os.getenv("XY_SCALE", "1.0"))          # Scale XY positions
ALIGN_WITH_LAST_YAW = os.getenv("ALIGN_YAW", "1") == "1"  # Align with LAST yaw (end position)
REVERSE_YAW = os.getenv("REVERSE_YAW", "1") == "1"      # Flip yaw 180° for backward facing

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
        sys.exit(1)
    
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


def reverse_and_normalize_trajectory(times, xs, ys, zs, yaws_deg):
    """
    Reverse trajectory order and normalize to start at origin.
    Now the END becomes the START (origin), and we trace back to the original start.
    """
    # REVERSE all arrays
    times = times[::-1]
    xs = xs[::-1]
    ys = ys[::-1]
    zs = zs[::-1]
    yaws_deg = yaws_deg[::-1]
    
    print("🔄 Trajectory REVERSED - End is now Start")
    
    # Recalculate times from 0 - need to rebuild time sequence
    # After reversing, times are decreasing, so we need to flip them
    t_max = times[0]  # This was the last time in original
    t_min = times[-1]  # This was the first time in original
    duration = t_max - t_min
    times = [duration - (t - t_min) for t in times]
    
    # Shift origin to first pose (which was the LAST pose in original trajectory)
    x0, y0, z0 = xs[0], ys[0], zs[0]
    xs_norm = [x - x0 for x in xs]
    ys_norm = [y - y0 for y in ys]
    zs_norm = [z - z0 for z in zs]
    
    # Align heading so first yaw (end of original trajectory) is 0 (optional)
    if ALIGN_WITH_LAST_YAW:
        yaw0_rad = to_radians(yaws_deg[0])
        rotated = [rotate_2d(x, y, -yaw0_rad) for x, y in zip(xs_norm, ys_norm)]
        xs_norm = [r[0] for r in rotated]
        ys_norm = [r[1] for r in rotated]
        yaws_aligned = [yaw - yaws_deg[0] for yaw in yaws_deg]
    else:
        yaws_aligned = yaws_deg[:]
    
    # Optionally reverse yaw by 180° for backward-facing flight
    if REVERSE_YAW:
        yaws_aligned = [yaw + 180.0 for yaw in yaws_aligned]
        print("↩️  Yaw reversed by 180° for backward-facing flight")
    
    # Print trajectory stats
    dx = xs_norm[-1] - xs_norm[0]
    dy = ys_norm[-1] - ys_norm[0]
    dz = zs_norm[-1] - zs_norm[0]
    duration = times[-1] - times[0]
    
    print(f"ℹ️  Reverse trajectory stats (ENU, aligned):")
    print(f"   Start (was END): ({xs[0]:.2f}, {ys[0]:.2f}, {zs[0]:.2f})")
    print(f"   End (was START): ({xs[-1]:.2f}, {ys[-1]:.2f}, {zs[-1]:.2f})")
    print(f"   Δx={dx:.2f}m, Δy={dy:.2f}m, Δz={dz:.2f}m")
    print(f"   Duration: {duration:.1f}s, Points: {len(times)}")
    
    return times, xs_norm, ys_norm, zs_norm, yaws_aligned


def set_guided_mode(conn):
    """Set GUIDED mode and optionally takeoff if not already flying"""
    print("🟢 Setting GUIDED mode...")
    conn.set_mode('GUIDED')
    time.sleep(1)
    
    # Check if already armed
    msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    armed = False
    if msg:
        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if armed:
            print("✅ Already armed")
        else:
            print("⚠️  Warning: Motors not armed! Will arm and takeoff...")
    
    # Check current altitude
    current_alt = 0.0
    msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg:
        current_alt = msg.relative_alt / 1000.0
        print(f"📍 Current altitude: {current_alt:.1f}m")
    
    # If not armed or low altitude, do takeoff
    if not armed or current_alt < 2.0:
        print(f"🛫 Taking off to {CURRENT_ALT_M}m...")
        if not armed:
            conn.arducopter_arm()
            conn.motors_armed_wait()
            print("✅ Armed!")
        
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0, 0, 0, 0,  # params 1-4
            0, 0,  # lat, lon (use current)
            CURRENT_ALT_M  # altitude
        )
        
        print("⏳ Waiting for takeoff to complete...")
        time.sleep(8)
        
        # Verify altitude
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            alt = msg.relative_alt / 1000.0
            print(f"📍 Reached altitude: {alt:.1f}m")
    else:
        print("✅ Already flying at altitude")


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
    print("🔙 Starting REVERSE trajectory following...")
    print("   (Going from END back to START)")
    
    # Calculate time deltas
    dt_list = []
    for i in range(len(times) - 1):
        dt = max(times[i + 1] - times[i], 0.001)
        dt_list.append(dt)
    
    # Debug: Check time deltas
    avg_dt = sum(dt_list) / len(dt_list) if dt_list else 0
    print(f"ℹ️  Average time delta: {avg_dt:.3f}s, Total duration: {sum(dt_list):.1f}s")
    
    # If timestamps are missing or invalid, use fallback rate
    if sum(dt_list) <= 0.0 or not dt_list:
        dt_fallback = 1.0 / SEND_HZ_FALLBACK
        dt_list = [dt_fallback] * (len(times) - 1)
        print(f"⚠️  Using fallback rate: {SEND_HZ_FALLBACK} Hz")
    
    start_time = time.time()
    
    for i in range(len(times)):
        # Apply XY scaling
        x_enu = xs[i] * XY_SCALE
        y_enu = ys[i] * XY_SCALE
        z_enu = zs[i]
        
        # Convert ENU to NED
        x_ned = y_enu
        y_ned = x_enu
        z_ned = -(z_enu + CURRENT_ALT_M)
        
        # Convert yaw
        yaw_enu_rad = to_radians(yaws_deg[i])
        yaw_ned_rad = enu_to_ned_yaw(yaw_enu_rad)
        
        # Time since start in milliseconds
        time_boot_ms = int((time.time() - start_time) * 1000)
        
        # Send position target
        send_ned_position_target(conn, x_ned, y_ned, z_ned, yaw_ned_rad, time_boot_ms)
        
        # Progress feedback
        if i % 5 == 0 or i == len(times) - 1:
            progress = (i + 1) / len(times) * 100
            print(f"[{i+1:03d}/{len(times)} {progress:5.1f}%] NED: ({x_ned:+.2f}, {y_ned:+.2f}, {z_ned:+.2f})m, "
                  f"yaw: {math.degrees(yaw_ned_rad):+.1f}°")
        
        # Sleep for next waypoint (time-scaled)
        if i < len(times) - 1:
            sleep_time = dt_list[i] / TIME_SCALE
            time.sleep(max(sleep_time, 0.001))
    
    print("✅ Reverse trajectory complete! Back at START position")


def main():
    """Main execution"""
    print("=" * 60)
    print("🔙 ArduPilot REVERSE Trajectory Follower")
    print("=" * 60)
    print("ℹ️  Assumes drone is at END position")
    print("ℹ️  Will trace back to START position")
    print("=" * 60)
    
    # Connect
    conn = connect_mavlink(TCP_TARGET)
    
    # Load trajectory
    times, xs, ys, zs, yaws = load_trajectory(TRAJ_FILE)
    
    # REVERSE and normalize
    times, xs, ys, zs, yaws = reverse_and_normalize_trajectory(times, xs, ys, zs, yaws)
    
    # Execute mission
    try:
        set_guided_mode(conn)
        print("⏳ Stabilizing position...")
        time.sleep(3)  # Give time to stabilize at altitude
        follow_trajectory(conn, times, xs, ys, zs, yaws)
        print("⏳ Holding final position for 2 seconds...")
        time.sleep(2)
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
    print("✅ Reverse mission complete!")
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

