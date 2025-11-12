#!/usr/bin/env python3
import rosbag2_py, csv
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu

bag_path = "/home/orin/imu_static"
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions('', '')
reader.open(storage_options, converter_options)
topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

out = open("imu_data.csv", "w", newline="")
writer = csv.writer(out)
writer.writerow(["time","ax","ay","az","gx","gy","gz"])

while reader.has_next():
    topic, data, _ = reader.read_next()
    if "imu/data" in topic:
        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        writer.writerow([t,
                         msg.linear_acceleration.x,
                         msg.linear_acceleration.y,
                         msg.linear_acceleration.z,
                         msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])
out.close()
print("✅ Exported to imu_data.csv")
