import argparse
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from leo_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import rosbag2_py

# https://mcap.dev/docs/python/ros2_example
def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp
    del reader


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )
    
    f_gt = open("gt_data.txt", "w")
    f_imu = open("imu_data.txt", "w")

    args = parser.parse_args()
    for topic, msg, timestamp in read_messages(args.input):
        if isinstance(msg, Imu):
            # print(f"{topic} [{timestamp}]: '{msg}'")
            print(topic)
            time = msg.stamp.sec + 1e-9 * msg.stamp.nanosec
            f_imu.write(str(time) + " ")
            f_imu.write(str(msg.gyro_x) + " " + str(msg.gyro_y) + " " + str(msg.gyro_z) + " ")
            f_imu.write(str(msg.accel_x) + " " + str(msg.accel_y) + " " + str(msg.accel_z))
            f_imu.write("\n")
        elif isinstance(msg, PoseStamped):
            # print(f"{topic} [{timestamp}]: '{msg}'")
            print(topic)
            time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            f_gt.write(str(time) + " ")
            f_gt.write(str(msg.pose.position.x) + " " + str(msg.pose.position.y) + " " + str(msg.pose.position.z) + " ")
            f_gt.write(str(msg.pose.orientation.x) + " " + str(msg.pose.orientation.y) + " " + str(msg.pose.orientation.z) + " " + str(msg.pose.orientation.w))
            f_gt.write("\n")
        else:
            # print(f"{topic} [{timestamp}]: ({type(msg).__name__})")
            print("other msg")
    f_gt.close()
    f_imu.close()


if __name__ == "__main__":
    main()
