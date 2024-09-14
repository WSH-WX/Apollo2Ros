import sys
import rosbag2_py
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def parse_ros2_bag_for_cmd_vel(bag_file_path, save_pic_name):
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader.open(storage_options, converter_options)

    linear_x_values = []
    angular_z_values = []
    timestamps_control = []
    i = 0
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if i == 0:
            start_timestamp = timestamp
            i += 1
        timestamp_control = (timestamp-start_timestamp) / 1e9
        # print(f"Time: {timestamp_control} s")
        if topic_name == "/cmd_vel":
            msg_type = get_message("geometry_msgs/Twist")
            twist_msg = deserialize_message(data, msg_type)

            # 获取线速度和角速度
            linear_x = twist_msg.linear.x
            angular_z = twist_msg.angular.z

            # 打印值以进行调试
            # print(f"Linear X: {linear_x}, Angular Z: {angular_z}")

            # 将解析的数据存储到列表中
            linear_x_values.append(linear_x)
            angular_z_values.append(angular_z)
            timestamps_control.append(timestamp_control)
            

    # 绘制线速度和角速度
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    ax1.plot(linear_x_values, linestyle='-', color='blue', linewidth=2)
    ax1.set_xlabel('Index')
    ax1.set_ylabel('Linear Velocity X (m/s)')
    ax1.set_title('Linear Velocity over Time')

    ax2.plot(angular_z_values, linestyle='-', color='green', linewidth=2)
    ax2.set_xlabel('Index')
    ax2.set_ylabel('Angular Velocity Z (rad/s)')
    ax2.set_title('Angular Velocity over Time')

    # 保存图像
    plt.tight_layout()
    plt.savefig("data/PNG/0820_PNG"+save_pic_name+".png")
    print(f"Figure saved to {save_pic_name}")

    print(f"{len(linear_x_values)}")

    return timestamps_control

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 parse_ros2_bag_for_cmd_vel.py <path_to_bag_file> save_image_name")
        sys.exit(1)

    bag_file_path = sys.argv[1]
    save_pic_name = sys.argv[2]

    try:
        timestamps_control = parse_ros2_bag_for_cmd_vel(bag_file_path, save_pic_name)
        print(f"{timestamps_control[0]}")
        print(f"{timestamps_control[1]}")
        print(f"{timestamps_control[2]}")
        print(f"{timestamps_control[3]}")
    except Exception as e:
        print(f"Failed to parse the bag file: {e}")
