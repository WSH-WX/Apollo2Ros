import sys
import rosbag2_py
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def parse_ros2_bag_for_point(bag_file_path, save_pic_name):
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader.open(storage_options, converter_options)

    x_values = []
    y_values = []
    z_values = []

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()

        if topic_name == "/track_erro_topic":  # 修改为实际的 topic 名称
            msg_type = get_message("geometry_msgs/Point")
            point_msg = deserialize_message(data, msg_type)

            # 获取 x, y, z 坐标值
            x = point_msg.x
            y = point_msg.y
            z = point_msg.z

            # 将解析的数据存储到列表中/track_erro_topic
            x_values.append(x)
            y_values.append(y)
            z_values.append(z)
            

    # 绘制 x, y, z 坐标值随时间的变化
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))

    ax1.plot( x_values, linestyle='-', color='blue', linewidth=2)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('value')
    ax1.set_title('Control_Radius')

    ax2.plot(y_values, linestyle='-', color='green', linewidth=2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('value')
    ax2.set_title('Control_Radius_line_vel')

    ax3.plot(z_values, linestyle='-', color='red', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('value')
    ax3.set_title('vel')

    # 保存图像
    plt.tight_layout()
    plt.savefig("data/0822_PNG/" + save_pic_name + ".png")
    print(f"Figure saved to {save_pic_name}")

    print(f"Parsed {len(x_values)} points")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 parse_ros2_bag_for_point.py <path_to_bag_file> <save_image_name>")
        sys.exit(1)

    bag_file_path = sys.argv[1]
    save_pic_name = sys.argv[2]

    try:
        parse_ros2_bag_for_point(bag_file_path, save_pic_name)
    except Exception as e:
        print(f"Failed to parse the bag file: {e}")
