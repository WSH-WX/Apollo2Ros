import sys
import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def parse_ros2_bag_for_amcl(bag_file_path, save_pic_name):
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader.open(storage_options, converter_options)

    x_values = []
    y_values = []
    yaw_values = []
    covariances = []

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()

        if topic_name == "/amcl_pose":
            msg_type = get_message("geometry_msgs/PoseWithCovarianceStamped")
            pose_with_cov_msg = deserialize_message(data, msg_type)

            # 提取位置、方向和协方差
            x = pose_with_cov_msg.pose.pose.position.x
            y = pose_with_cov_msg.pose.pose.position.y
            yaw = np.arctan2(2.0 * (pose_with_cov_msg.pose.pose.orientation.w * pose_with_cov_msg.pose.pose.orientation.z + 
                                    pose_with_cov_msg.pose.pose.orientation.x * pose_with_cov_msg.pose.pose.orientation.y),
                            1.0 - 2.0 * (pose_with_cov_msg.pose.pose.orientation.y * pose_with_cov_msg.pose.pose.orientation.y + 
                                        pose_with_cov_msg.pose.pose.orientation.z * pose_with_cov_msg.pose.pose.orientation.z))
            covariance = pose_with_cov_msg.pose.covariance

            # 打印调试信息
            # print(f"x: {x}, y: {y}, yaw: {yaw}")
            # print(f"Covariance: {covariance}")

            # 存储提取的数据
            x_values.append(x)
            y_values.append(y)
            yaw_values.append(yaw)
            covariances.append(covariance)

    # 绘制位置和方向
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))

    ax1.plot(x_values, y_values, linestyle='-', color='blue', linewidth=2)
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_title('AMCL Estimated Path (X vs Y)')
    ax1.grid(True)

    ax2.plot(yaw_values, linestyle='-', color='green', linewidth=2)
    ax2.set_xlabel('Index')
    ax2.set_ylabel('Yaw Angle (radians)')
    ax2.set_title('Yaw Angle over Time')
    ax2.grid(True)

    # 保存图像
    plt.tight_layout()
    plt.savefig("data/PNG/"+save_pic_name+".png")
    print(f"Figure saved to {save_pic_name}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 parse_ros2_bag_for_amcl.py <path_to_bag_file> save_image_name")
        sys.exit(1)

    bag_file_path = sys.argv[1]
    save_pic_name = sys.argv[2]

    try:
        parse_ros2_bag_for_amcl(bag_file_path, save_pic_name)
    except Exception as e:
        print(f"Failed to parse the bag file: {e}")
