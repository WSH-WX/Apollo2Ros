import sys
import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import tf_transformations

def parse_ros2_bag_for_point(bag_file_path):
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader.open(storage_options, converter_options)

    x_values = []
    y_values = []
    z_values = []
    yaw_values = []
    timestamps = []

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()

        if topic_name == "/robot_pose_current":  # 修改为实际的 topic 名称  /goal_pose
            msg_type = get_message("geometry_msgs/PoseStamped")
            pose_msg = deserialize_message(data, msg_type)

            # 获取 x, y, z 坐标值
            x = pose_msg.pose.position.x
            y = pose_msg.pose.position.y
            z = pose_msg.pose.position.z

            # 从 header 中提取时间戳信息
            secs = pose_msg.header.stamp.sec
            nsecs = pose_msg.header.stamp.nanosec
            msg_timestamp = secs + nsecs * 1e-9

            # 提取四元数
            orientation_q = pose_msg.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    
            # 将四元数转换为欧拉角
            (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)


            # 将解析的数据存储到列表中/track_erro_topic
            x_values.append(x)
            y_values.append(y)
            z_values.append(z)
            yaw_values.append(yaw)
            timestamps.append(msg_timestamp)
    return x_values,y_values,z_values,yaw_values,timestamps
    
def parse_ros2_bag_for_point2(bag_file_path):
    reader2 = rosbag2_py.SequentialReader()

    storage_options2 = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options2 = rosbag2_py.ConverterOptions('', '')

    reader2.open(storage_options2, converter_options2)

    x_values2 = []
    y_values2 = []
    z_values2 = []
    timestamps2 = []

    while reader2.has_next():
        topic_name, data, timestamp = reader2.read_next()
        # /lookahead_point_world lookahead_point
        if topic_name == "/lookahead_point":  # 修改为实际的 topic 名称
            msg_type = get_message("geometry_msgs/PointStamped")
            point_msg = deserialize_message(data, msg_type)

            # 获取 x, y, z 坐标值
            x = point_msg.point.x
            y = point_msg.point.y
            z = point_msg.point.z

            print(f"x,y:{x, y}")

            # 从 header 中提取时间戳信息
            secs = point_msg.header.stamp.sec
            nsecs = point_msg.header.stamp.nanosec
            msg_timestamp = secs + nsecs * 1e-9

            # 将解析的数据存储到列表中
            x_values2.append(x)
            y_values2.append(y)
            z_values2.append(z)
            timestamps2.append(msg_timestamp)
    return x_values2,y_values2,z_values2,timestamps2

def parse_ros2_bag_for_plan(bag_file_path):
    
    #--------------------------------------planner--------------------------------------
    # 定义一个日志筛选字段的列表
    # 数据类型要求：nav_msgs/msg/Path
    filter_keys_plan = ["/unsmoothed_plan", "/plan_smoothed", "/plan" ]
    filter_keys_plan = ["/plan" ,"/unsmoothed_plan"]
    num_cols = 3
    data_plan = {plan_key: np.empty((0, num_cols)) for plan_key in filter_keys_plan}
    
    # list
    plan_list = []

    unsmoothed_plan_list = []

    # 读取plan消息
    for plan_key in filter_keys_plan:
        print(f"Checking plan_key: {plan_key}")
        # 初始化BagReader
        reader = rosbag2_py.SequentialReader()
        # 设置存储选项
        storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        # 打开bag文件
        reader.open(storage_options, converter_options)
        
        while reader.has_next():
            topic_name, data, timestamp = reader.read_next()
            
            if topic_name == plan_key:
                print(f"Processing topic: {topic_name}")
                msg_type = get_message("nav_msgs/Path")
                path_msg = deserialize_message(data, msg_type)

                # 提取并转换时间戳
                timestamp_sec = path_msg.header.stamp.sec
                timestamp_nanosec = path_msg.header.stamp.nanosec
                timestamp_path = timestamp_sec + timestamp_nanosec * 1e-9

                print(f"path_timestamp: {timestamp_path}")

                for pose_stamped in path_msg.poses:
                    # 从四元数中计算 yaw 角度
                    q = pose_stamped.pose.orientation
                    yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    
                    # 临时数组
                    temp_array = np.array([pose_stamped.pose.position.x,
                                           pose_stamped.pose.position.y,
                                           yaw])
                
                    # 数组存放
                    data_plan[plan_key] = np.vstack([data_plan[plan_key], temp_array])

                
                # 将数组转换为元组，并动态添加到集合
                if plan_key == "/plan":
                    plan_list.append(tuple(map(tuple, data_plan[plan_key])))
                if plan_key == "/unsmoothed_plan":
                    unsmoothed_plan_list.append(tuple(map(tuple, data_plan[plan_key])))
                # 清理
                data_plan[plan_key] = np.empty((0, num_cols))
    #-----------------------------------------------------------------------------------
    return plan_list,unsmoothed_plan_list

def plot_and_save_plan(x_values,y_values,z_values,yaw_values,timestamps,
                       x_values2,y_values2,z_values2,timestamps2,plan_list,
                        save_pic_name):
    # 绘制 x, y, z 坐标值随时间的变化
    # fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
    for item in plan_list:
        plan_temp = np.array(item)
        # ax1.plot( x_values, y_values ,linestyle='-', color='red', linewidth=0.5)
        
        # ax2.plot(x_values2, y_values2,color='blue', linewidth=0.5)

        plt.plot( x_values2 ,linestyle='-', color='red', linewidth=0.5)


#     ax1.plot( x_values, y_values ,label = 'current_pose',linestyle='-', color='blue', linewidth=2)
#     ax1.scatter( x_values2, y_values2 ,color='red', linewidth=2)
#     ax1.set_xlabel('X Position')
#     ax1.set_ylabel('Y Position')
#     ax1.set_title('Pose')
    
#     ax2.plot([a - b for a, b in zip(x_values2, x_values)],linestyle='-', color='green', linewidth=2)
#     ax2.set_xlabel('Time (s)')
#     ax2.set_ylabel('x')
#     ax2.set_title('erro_x')

#     ax3.plot([a - b for a, b in zip(y_values2, y_values)],linestyle='-', color='green', linewidth=2)
#     ax3.set_xlabel('Time (s)')
#     ax3.set_ylabel('y')
#     ax3.set_title('erro_y')

    # 保存图像
    plt.tight_layout()
    plt.legend()
    plt.savefig("data/0822_PNG/" + save_pic_name + ".png")
    print(f"Figure saved to {save_pic_name}")

    print(f"Parsed {len(x_values)} points")
    print(f"Parsed {len(x_values2)} points")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 parse_ros2_bag_for_point.py <path_to_bag_file> <save_image_name>")
        sys.exit(1)

    bag_file_path = sys.argv[1]
    save_pic_name = sys.argv[2]

    try:
        x_values,y_values,z_values,yaw_values,timestamps = parse_ros2_bag_for_point(bag_file_path)
        x_values2,y_values2,z_values2,timestamps2 = parse_ros2_bag_for_point2(bag_file_path)
        plan_list = parse_ros2_bag_for_plan(bag_file_path)
        plot_and_save_plan(x_values,y_values,z_values,yaw_values,timestamps,
                       x_values2,y_values2,z_values2,timestamps2,plan_list,
                        save_pic_name)
    except Exception as e:
        print(f"Failed to parse the bag file: {e}")