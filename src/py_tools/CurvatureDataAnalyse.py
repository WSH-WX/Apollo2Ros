import sys
import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def parse_ros2_bag_for_plan(bag_file_path):
    # 定义一个日志筛选字段的列表
    # 数据类型要求：nav_msgs/msg/Path
    filter_keys_plan = ["/unsmoothed_plan", "/plan_smoothed", "/plan" ]
    filter_keys_plan = ["/plan","/unsmoothed_plan"]
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
    return plan_list,unsmoothed_plan_list


def parse_ros2_bag_for_curvature(bag_file_path, save_pic_name):
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader.open(storage_options, converter_options)

    curvature_values = []

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()

        if topic_name == "/curvature_control":
            msg_type = get_message("std_msgs/Float64MultiArray")
            curvature_msg = deserialize_message(data, msg_type)
            # 提取曲率控制数据
            curvature_values.append(curvature_msg.data)

    # 将数据转换为 numpy 数组，方便后续处理
    curvature_values = np.array(curvature_values)
    # 打印 curvature_values 的长度
    # print(f"Length of curvature_values: {curvature_values[0]}")
    return curvature_values
#     # 绘制曲率控制数据
#     fig, ax = plt.subplots(figsize=(10, 6))
#     j = 0
#     k = 0
#     temp = []
#     for i in range(curvature_values.shape[1]):
#         temp.append(curvature_values[:, i])
#         # print(f'{curvature_values[:, i]}')
#         if curvature_values[:, i] > 1.3333:
#             k += 1
#             print(f'path_k_j : {curvature_values[:, i]} {j}')
#         j += 1
#     print(f'num : {j}')
#     print(f'num : {k}')
    
#     # minR = 0.75 -> min_curvature = 1/minR = 1.33...

#     ax.plot(temp,color='red', linewidth=2)
#     ax.plot(temp,color='blue', linewidth=2)
#     ax.set_xlabel('Index')
#     ax.set_ylabel('Curvature Value')
#     ax.set_title('Curvature Control Data Over Time')
# #     ax.legend()

#     # 保存图像
#     plt.tight_layout()
#     plt.savefig(f"data/0823_PNG/{save_pic_name}.png")
#     print(f"Figure saved to {save_pic_name}.png")

def plot_and_save_plan(plan_list,unsmoothed_plan_list, curvature_values):
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 12))

    i = 0
    j = 0
    # 选择绘制第几组轨迹
    set_times = 1
    # 每隔5个点绘制一个箭头
    step = 2
    # 遍历集合并将元组解析回 NumPy 数组
    for item in plan_list:
        plan_temp = np.array(item)
        ax1.plot(plan_temp[:,0], plan_temp[:,1],color='blue', linewidth=0.5)
        ax1.scatter(plan_temp[184,0], plan_temp[184,1], color='red', s=100)  # 绘制一个红色的点，大小为100
        print(f'{len(plan_temp[:,0])}')
#         temp = []
#         for i in range(curvature_values.shape[1]):
#             temp.append(curvature_values[:, i])

    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_title('Planned Path')
    ax1.legend()
    # 移除上边线和右侧线
    ax1.spines['top'].set_visible(False)
    ax1.spines['right'].set_visible(False)
    # 调整子图之间的间距
    plt.subplots_adjust(hspace=0.5)  # hspace 控制纵向间距
    # 保存图片
    plt.savefig("data/0823_PNG/"+save_pic_name+".png")
    print(f"Figure saved to {save_pic_name}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 parse_ros2_bag_for_amcl.py <path_to_bag_file> save_image_name")
        sys.exit(1)

    bag_file_path = sys.argv[1]
    save_pic_name = sys.argv[2]

    try:
        curvature_values = parse_ros2_bag_for_curvature(bag_file_path, save_pic_name)
        plan_list,unsmoothed_plan_list = parse_ros2_bag_for_plan(bag_file_path)
        plot_and_save_plan(plan_list,unsmoothed_plan_list, curvature_values)
    except Exception as e:
        print(f"Failed to parse the bag file: {e}")
