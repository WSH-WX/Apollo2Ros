import sys
import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path
import tf_transformations

def parse_ros2_bag_for_plan(bag_file_path):
    
    #--------------------------------------planner--------------------------------------
    # 定义一个日志筛选字段的列表
    # 数据类型要求：nav_msgs/msg/Path
    filter_keys_plan = ["/unsmoothed_plan", "/plan_smoothed", "/plan" ,"segemnt_path","pruned_path"]
    filter_keys_plan = ["/plan","/segemnt_path"]

    num_cols = 3
    data_plan = {plan_key: np.empty((0, num_cols)) for plan_key in filter_keys_plan}
    
    # list
    plan_list = []
    unsmoothed_plan_list = []

    # 读取plan消息
    for plan_key in filter_keys_plan:
        # print(f"Checking plan_key: {plan_key}")
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
                if plan_key == "/segemnt_path":
                    unsmoothed_plan_list.append(tuple(map(tuple, data_plan[plan_key])))
                # 清理
                data_plan[plan_key] = np.empty((0, num_cols))
    #-----------------------------------------------------------------------------------
    return plan_list,unsmoothed_plan_list

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

    x_values3 = []
    y_values3 = []
    z_values3 = []
    timestamps3 = []

    while reader2.has_next():
        topic_name, data, timestamp = reader2.read_next()
        # /lookahead_point_world lookahead_point
        if topic_name == "/lookahead_point_world":  # 修改为实际的 topic 名称
            msg_type = get_message("geometry_msgs/PointStamped")
            point_msg = deserialize_message(data, msg_type)

            # 获取 x, y, z 坐标值
            x = point_msg.point.x
            y = point_msg.point.y
            z = point_msg.point.z

            # print(f"x,y:{x, y}")

            # 从 header 中提取时间戳信息
            secs = point_msg.header.stamp.sec
            nsecs = point_msg.header.stamp.nanosec
            msg_timestamp = secs + nsecs * 1e-9

            # 将解析的数据存储到列表中
            x_values3.append(x)
            y_values3.append(y)
            z_values3.append(z)
            timestamps3.append(msg_timestamp)
    return x_values3,y_values3,z_values3,timestamps3

def parse_ros2_bag_for_float64(bag_file_path):
    reader2 = rosbag2_py.SequentialReader()

    storage_options2 = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options2 = rosbag2_py.ConverterOptions('', '')

    reader2.open(storage_options2, converter_options2)

    erro = []

    while reader2.has_next():
        topic_name, data, timestamp = reader2.read_next()
        if topic_name == "/erro_control":  # 修改为实际的 topic 名称
            msg_type = get_message("std_msgs/msg/Float64")
            float64_msg = deserialize_message(data, msg_type)

            # 获取 x, y, z 坐标值
            erro.append(float64_msg.data)
    return erro

def parse_ros2_bag_for_curvature(bag_file_path):
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
    # 打印 curvature_values 的长度
    curvature_values = np.concatenate(curvature_values)
    # print(f"Length of curvature_values: {curvature_values[0]}")
    return curvature_values

# 绘制轨迹图
def plot_and_save_plan(plan_list,unsmoothed_plan_list,x_values ,y_values ,x_values3,y_values3,erro,
                       curvature_values,yaw_values,save_pic_name):
#     fig, (ax1, ax2,ax3 ) = plt.subplots(2, 1, figsize=(10, 12))

    path_yaw = []
    fig, axes = plt.subplots(3, 2, figsize=(10, 12))
    for item2 in plan_list:
        plan_temp = np.array(item2)
        # 计算向量
        vectors = np.vstack([np.diff(plan_temp[:,0]), np.diff(plan_temp[:,1])]).T

        # 归一化向量
        norms = np.linalg.norm(vectors, axis=1, keepdims=True)
        normalized_vectors = vectors / norms

        # 计算相邻向量之间的夹角（使用点积）
        cos_angles = np.einsum('ij,ij->i', normalized_vectors[:-1], normalized_vectors[1:])
        angles = np.arccos(np.clip(cos_angles, -1.0, 1.0))

        # 找到转向角度最大的点，即尖点的索引
        cusp_index = np.argmax(angles) + 1  # +1 是因为角度对应的是第二个向量的起点

        # 输出尖点的索引
        print("Cusp index:", cusp_index)
        cusp_xy = [plan_temp[cusp_index,0],plan_temp[cusp_index,1]]

        axes[0,0].plot(plan_temp[:,0], plan_temp[:,1],color='red', label = "path",linewidth=0.5)
        axes[0,1].plot(plan_temp[:,0], plan_temp[:,1],color='red', label = "path",linewidth=0.5)
        axes[1,0].plot(plan_temp[:,0], plan_temp[:,1],color='red', label = "path",linewidth=0.5)
        path_yaw = plan_temp[:,2]
    for item2 in unsmoothed_plan_list:
        segment_plan_temp = np.array(item2)
        axes[0,0].plot(segment_plan_temp[:,0], segment_plan_temp[:,1],color='green',  label = "segment_path",linewidth=0.5)
        axes[1,0].plot(segment_plan_temp[:,0], segment_plan_temp[:,1],color='green',  label = "segment_path",linewidth=0.5)
        # axes[2,0].plot(segment_plan_temp[:,2],color='red',  label = "segment_path",linewidth=2)
        
    axes[0, 0].set_xlabel('X')
    axes[0, 0].set_xlabel('Y')
    axes[0, 0].set_title('Path')
    axes[0, 0].legend()
    # 遍历集合并将元组解析回 NumPy 数组
    tempX = []
    tempY = []
    for item in plan_list:
        plan_temp = np.array(item)
        tempX.append(plan_temp[0,0])
        tempY.append(plan_temp[0,1])
    # plt.scatter(tempX, tempY,color='red',s=1, marker='o')

    # robot_pose_current
    # 计算所有点与cusp_xy之间的欧氏距离
    distances = np.sqrt((x_values - cusp_xy[0])**2 + (y_values - cusp_xy[1])**2)

    # 找到距离最近的点的索引
    closest_index = np.argmin(distances)

    # 输出最近点的索引
    print("Closest index:", closest_index)
    axes[1,0].scatter(x_values, y_values,color='y',s=0.1, marker='*')
    
    axes[0, 1].set_xlabel('X')
    axes[0, 1].set_xlabel('Y')
    axes[0, 1].set_title('robot_pose_current')
    axes[0, 1].legend()

    # lookahead_point
    axes[0,1].scatter(x_values3, y_values3,color='green',s=0.1, marker='*') 
    axes[0, 1].set_xlabel('X')
    axes[0, 1].set_xlabel('Y')
    axes[0, 1].set_title('lookahead_point')
    axes[0, 1].legend()

    # erro
    axes[1,1].plot(erro,color='red',linewidth=1.5)
    axes[1,1].axvline(x=closest_index, color='r', linestyle='--', label='x = 750')
    axes[1, 1].set_xlabel('X')
    axes[1, 1].set_xlabel('Y')
    axes[1, 1].set_title('erro')
    axes[1, 1].legend()

    # yaw
    path_yaw_size = len(path_yaw)
    curr_yaw_size = len(yaw_values)
    # 生成原始path_yaw的索引
    original_indices = np.linspace(0, path_yaw_size - 1, path_yaw_size)

    # 生成插值后的索引
    target_indices = np.linspace(0, path_yaw_size - 1, curr_yaw_size)

    # 对path_yaw进行插值
    interpolated_path_yaw = np.interp(target_indices, original_indices, path_yaw)
    axes[2,0].plot(yaw_values,color='green', linewidth=0.5)
    axes[2,0].plot(interpolated_path_yaw,color='red', linewidth=0.5)
    axes[2,0].axvline(x=closest_index, color='r', linestyle='--', label='x = 750')
    axes[2, 0].set_xlabel('X')
    axes[2, 0].set_xlabel('Y')
    axes[2, 0].set_title('yaw')
    axes[2, 0].legend()

    # curvature
    # axes[2,1].plot(interpolated_path_yaw-yaw_values,color='red',linewidth=1.5)
    axes[2,1].plot(curvature_values,color='red',linewidth=1.5)
    axes[2,1].axvline(x=closest_index, color='r', linestyle='--', label='x = 750')
    axes[2, 1].set_xlabel('X')
    axes[2, 1].set_xlabel('Y')
    axes[2, 1].set_title('curvature')
    # axes[2, 1].set_title('heading_erro')
    axes[2, 1].legend()

#     print(f'erro_len:{len(erro)}')
#     print(f'lookahead_len:{len(x_values3)}')

    # 保存图片
    plt.savefig("data/0827_PNG/"+save_pic_name+".png")
    print(f"Figure saved to {save_pic_name}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 parse_ros2_bag_for_plan.py <path_to_bag_file> save_image_name")
        sys.exit(1)

    bag_file_path = sys.argv[1]
    save_pic_name = sys.argv[2]

    plan_num_column = 2

    try:
        plan_list,unsmoothed_plan_list = parse_ros2_bag_for_plan(bag_file_path)
        x_values,y_values,z_values,yaw_values,timestamps = parse_ros2_bag_for_point(bag_file_path)
        x_values3,y_values3,z_values3,timestamps3 = parse_ros2_bag_for_point2(bag_file_path)
        erro = parse_ros2_bag_for_float64(bag_file_path)
        curvature_values = parse_ros2_bag_for_curvature(bag_file_path)
        plot_and_save_plan(plan_list,unsmoothed_plan_list,x_values,y_values,x_values3,y_values3, erro,
                           curvature_values,yaw_values,save_pic_name)
    except Exception as e:
        print(f"Failed to parse the bag file: {e}")