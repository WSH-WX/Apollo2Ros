import sys
import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path

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

def parse_ros2_bag_for_amcl(bag_file_path):
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader.open(storage_options, converter_options)

    x_values = []
    y_values = []
    yaw_values = []
    covariances = []
    timestamps_acml = []

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

            # 提取并转换时间戳
            timestamp_sec = pose_with_cov_msg.header.stamp.sec
            timestamp_nanosec = pose_with_cov_msg.header.stamp.nanosec
            timestamp_acml = timestamp_sec + timestamp_nanosec * 1e-9

            # 存储提取的数据
            x_values.append(x)
            y_values.append(y)
            yaw_values.append(yaw)
            covariances.append(covariance)
            timestamps_acml.append(timestamp_acml)
    return x_values,y_values,yaw_values,covariances,timestamps_acml
# 绘制轨迹图
def plot_and_save_plan(plan_list,unsmoothed_plan_list, x_values,y_values,
                       yaw_values,timestamps_acml,save_pic_name):
    
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(8, 12))

    i = 0
    j = 0
    # 选择绘制第几组轨迹
    set_times = 1
    # 每隔5个点绘制一个箭头
    step = 2
    # 遍历集合并将元组解析回 NumPy 数组
    for item in plan_list:
        plan_temp = np.array(item)
        # unsmoothed_plan_temp = np.array(item2)
        # plt.plot(plan_temp[:,0], plan_temp[:,1],color='blue', linewidth=0.5)
        # plt.plot(unsmoothed_plan_temp[:,0], unsmoothed_plan_temp[:,1],color='bule', linewidth=0.5)
        i = i + 1
        if i == set_times:
            ax1.plot(plan_temp[:,0], plan_temp[:,1],label = 'smooth_path',color='blue',linewidth=0.5)
            ax2.plot(plan_temp[:,0], plan_temp[:,1],label = 'smooth_path',color='blue',linewidth=0.5)
            ax2.quiver(plan_temp[::step,0], plan_temp[::step,1],
                       np.cos(plan_temp[::step,2]),np.sin(plan_temp[::step,2]),
                       scale=50,
                       color='y')
            ax3.plot(plan_temp[:,2],label = 'path_yaw',linestyle='-',color='red', linewidth=0.5)
            break
    for item in unsmoothed_plan_list:
        plan_temp = np.array(item)
        j = j + 1
        if j == set_times:
            ax1.plot(plan_temp[:,0], plan_temp[:,1],label = 'raw_path',color='red', linewidth=0.5)
            break

    ax1.plot(x_values, y_values,label = 'acml_path',linestyle='--',color='red', linewidth=0.5)
    
    timestamps_acml = np.array(timestamps_acml).astype(float)
    ax3.plot(yaw_values,label = 'acml_path_yaw',linestyle='-',color='b', linewidth=0.5)
    acml_length = len(timestamps_acml)
    # print(f"Acml_Time的长度是: {timestamps_acml}")
    print(f"Acml的长度是: {acml_length}")

    ax4.plot(x_values, y_values,label = 'acml_path',linestyle='--',color='red', linewidth=0.5)
    ax4.quiver(plan_temp[::step,0], plan_temp[::step,1],
                       np.cos(plan_temp[::step,2]),np.sin(plan_temp[::step,2]),
                       scale=50,
                       color='y')

    x_length = len(x_values)
    print(f"x_values的长度是: {x_length}")

    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_title('Planned Path')
    ax1.legend()
    # 移除上边线和右侧线
    ax1.spines['top'].set_visible(False)
    ax1.spines['right'].set_visible(False)

    ax2.set_xlabel('X Position')
    ax2.set_ylabel('Y Position')
    ax2.set_title('Planned Path with Yaw')
    ax2.legend()
    # 移除上边线和右侧线
    ax2.spines['top'].set_visible(False)
    ax2.spines['right'].set_visible(False)

    ax3.set_xlabel('T Time')
    ax3.set_ylabel('Yaw Position')
    ax3.set_title('Yaw')
    ax3.legend()
    # 移除上边线和右侧线
    ax3.spines['top'].set_visible(False)
    ax3.spines['right'].set_visible(False)

    ax4.set_xlabel('X Position')
    ax4.set_ylabel('Y Position')
    ax4.set_title('Planned Path with Yaw')
    ax4.legend()
    # 移除上边线和右侧线
    ax4.spines['top'].set_visible(False)
    ax4.spines['right'].set_visible(False)

    # 调整子图之间的间距
    plt.subplots_adjust(hspace=0.5)  # hspace 控制纵向间距
    # 保存图片
    plt.savefig("data/0820_PNG/"+save_pic_name+".png")
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

        x_values,y_values,yaw_values,covariances,timestamps_acml = parse_ros2_bag_for_amcl(bag_file_path)

        plot_and_save_plan(plan_list,unsmoothed_plan_list, x_values,y_values,
                           yaw_values,timestamps_acml,save_pic_name)
    except Exception as e:
        print(f"Failed to parse the bag file: {e}")
