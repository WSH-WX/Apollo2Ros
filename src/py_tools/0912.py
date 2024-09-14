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
    # 数据类型要求：nav_msgs/msg/Path osqp_plan_profile
    filter_keys_plan = ["/unsmoothed_plan", "/plan_smoothed", "/plan" ,"segemnt_path","pruned_path"]
    filter_keys_plan = ["/plan"]

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
                if plan_key == "/osqp_plan_profile":
                    unsmoothed_plan_list.append(tuple(map(tuple, data_plan[plan_key])))
                # 清理
                data_plan[plan_key] = np.empty((0, num_cols))
    #-----------------------------------------------------------------------------------
    return plan_list,unsmoothed_plan_list

# 绘制轨迹图
def plot_and_save_plan(plan_list,unsmoothed_plan_list,save_pic_name):
    fig, axes = plt.subplots(2, 1, figsize=(10, 12))
    for item in plan_list:
        plan_temp = np.array(item)
        axes[0,0].plot(plan_temp[:,0], plan_temp[:,1],color='red', label = "path",linewidth=0.5)
    for item2 in unsmoothed_plan_list:
        plan_temp = np.array(item2)
        # axes[0,1].plot(plan_temp[:,0],color='red', label = "path",linewidth=0.5)

    axes[0, 0].set_xlabel('X')
    axes[0, 0].set_xlabel('Y')
    axes[0, 0].set_title('Path')
    axes[0, 0].legend()

    axes[0, 1].set_xlabel('X')
    axes[0, 1].set_xlabel('Y')
    axes[0, 1].set_title('Path')
    axes[0, 1].legend()

    # 保存图片
    plt.savefig("data/0912_PNG/"+save_pic_name+".png")
    print(f"Figure saved to {save_pic_name}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 parse_ros2_bag_for_plan.py <path_to_bag_file> save_image_name")
        sys.exit(1)

    bag_file_path = sys.argv[1]
    save_pic_name = sys.argv[2]

    try:
        plan_list,unsmoothed_plan_list = parse_ros2_bag_for_plan(bag_file_path)
        
        plot_and_save_plan(plan_list,unsmoothed_plan_list,save_pic_name)
    except Exception as e:
        print(f"Failed to parse the bag file: {e}")