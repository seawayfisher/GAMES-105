import numpy as np
from scipy.spatial.transform import Rotation as R

def load_motion_data(bvh_file_path):
    """part2 辅助函数，读取bvh文件"""
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if lines[i].startswith('Frame Time'):
                break
        motion_data = []
        for line in lines[i+1:]:
            data = [float(x) for x in line.split()]
            if len(data) == 0:
                break
            motion_data.append(np.array(data).reshape(1,-1))
        motion_data = np.concatenate(motion_data, axis=0)
    return motion_data


def get_content_by_title(content:str, title:str):
    titleSet = {"HIERARCHY", "MOTION"}
    assert(title in titleSet )
    lines = content.split("\n")
    validLines = []
    curTitle = ""
    for line in lines:
        line = line.strip()
        isTitle = line in titleSet
        if isTitle:
            curTitle = line
            continue
        if curTitle != title: 
            continue
        validLines.append(line)
    return "\n".join(validLines)

def parse_content(content):
    joint_name = []
    joint_parent = []
    joint_offset = []
    lines = content.split("\n")
    joint_name, joint_parent, joint_offset = GetBVHByStack(lines)
    return joint_name, joint_parent, joint_offset

def GetBVHByStack(lines):
    stack = []
    answer = []
    sort_index = 0
    for line in lines:
        segments = line.split()
        first_segment = segments[0].strip()
        if first_segment == "ROOT" or first_segment == "JOINT":
            stack.append({"name": segments[1], "sort_index": sort_index}) #,  "parent_name": parent_name})
            sort_index+=1
            continue
        if first_segment == "End":
            parent_name = stack[-1].get("name")
            stack.append({"name": "".join([parent_name, "_end"]), "sort_index": sort_index}) #,  "parent_name": parent_name})
            sort_index+=1
            continue
        if first_segment == "{":
            continue
        if first_segment == "OFFSET":
            stack[-1]["offset"] = [float(value) for value in line.split()[1:] ] 
            continue
        if first_segment == "CHANNELS":
            continue
        if first_segment == "End":
            continue
        if first_segment == "}":
            item = stack.pop()
            parent_name = stack[-1].get("name", None) if stack else None

            item["parent_name"] = parent_name
            answer.append(item)
            continue

    joint_name = []
    joint_parent = []
    joint_offset = []

    joint_parent_name = []
    name2index = {}
    sorted_lst = sorted(answer, key=lambda x: x['sort_index'])
    index = 0
    for item in sorted_lst:
        name = item["name"].strip()
        name2index[name] = index
        joint_name.append(name)
        joint_offset.append(np.array(item["offset"], dtype=np.float64))
        joint_parent_name.append(item["parent_name"])

        index += 1

    for name in joint_parent_name:
        joint_parent.append(name2index.get(name, -1))

    
    return joint_name, joint_parent, joint_offset

        

def part1_calculate_T_pose(bvh_file_path):
    """请填写以下内容
    输入： bvh 文件路径
    输出:
        joint_name: List[str]，字符串列表，包含着所有关节的名字
        joint_parent: List[int]，整数列表，包含着所有关节的父关节的索引,根节点的父关节索引为-1
        joint_offset: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的相对偏移量

    Tips:
        joint_name顺序应该和bvh一致
    """
    with open(bvh_file_path, "r") as file:
        content = file.read()
        content = get_content_by_title(content, "HIERARCHY")
    joint_name, joint_parent, joint_offset = parse_content(content)
    return joint_name, joint_parent, joint_offset


def part2_forward_kinematics(joint_name, joint_parent, joint_offset, motion_data, frame_id):
    """请填写以下内容
    输入: part1 获得的关节名字，父节点列表，偏移量列表
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数
        frame_id: int，需要返回的帧的索引
    输出:
        joint_positions: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的全局位置
        joint_orientations: np.ndarray，形状为(M, 4)的numpy数组，包含着所有关节的全局旋转(四元数)
    Tips:
        1. joint_orientations的四元数顺序为(x, y, z, w)
        2. from_euler时注意使用大写的XYZ
    """

    joint_positions = []  # todo 改成numpy
    joint_orientations = []

    frame_data = motion_data[frame_id]
    for index, name in enumerate(joint_name):
        parent_index =joint_parent[index]
        parent_pos = frame_data[:3] if parent_index < 0 else joint_positions[parent_index]  # 因为root的父是-1
        postion = parent_pos + joint_offset[index]
        joint_positions.append(postion)  #  位置偏移直接累加

    for index, name in enumerate(joint_name):
        parent_index =joint_parent[index]
        readIndex = index + 1 #  
        rotation_vec3 = frame_data[3* readIndex: 3*readIndex + 3]
        # quat = R.from_rotvec(rotation_vec3).as_quat()
        euler_angle = R.from_euler('XYZ', rotation_vec3, degrees=True)
        offset_matrix = euler_angle.as_matrix()
        parent_quat = R.from_quat(joint_orientations[parent_index] if parent_index >=0 else [0,0,0,0])  # TODO 四个零不一定对
        parent_matrix = parent_quat.as_matrix()
        curr_maxtrix = parent_matrix * offset_matrix
        joint_orientations.append(curr_maxtrix.as_quat())

        # 旋转的话
    return joint_positions, joint_orientations


def part3_retarget_func(T_pose_bvh_path, A_pose_bvh_path):
    """
    将 A-pose的bvh重定向到T-pose上
    输入: 两个bvh文件的路径
    输出: 
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数。retarget后的运动数据
    Tips:
        两个bvh的joint name顺序可能不一致哦(
        as_euler时也需要大写的XYZ
    """
    motion_data = None
    return motion_data
