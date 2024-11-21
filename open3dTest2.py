'''
Author: your name
Date: 2021-09-13 08:44:04
LastEditTime: 2021-09-13 09:05:51
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: \open3d_code\test_data\08_ICPpeizhun.py
'''
import open3d as o3d
import numpy as np
import copy


# 输入是两个点云和一个初始转换，该转换将源点云和目标点云大致对齐，输出是精确的变换，使两点云紧密对齐。
# 可视化帮助函数
# 将目标点云和源点云可视化，并通过对齐变换对其进行转换。
# 目标点云和源点云分别用青色和黄色绘制。两点云重叠的越紧密，对齐的结果就越好。
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])


# 输入
source = o3d.io.read_point_cloud("template-gai.pcd")
target = o3d.io.read_point_cloud("target-1.pcd")
threshold = 0.01
trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                         [-0.139, 0.967, -0.215, 0.7],
                         [0.487, 0.255, 0.835, -1111.4],
                         [0.0, 0.0, 0.0, 1.0]])
draw_registration_result(source, target, trans_init)
# 该函数evaluate_registration计算两个主要指标。fitness计算重叠区域（内点对应关系/目标点数）。越高越好。inlier_rmse计算所有内在对应关系的均方根误差RMSE。越低越好。
reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
                                                      o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                      o3d.pipelines.registration.ICPConvergenceCriteria(
                                                          max_iteration=2000))
print(reg_p2p)
print("Transformation is:")
print(reg_p2p.transformation)
draw_registration_result(source, target, reg_p2p.transformation)