import open3d as o3d
import numpy as np


class PointCloudLayerViewer:
    def __init__(self, pcd, layer_thickness=0.1):
        self.pcd = pcd
        self.points = np.asarray(pcd.points)

        # Step 1: 估计法向量并确定正面方向
        print("Estimating normals...")
        self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        self.pcd.orient_normals_consistent_tangent_plane(100)  # 使法向量方向一致

        # 法向量可以帮助我们确定"正面"，例如我们使用Z轴方向
        self.normals = np.asarray(self.pcd.normals)
        self.min_z = np.min(self.points[:, 2])
        self.max_z = np.max(self.points[:, 2])
        self.current_layer = 0
        self.layer_thickness = layer_thickness
        self.vis = o3d.visualization.VisualizerWithKeyCallback()

    def show_layer(self):
        """显示当前层的点云"""
        min_z_layer = self.min_z + self.current_layer * self.layer_thickness
        max_z_layer = min_z_layer + self.layer_thickness

        # 提取当前层的点（按 z 值分层）
        layer_points = self.points[
            (self.points[:, 2] >= min_z_layer) & (self.points[:, 2] < max_z_layer)
            ]

        # 创建新点云
        layer_pcd = o3d.geometry.PointCloud()
        layer_pcd.points = o3d.utility.Vector3dVector(layer_points)

        # 如果需要，将每层的法向量也显示出来
        # 提取法向量，并将其添加到显示的点云上
        layer_normals = self.normals[
            (self.points[:, 2] >= min_z_layer) & (self.points[:, 2] < max_z_layer)
            ]
        layer_pcd.normals = o3d.utility.Vector3dVector(layer_normals)

        # 清除上一个点云并显示新的点云
        self.vis.clear_geometries()
        self.vis.add_geometry(layer_pcd)
        print(f"Displaying layer {self.current_layer}, z range: {min_z_layer:.2f} to {max_z_layer:.2f}")

    def next_layer(self, vis):
        """切换到下一层并更新显示"""
        if self.current_layer < (self.max_z - self.min_z) / self.layer_thickness - 1:
            self.current_layer += 1
            self.show_layer()
        else:
            print("已经是最后一层")

    def run(self):
        # 设置键盘回调
        self.vis.create_window()
        self.vis.register_key_callback(ord("S"), self.next_layer)

        # 显示第一层
        self.show_layer()

        # 开始渲染
        self.vis.run()


# 读取点云文件
pcd = o3d.io.read_point_cloud("test1.pcd")

# 初始化查看器，设置每层的厚度
layer_thickness = float(input("请输入每层的厚度："))  # 允许用户自定义层厚度
viewer = PointCloudLayerViewer(pcd, layer_thickness=layer_thickness)

# 运行查看器
viewer.run()
