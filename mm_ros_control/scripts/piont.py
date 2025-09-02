import open3d as o3d
import numpy as np

# 加载 .pcd 文件
pcd = o3d.io.read_point_cloud("/home/szy/save_map.pcd")

# 转换为 numpy 数组以便操作
points = np.asarray(pcd.points)

# 对 x 坐标取负值
points[:, 0] = -points[:, 0] 
points[:, 1] = -points[:, 1]
points[:, 2] =  points[:, 2] 
for point in  points:
    if point[2] < 0.15 or point[0] < 0.8:
         point[0] = -2
    
point = points[0]
# 将修改后的坐标赋值回点云对象
pcd.points = o3d.utility.Vector3dVector(points)
voxel_size = 0.05
pcd_downsampled = pcd.voxel_down_sample(voxel_size)
#添加地面
# for i in range(-40, 200):
#     for j in range(-100, 100):
#         pcd_downsampled.points.append([i*0.05, j*0.05, -0.05])
# 保存修改后的点云（可选）
o3d.io.write_point_cloud("/home/szy/downsampled.pcd", pcd_downsampled)
