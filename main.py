# Import libraries
import numpy as np
import open3d as o3d 
import Astar
import time
# Load a point cloud
ply_file_path_read = "room2_mesh.ply"


cd_load_ply = o3d.io.read_point_cloud(ply_file_path_read)

# 创建3D占据栅格地图


voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(cd_load_ply,
                                                            voxel_size=0.05)

start_point = [0,0,0]

goal_point  = [0,0,10.4]

start_time = time.time()
path_list =  Astar.a_star_serach(voxel_grid=voxel_grid,start_point=start_point,goal_point=goal_point)
start_time2 = time.time()


print("cost: {} s".format(start_time2-start_time))
o3d.visualization.draw_geometries([cd_load_ply])

o3d.visualization.draw_geometries([voxel_grid])





