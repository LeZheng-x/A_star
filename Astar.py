import open3d as o3d 
import heapq

class Node:
    def __init__(self,open3d_voxel) -> None:
        
        #mean the voxel coordinate  
        self.x = open3d_voxel[0] 
        self.y = open3d_voxel[1] 
        self.z = open3d_voxel[2] 
        self.g = 0 
        self.h = 0
        self.f = 0
        self.parent = None 

    def __lt__(self,other):
        return self.f < other.f    
    
    def __eq__(self, other):
        return isinstance(other, Node) and self.x == other.x and self.y == other.y and self.z == other.z

    def __hash__(self):
        return hash(( self.x, self.y,self.z ))
    
    def ifequal(self,open3d_voxel):
        return self.x == open3d_voxel[0] and self.y == open3d_voxel[1] and self.z == open3d_voxel[2]
    
def a_star_serach(voxel_grid,start_point,goal_point):


    #step1: 将点转换为对应的voxel
    start_voxel = voxel_grid.get_voxel(start_point)
    goal_voxel  = voxel_grid.get_voxel(goal_point)

    start_node = Node(start_voxel)
    goal_node  = Node(goal_voxel)


    #step2: 查询voxel是否被占据，如果被占据，则选择至最接近且free的目标点
    occupucy_list =  voxel_grid.check_if_included(o3d.utility.Vector3dVector([start_point,goal_point]))
    if  occupucy_list[0] or occupucy_list[1]  :
            return None

    #steps:开始路径规划，计算距离
    open_list = [] 
    closed_list = set()

    heapq.heappush(open_list,start_node)

    # path = []
    while(open_list):
        current_node = heapq.heappop(open_list)
        # path.append([current_node.x,current_node.y,current_node.z])
        if current_node.ifequal(goal_voxel):
           
            path=[]
            while current_node is not None:
                path.append(current_node)
                current_node = current_node.parent
            return path     
        
        closed_list.add(current_node)
        

        for i in range (-1,2):
            for j in range(-1,2):
                for k in range(-1,2):
                    if i==0 and j==0 and k==0:
                        continue
                    neighbor_x = current_node.x+i 
                    neighbor_y = current_node.y+j
                    neighbor_z = current_node.z+k

                    #选择是否构造新的neighbor_node 

                    neighbor_point = voxel_grid.get_voxel_center_coordinate([neighbor_x,neighbor_y,neighbor_z])
                    
                   
                    if not all(element==0 for element in neighbor_point): #排除障碍物
                        continue


                    neighbor_node_wait = Node([neighbor_x,neighbor_y,neighbor_z]) 

                    if neighbor_node_wait in closed_list: #跳过以搜寻的点
                        continue

                    neighbor_node_wait.parent = current_node
                    neighbor_node_wait.g = current_node.g+1
                    neighbor_node_wait.h = calc_h(neighbor_node_wait,goal_node)
                    neighbor_node_wait.f = neighbor_node_wait.g +2* neighbor_node_wait.h

                    neighbor_node = search_in_list(open_list,neighbor_node_wait)

                    if neighbor_node == None:
                        neighbor_node = neighbor_node_wait
                        heapq.heappush(open_list,neighbor_node_wait)
                         

                    else: #进行对比操作
                        if neighbor_node_wait.f <neighbor_node.f:
                            index = open_list.index(neighbor_node)
                            open_list[index] = neighbor_node_wait
                            heapq.heapify(open_list)

    return None

def calc_h(neighbor_node,goal_node):
    return abs(neighbor_node.x-goal_node.x) + abs(neighbor_node.y-goal_node.y) + abs(neighbor_node.z-goal_node.z)

def search_in_list(node_list, target_node):
    for node in node_list:
        if node == target_node:
            return node

    else:
        return None

def update_list(node_list,target_node):
    for node in node_list:
        if node == target_node:
            node = target_node
            return

    node_list.add(target_node)    