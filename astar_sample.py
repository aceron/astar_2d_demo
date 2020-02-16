import time
import threading
import cv2
import numpy as np

map_img = cv2.resize(cv2.imread('testlab.png', cv2.IMREAD_UNCHANGED), (100, 75))
map_img_temp = map_img.copy()

mutex_mat = threading.Lock()
buffer_mat = np.zeros(map_img.shape, np.uint8)

continue_display = True

class a_node:
    parent_node = None
    g_cost = 0
    h_cost = 0
    f_cost = 0
    position = None

    def __init__(self):
        self.g_cost = 0
        self.h_cost = 0
        self.f_cost = 0
        self.parent_node = None
        self.position = []

    def setPosition(self, new_pos):
        self.position = new_pos.copy()

    def setParent(self, new_node):
        self.parent_node = new_node

    def calculateCost(self, start_position, goal_position):
        dif_x = self.position[0]-start_position[0]
        dif_y = self.position[1]-start_position[1]
        self.g_cost = np.sqrt(dif_x*dif_x + dif_y*dif_y)
        dif_x = self.position[0]-goal_position[0]
        dif_y = self.position[1]-goal_position[1]
        self.h_cost = np.sqrt(dif_x*dif_x + dif_y*dif_y)
        self.f_cost = self.g_cost + self.h_cost

def display_thread():
    while continue_display:
        mutex_mat.acquire()
        cv2.imshow('Mat image',buffer_mat)
        cv2.waitKey(1)
        mutex_mat.release()
        time.sleep(1.0/60.0)
    cv2.destroyAllWindows()

threading.Thread(target=display_thread).start()

obstacle_map = cv2.cvtColor(map_img, cv2.COLOR_BGR2GRAY)

print("Map dimensions:")
print(obstacle_map.shape)

start = [10, 67]
finish = [70, 70]

lower_x = 0
upper_x = obstacle_map.shape[1] - 1
lower_y = 0
upper_y = obstacle_map.shape[0] - 1

map_nodes = []
for idx in range(0, obstacle_map.shape[1]):
    map_nodes.append([])
    for jdx in range(0, obstacle_map.shape[0]):
        map_nodes[len(map_nodes) - 1].append(None)

open_list = []
closed_list = []
obstacle_list = []
path = []

start_node = a_node()
start_node.setPosition(start)
start_node.calculateCost(start, finish)

open_list.append(start_node)

display_count = 0
max_cost = (upper_x + 1)*(upper_x + 1)*4
try:
    while(True):
        current_node = min(open_list, key = lambda x: x.f_cost)
        open_list.remove(current_node)
        closed_list.append(current_node)
        if current_node.position == finish:
            print("FOUND")
            parent_node = current_node.parent_node
            while(parent_node is not None):
                path.append(parent_node)
                parent_node = parent_node.parent_node
            for node in path:
                map_img[node.position[1],node.position[0]] = [0, 255, 255]

            map_img[start[1], start[0]] = [255, 0, 0]
            map_img[finish[1], finish[0]] = [0, 0, 255]

            mutex_mat.acquire()
            buffer_mat.data = map_img.data
            mutex_mat.release()    
        
            break
        
        for sidx in range(-1, 2):
            for sjdx in range(-1, 2):
                idx = current_node.position[0] + sidx
                jdx = current_node.position[1] + sjdx
                if idx >= lower_x and idx <= upper_x and jdx >= lower_y and jdx <= upper_y:
                    temp_pos = [idx, jdx]
                    temp_list_closed = list(filter(lambda x: x.position == temp_pos, closed_list))
                    if obstacle_map[jdx][idx] == 255 and len(temp_list_closed) == 0:
                        temp_list_open = list(filter(lambda x: x.position == temp_pos, open_list))
                        new_node = a_node()
                        new_node.setPosition([idx, jdx])
                        new_node.calculateCost(start, finish)
                        new_node.parent_node = current_node
                        if len(temp_list_open) == 0:
                            open_list.append(new_node)
                        else:
                            temp_node = open_list[open_list.index(temp_list_open[0])]
                            if temp_node.g_cost < current_node.g_cost:
                                open_list.remove(temp_node)
                                open_list.append(new_node)
                    elif obstacle_map[jdx][idx] < 255:
                        temp_pos = [idx, jdx]
                        temp_list_obs = list(filter(lambda x: x.position == temp_pos, obstacle_list))
                        new_node = a_node()
                        new_node.setPosition([idx, jdx])
                        new_node.calculateCost(start, finish)
                        new_node.parent_node = current_node
                        new_node.g_cost = max_cost
                        new_node.h_cost = max_cost
                        new_node.f_cost = max_cost
                        if len(temp_list_obs) == 0:
                            obstacle_list.append(new_node)
                        if len(temp_list_closed) == 0:
                            closed_list.append(new_node)

        if display_count == 10:
            map_img = map_img_temp.copy()

            for node in open_list:
                map_img[node.position[1], node.position[0]] = [0,255 - int(255.0*((node.f_cost)*(node.f_cost)/max_cost)),int(255.0*((node.f_cost)*(node.f_cost)/max_cost))]#[0,0,int(((node.f_cost)*(node.f_cost))/2)]  

            for node in closed_list:
                map_img[node.position[1], node.position[0]] = [int(255.0*((node.f_cost)*(node.f_cost)/max_cost)),0,255 - int(255.0*((node.f_cost)*(node.f_cost)/max_cost))]               

            for node in obstacle_list:
                map_img[node.position[1], node.position[0]] = [32,32,32]  

            map_img[start[1], start[0]] = [255,0,0]
            map_img[finish[1], finish[0]] = [0,0,255]

            mutex_mat.acquire()
            buffer_mat = map_img.copy()
            mutex_mat.release()
            display_count = 0

        display_count += 1

        if len(open_list) == 0:
            print("NOT FOUND")
            break
except KeyboardInterrupt:
    print("Program interrupted")
    continue_display = False
