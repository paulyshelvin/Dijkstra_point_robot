import numpy as np
import cv2
import time
import matplotlib.pyplot as plt

class Node:
    def __init__(self, number, x, y, cost = np.Inf):
        self.number = number
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = None

class Robot:
    def __init__(self, clearance, initial_x, initial_y, goal_x, goal_y): #radius of the bot is 0
        self.clearance = clearance
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.goal_x = goal_x
        self.goal_y = goal_y

#Prompt the user to input valid start location
def get_initial(height, length, clearance):
    print("Enter the initial node coordinates")
    x_initial =  int(input())
    y_initial = int(input())

    if check_conditions(x_initial, y_initial, clearance) == False:
        print("Invalid input. Please try again.")
        x_initial, y_initial = get_initial(height, length, clearance)

    return x_initial, y_initial

#Prompt the user to input valid goal location
def get_goal(height, length, clearance):
    print("Enter the goal node coordinates")
    x_goal =  int(input()) 
    y_goal = int(input())

    if check_conditions(x_goal, y_goal, clearance) == False:
        print("Invalid input. Please try again.")
        x_goal, y_goal = get_goal(height, length, clearance)

    return x_goal, y_goal

def check_conditions(x, y, clearance):
    flag = True

    #boundary conditions
    if x < 0 or x >= 400 or y < 0 or y >= 250:
        flag = False

    #check if point is  outside the circle while adjusting for clearance
    if np.sqrt(((x - 300) ** 2) + ((y - 185) ** 2)) <= 40 + clearance:
        flag = False

    #check if point is  outside the hexgon while adjusting for clearance
    h1 = y - 0.577* x - 24.97
    h2 = y + 0.577* x - 255.82
    h3 = x - 235 + clearance
    h6 = x - 165 + clearance
    h5 = y + 0.577 * x - 175
    h4 = y - 0.577 * x + 55.82
    if h1 < 0 and h2 < 0 and h3 < 0 and h4 > 0 and h5 > 0 and h6 > 0:
        flag = False

    #check if point is  outside the hexgon while adjusting for clearance
    l1 = y - ((0.316) * x) - 173.608  
    l2 = y + (1.23 * x) - 229.34 
    l3 = y + (3.2 * x) - 436 
    l4 = y - 0.857 * x - 111.42          
    l5 = y + (0.1136 * x) - 189.09
    if (l1 < 0 and l5 > 0 and l4 > 0) or (l2 > 0 and l5 < 0 and l3 < 0):
        flag = False

    return flag

#Finding the node with the least cost to come
def least_cost(node_list):
    min_node = Node(0, 0, 0, 0)
    idx = 0
    for i in range(len(node_list)):
        #check with cost map
        if node_list[i].cost < min_node.cost:
            min_node = node_list[i]
            idx = i
    return node_list.pop(i)

#Calling the appropriate action       
def actionspace(move, node, clearance):
    if move == 'N':
        return move_north(node, clearance)
    if move == 'S':
        return move_south(node, clearance)
    if move == 'W':
        return move_west(node, clearance)
    if move == 'E':
        return move_east(node, clearance)
    if move == 'NE':
        return move_north_east(node, clearance)
    if move == 'NW':
        return move_north_west(node, clearance)
    if move == 'SE':
        return move_south_east(node, clearance)
    if move == 'SW':
        return move_south_west(node, clearance)

def move_north(node, clearance):
    x = node.x
    y = node.y
    if check_conditions(x, y + 1, clearance) == True:
        cost = 1
        return [x, y + 1], cost
    else:
        return None, None

def move_north_east(node, clearance):
    x = node.x
    y = node.y
    if check_conditions(x + 1, y + 1, clearance) == True:
        cost = 1.4
        return [x + 1, y + 1], cost
    else:
        return None, None

def move_east(node, clearance):
    x = node.x
    y = node.y
    if check_conditions(x + 1, y, clearance) == True:
        cost = 1
        return [x + 1, y], cost
    else:
        return None, None

def move_south_east(node, clearance):
    x = node.x
    y = node.y
    if check_conditions(x + 1, y - 1, clearance) == True:
        cost = 1.4
        return [x + 1, y - 1], cost
    else:
        return None, None

def move_south(node, clearance):
    x = node.x
    y = node.y
    if check_conditions(x, y - 1, clearance) == True:
        cost = 1
        return [x, y - 1], cost
    else:
        return None, None

def move_south_west(node, clearance):
    x = node.x
    y = node.y
    if check_conditions(x - 1, y - 1, clearance) == True:
        cost = 1.4
        return [x - 1, y - 1], cost
    else:
        return None, None

def move_west(node, clearance):
    x = node.x
    y = node.y
    if check_conditions(x - 1, y, clearance) == True:
        cost = 1
        return [x - 1, y], cost
    else:
        return None, None

def move_north_west(node, clearance):
    x = node.x
    y = node.y
    if check_conditions(x - 1, y + 1, clearance) == True:
        cost = 1.4
        return [x - 1, y + 1], cost
    else:
        return None, None

def gen_path(goal_node, initial_node):
    path = []
    path.append(goal_node)
    parent_node = goal_node.parent
    while parent_node != None:
        path.append(parent_node) 
        parent_node = parent_node.parent_node
        
    path.append(initial_node)
    path.reverse()
    return path

def workspace():

    map = np.full((250,400), np.Infinity) 
    obstacle_map = []
    for y in range(250):
        for x in range(400):
            #Circle 
            if (185 - y) ** 2+(300 - x) ** 2 <= (40 ** 2):
                map[y, x] = -1
                obstacle_map.append([x, y])
            
            #Hexagon
            h1 = y - 0.577 * x - 24.97 
            h2 = y + 0.577 * x - 255.82
            h3 = x - 235 
            h6 = x - 165 
            h5 = y + 0.577 * x - 175 
            h4 = y - 0.577 * x + 55.82 
            if(h1 < 0 and h2 < 0 and h3 < 0 and h4 > 0 and h5 > 0 and h6 > 0):
                map[y, x] = -1
                obstacle_map.append([x, y])
            
            #Polygon Obstacle
            l1 = y - ((0.316) * x) - 173.608  
            l2 = y + (1.23 * x) - 229.34 
            l3 = y + (3.2 * x) - 436 
            l4 = y - 0.857* x - 111.42          
            l5 = y + (0.1136 * x) - 189.09
            if (l1 < 0 and l5 > 0 and l4 > 0) or (l2 > 0 and l5 < 0 and l3 < 0):
                map[y,x] = -1 
                obstacle_map.append([x, y])

    return map, obstacle_map

#Finding the index of the node in the list of nodes
def find_index(x, y, node_list):
    for i in range(len(node_list)):
        if node_list[i].x == x and node_list[i].y == y:
            return node_list.index(node_list)
        else:
            return None

def dijkstra(ws, robot_object, initial_node, img_show): 

    closed_nodes = []
    open_nodes = [initial_node]
    actions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
    number = 0
    while True:
        #Choosing next vertex from open nodes
        temp = least_cost(open_nodes)
        closed_nodes.append(temp)
        
        if temp.x == robot_object.goal_x and temp.y == robot_object.goal_y:
            return temp, img_show
         
        #exploring
        for action in actions:
            new_point, cost = actionspace(action, temp, robot_object.clearance)
            if new_point is not None:
                new_node = Node(0, new_point[0], new_point[1], np.Inf)
                
                #Updating cost to come
                if cost + temp.cost < ws[new_point[0]][new_point[1]] and new_node not in closed_nodes:
                    ws[new_point[0]][new_point[1]] = cost + temp.cost
                    new_node.parent = temp
                    new_node.cost = cost + temp.cost
                    
                    #Life would be simpler if I used a key for every node
                    if new_node in open_nodes and new_node not in closed_nodes:
                        idx = find_index(new_node.x, new_node.y, open_nodes)
                        open_nodes[idx] = new_node

                #Appending new nodes in opened nodes
                if new_node not in open_nodes and new_node not in closed_nodes:
                    new_node.number = number
                    img_show[new_node.y, new_node.x, :] = np.array([0,0,255])
                    open_nodes.append(new_node)
                    number += 1

x_start, y_start = get_initial(250, 400, 5)
x_goal, y_goal = get_goal(250, 400, 5)

robot = Robot(5, x_start, y_start, x_goal, y_goal)
start = time.time()

ws, obstacle_space = workspace()
initial_node = Node(0, x_start, y_start, 0)
img_show = np.dstack([ws.copy()*255, ws.copy()*255, ws.copy()*255])
goal, img = dijkstra(ws, robot, initial_node, img_show)
path = gen_path(goal, initial_node)

print("The shortest path is (x, y)")
for i in range(len(path)):
    img_show[path[i].y, path[i].x,:] = np.array([255,0,0])
    print(path[i].x, "   ", path[i].y)

cv2.imshow('img', img_show)
cv2.waitKey(0)