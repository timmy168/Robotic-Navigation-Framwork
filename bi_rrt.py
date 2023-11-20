import numpy as np
import argparse
import os
import random
import math
import cv2
import matplotlib.pyplot as plt

class Nodes:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.x_parent = []
        self.y_parent = []

class RRTree:
    def __init__(self, map, offset, iteration):
        # create black obstacle map
        img = cv2.imread(map, cv2.IMREAD_GRAYSCALE) 
        img[np.where((img[:,:] != 255))] = 0
        dilation = cv2.erode(img, np.ones((3,3), np.uint8), iterations = 12)
        # cv2.imshow('dilation',dilation)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        self.dilation = dilation 
        self.map = cv2.imread(map) 
        self.iteration = iteration
        self.start = [0, 0]
        self.goal = [0, 0]
        self.offset = offset
        self.start_nodes = [()]
        self.goal_nodes = [()]
        self.target = " "

    def set_target(self,target):
        object = {"refrigerator":(455,457),
                  "lamp":(894,657),
                  "cushion":(1039,454),
                  "rack":(748,284),
                  "cooktop":(380,545)}
        x = object[target][0]
        y = object[target][1]
        self.target = target
        self.goal = (x,y)
    
    # generate a random point in the 2D image pixel
    def random_point(self, height, width):
        random_x = random.randint(0, width)
        random_y = random.randint(0, height)
        return (random_x,random_y)
    
    # Calculate L2 distance between new point and nearest node
    def distance(self,x1,y1,x2,y2):
        return math.sqrt(((x1-x2) ** 2 ) + ((y1-y2) ** 2))
    
    # Calculate angle between new point and nearest node
    def angle(self,x1,y1,x2,y2):
        return math.atan2(y2-y1, x2-x1)
        
    # return the index of point among the points in node_list,
    # the one that closet to point (x,y)
    def nearest_node(self, x, y, node_list):
        distances = [self.distance(x, y, node.x, node.y) 
                     for node in node_list]
        return distances.index(min(distances))
    
    # check collision
    def collision(self, x1, y1, x2, y2, img):
        color = []
        print("check collision between x:", x1, x2)
    
        if int(x1) == int(x2) and int(y1) == int(y2):
            # Points are the same, no collision
            print("No collision")
            return False

        line_points = np.column_stack((np.linspace(x1, x2, num=100), np.linspace(y1, y2, num=100)))

        for point in line_points:
            x, y = map(int, point)
            color.append(img[y, x])

        # If there is black in color => collision
        if 0 in color:
            return True
        else:
            return False
    
    # check the collision with obstacle and the trajectory
    # if there is no collision, add the newpoint to the tree
    def expansion(self, random_x ,random_y ,nearest_x ,nearest_y , offset, map):
    
        theta = self.angle(nearest_x, nearest_y, random_x, random_y)
        new_node_x = nearest_x + offset * np.cos(theta)
        new_node_y = nearest_y + offset * np.sin(theta)
        width, height = map.shape

        # Check the point is out of bounds or not
        if new_node_y < 0 or new_node_y > width or new_node_x < 0 or new_node_x > height:
            # print("Points out of bounds")
            expand_connect = False
        else:
            if self.collision(new_node_x,new_node_y,nearest_x,nearest_y,map):
                expand_connect = False
            else:
                expand_connect = True
        return(new_node_x, new_node_y , expand_connect)

    # find the neareast point to from 
    # newest point in Tree_A to Tree_B
    def extend(self, Tree_A, Tree_B, map):
        # index -1 gets the last object in the list
        x = Tree_A[-1].x
        y = Tree_A[-1].y
        nearest_index_b = self.nearest_node(x,y,Tree_B)
        nearest_x = Tree_B[nearest_index_b].x
        nearest_y = Tree_B[nearest_index_b].y
        # check direct connection
        if self.collision(x,y,nearest_x,nearest_y,map):
            extend_connect = False
        else:
            extend_connect =True

        # return(directCon,nearest_index_a,nearest_index_b)
        return(extend_connect,nearest_index_b)

    def rrt_path(self, target):
        # object Target
        if(target != " "):
            self.set_target(target)
            print("Sucessfully find the target.")
        # point Target
        else:
            print("No object target.")

        height, width = self.dilation.shape

        # record the initial nodes       
        self.start_nodes[0] = Nodes(self.start[0], self.start[1])
        self.start_nodes[0].x_parent.append(self.start[0])
        self.start_nodes[0].y_parent.append(self.start[1])
        self.goal_nodes[0] = Nodes(self.goal[0],self.goal[1])
        self.goal_nodes[0].x_parent.append(self.goal[0])
        self.goal_nodes[0].y_parent.append(self.goal[1])

        grow_tree = -1
        # grow_tree = -1 : spread tree from start to goal
        # grow_tree = 1 : spread tree from goal to start
        # Tree from A to B
        i = 1
        while i < self.iteration:
            if grow_tree == -1:
                Tree_A = self.start_nodes.copy()
                Tree_B = self.goal_nodes.copy()
            else:
                Tree_A = self.goal_nodes.copy()
                Tree_B = self.start_nodes.copy()

            # generate random points
            random_x, random_y = self.random_point(height, width)

            
            nearest_index = self.nearest_node(random_x,random_y,Tree_A)
            nearest_x, nearest_y = Tree_A[nearest_index].x, Tree_A[nearest_index].y
            new_node_x,new_node_y,expand_connect = self.expansion(random_x,random_y,
                                           nearest_x,nearest_y,
                                           self.offset,self.dilation)
            
            if expand_connect:
                # Add the point to the Tree
                Tree_A.append(Nodes(new_node_x,new_node_y))
                Tree_A[i].x_parent = Tree_A[nearest_index].x_parent.copy()
                Tree_A[i].y_parent = Tree_A[nearest_index].y_parent.copy()
                Tree_A[i].x_parent.append(new_node_x)
                Tree_A[i].y_parent.append(new_node_y)

                # display
                cv2.circle(self.map, (int(new_node_x),int(new_node_y)), 2,
                           (0,0,255), thickness=3, lineType=8)
                cv2.line(self.map, (int(new_node_x),int(new_node_y)), 
                         (int(Tree_A[nearest_index].x),
                          int(Tree_A[nearest_index].y)), 
                          (0,255,0), thickness=1, lineType=8)
                cv2.imwrite("bi-RRT_Path/"+str(i)+".jpg",self.map)
                cv2.imshow("image",self.map)
                cv2.waitKey(1)
                
                # if extend successful connect
                # which means the two Trees are succesfully connected
                extend_connect, index = self.extend(Tree_A,Tree_B,self.dilation)

                if extend_connect :
                    print("bi-RRT Path is successfully formulated")
                    path = []
                    cv2.line(self.map, 
                             (int(new_node_x),int(new_node_y)), 
                             (int(Tree_B[index].x),int(Tree_B[index].y)), 
                             (0,255,0), thickness=1, lineType=8)
                    if grow_tree == -1:
                        for i in range(len(Tree_A[-1].x_parent)):
                            path.append((Tree_A[-1].x_parent[i],Tree_A[-1].y_parent[i]))
                            pass
                        Tree_B[index].x_parent.reverse()
                        Tree_B[index].y_parent.reverse()    
                        for i in range(len(Tree_B[index].x_parent)):
                            path.append((Tree_B[index].x_parent[i],Tree_B[index].y_parent[i]))
                    else:
                        for i in range(len(Tree_B[index].x_parent)):
                            path.append((Tree_B[index].x_parent[i],Tree_B[index].y_parent[i]))
                            pass
                        Tree_A[-1].x_parent.reverse()
                        Tree_A[-1].y_parent.reverse()
                        for i in range(len(Tree_A[-1].x_parent)):
                            path.append((Tree_A[-1].x_parent[i],Tree_A[-1].y_parent[i]))
                    for i in range(len(path)-1):
                        cv2.line(self.map, (int(path[i][0]),int(path[i][1])), (int(path[i+1][0]),int(path[i+1][1])), (255,0,0), thickness=2, lineType=8)
                    cv2.waitKey(1)
                    cv2.imwrite("bi-RRT_Path/"+str(i)+".jpg",self.map)
                    if(self.target == " "):
                        cv2.imwrite("birrt.jpg",self.map)
                    else:
                        cv2.imwrite("bi-RRT_Path/"+self.target+".jpg",self.map)
                    break
            else:
                continue

            if grow_tree == -1:
                grow_tree = grow_tree*(-1)
                self.start_nodes = Tree_A.copy()
                self.goal_nodes = Tree_B.copy()
            else:
                grow_tree = grow_tree*(-1)
                i = i+1
                self.start_nodes = Tree_B.copy()
                self.goal_nodes = Tree_A.copy()

        if i==self.iteration:
            print("Failed to find the path")
        print("Number of iteration: ",i*2)

        path = np.asarray(path)
        width_transform = width / 17
        height_transform = height / 11
        path[:,0] = path[:,0] / width_transform - 6
        path[:,1] = 7 - path[:,1] / height_transform
        return path

# click on the picture to get the point
def draw_circle(event,u,v,flags,param):
    global coordinates
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(rrt.map,(u,v),5,(255,0,0),-1)
        coordinates.append(u)
        coordinates.append(v)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description = 'Parameters:')
    parser.add_argument('-p', type=str, default='map.png',metavar='ImagePath', 
                        action='store', dest='imagePath',
                        help='File path of the map image')
    parser.add_argument('-o', type=int, default=40, metavar='offset', 
                        action='store', dest='offset',
                        help='Step size in RRT algrithm')
    parser.add_argument('-f', type=str, default=" ",metavar='END', 
                        action='store', dest='End',
                        help='The target object of the Navigation')
    args = parser.parse_args()

    # remove previously stored data
    try:
      os.system("rm -rf bi-RRT_Path")
    except:
      print("Directory is not exist")
    os.mkdir("bi-RRT_Path")

    target = args.End
    rrt = RRTree(args.imagePath, args.offset, 10000)

    coordinates=[]
    print("Set the start point and goal point by double click on the image")
    print("Or press ESC to exit")
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', draw_circle)
    while(1):
        cv2.imshow('image',rrt.map)
        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            break
    rrt.start=(coordinates[0],coordinates[1])
    rrt.goal=(coordinates[2],coordinates[3])
    path = rrt.rrt_path(target)
    np.save('path', path)
