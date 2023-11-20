import numpy as np
import argparse
import os
import random
import math
import cv2

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
        dilation = cv2.erode(img, np.ones((3,3), np.uint8), iterations=12)
        self.dilation = dilation 
        self.map = cv2.imread(map) 
        self.iteration = iteration
        self.start = [0, 0]
        self.goal = [0, 0]
        self.offset = offset
        self.start_nodes = [()]
        self.target = " "

    def set_target(self, target):
        object_positions = {"refrigerator": (455, 457),
                            "lamp": (894, 657),
                            "cushion": (1039, 454),
                            "rack": (748, 284),
                            "cooktop": (380, 545)}
        x, y = object_positions[target]
        self.target = target
        self.goal = (x, y)

    def random_point(self, height, width):
        random_x = random.randint(0, width)
        random_y = random.randint(0, height)
        return (random_x, random_y)

    def distance(self, x1, y1, x2, y2):
        return math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))

    def angle(self, x1, y1, x2, y2):
        return math.atan2(y2 - y1, x2 - x1)

    def nearest_node(self, x, y, node_list):
        distances = [self.distance(x, y, node.x, node.y) for node in node_list]
        return distances.index(min(distances))

    def collision(self, x1, y1, x2, y2, img):
        color = []
        if int(x1) == int(x2) and int(y1) == int(y2):
            print("No collision")
            return False

        line_points = np.column_stack((np.linspace(x1, x2, num=100), np.linspace(y1, y2, num=100)))

        for point in line_points:
            x, y = map(int, point)
            color.append(img[y, x])

        if 0 in color:
            return True
        else:
            return False

    def expansion(self, random_x, random_y, nearest_x, nearest_y, offset, map):
        theta = self.angle(nearest_x, nearest_y, random_x, random_y)
        new_node_x = nearest_x + offset * np.cos(theta)
        new_node_y = nearest_y + offset * np.sin(theta)
        width, height = map.shape

        if new_node_y < 0 or new_node_y > width or new_node_x < 0 or new_node_x > height:
            expand_connect = False
        else:
            if self.collision(new_node_x, new_node_y, nearest_x, nearest_y, map):
                expand_connect = False
            else:
                expand_connect = True
        return (new_node_x, new_node_y, expand_connect)

    def extend(self, Tree, goal_node, map):
        x = Tree[-1].x
        y = Tree[-1].y
        nearest_index_b = self.nearest_node(x, y, goal_node)
        nearest_x = goal_node[nearest_index_b].x
        nearest_y = goal_node[nearest_index_b].y

        if self.collision(x, y, nearest_x, nearest_y, map):
            extend_connect = False
        else:
            extend_connect = True

        return (extend_connect, nearest_index_b)

    def rrt_path(self, target):
        if target != " ":
            self.set_target(target)
            print("Successfully find the target.")
        else:
            print("No object target.")

        height, width = self.dilation.shape

        self.start_nodes[0] = Nodes(self.start[0], self.start[1])
        self.start_nodes[0].x_parent.append(self.start[0])
        self.start_nodes[0].y_parent.append(self.start[1])

        i = 1
        while i < self.iteration:
            Tree_A = self.start_nodes.copy()
            random_x, random_y = self.random_point(height, width)

            nearest_index = self.nearest_node(random_x, random_y, Tree_A)
            nearest_x, nearest_y = Tree_A[nearest_index].x, Tree_A[nearest_index].y
            new_node_x, new_node_y, expand_connect = self.expansion(random_x, random_y,
                                                                    nearest_x, nearest_y,
                                                                    self.offset, self.dilation)

            if expand_connect:
                Tree_A.append(Nodes(new_node_x, new_node_y))
                Tree_A[i].x_parent = Tree_A[nearest_index].x_parent.copy()
                Tree_A[i].y_parent = Tree_A[nearest_index].y_parent.copy()
                Tree_A[i].x_parent.append(new_node_x)
                Tree_A[i].y_parent.append(new_node_y)

                cv2.circle(self.map, (int(new_node_x), int(new_node_y)), 2,
                           (0, 0, 255), thickness=3, lineType=8)
                cv2.line(self.map, (int(new_node_x), int(new_node_y)),
                         (int(Tree_A[nearest_index].x), int(Tree_A[nearest_index].y)),
                         (0, 255, 0), thickness=1, lineType=8)
                cv2.imwrite("RRT_Path/" + str(i) + ".jpg", self.map)
                cv2.imshow("image", self.map)
                cv2.waitKey(1)

                extend_connect, index = self.extend(Tree_A, [Nodes(self.goal[0], self.goal[1])], self.dilation)

                if extend_connect:
                    print("Path is successfully formulated")
                    path = []
                    cv2.line(self.map, (int(new_node_x), int(new_node_y)),
                             (int(self.goal[0]), int(self.goal[1])), (0, 255, 0), thickness=1, lineType=8)

                    for i in range(len(Tree_A[-1].x_parent)):
                        path.append((Tree_A[-1].x_parent[i], Tree_A[-1].y_parent[i]))

                    Nodes(self.goal[0], self.goal[1]).x_parent.reverse()
                    Nodes(self.goal[0], self.goal[1]).y_parent.reverse()

                    for i in range(len(Nodes(self.goal[0], self.goal[1]).x_parent)):
                        path.append((Nodes(self.goal[0], self.goal[1]).x_parent[i],
                                     Nodes(self.goal[0], self.goal[1]).y_parent[i]))

                    # Draw the last segment with a different color (red)
                    cv2.line(self.map, (int(Tree_A[-1].x_parent[-1]), int(Tree_A[-1].y_parent[-1])),
                             (int(self.goal[0]), int(self.goal[1])), (255, 0, 0), thickness=2, lineType=8)

                    for i in range(len(path) - 1):
                        cv2.line(self.map, (int(path[i][0]), int(path[i][1])),
                                 (int(path[i + 1][0]), int(path[i + 1][1])), (255, 0, 0), thickness=2, lineType=8)

                    cv2.waitKey(1)
                    cv2.imwrite("RRT_Path/" + str(i) + ".jpg", self.map)
                    if self.target == " ":
                        cv2.imwrite("rrt.jpg", self.map)
                    else:
                        cv2.imwrite("RRT_Path/" + self.target + ".jpg", self.map)
                    break
            else:
                continue

            i = i + 1
            self.start_nodes = Tree_A.copy()

        if i == self.iteration:
            print("Failed to find the path")
        print("Number of iteration: ", i)

        path = np.asarray(path)
        width_transform = width / 17
        height_transform = height / 11
        path[:, 0] = path[:, 0] / width_transform - 6
        path[:, 1] = 7 - path[:, 1] / height_transform
        return path

# click on the picture to get the point
def draw_circle(event, u, v, flags, param):
    global coordinates
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(rrt.map, (u, v), 5, (255, 0, 0), -1)
        coordinates.append(u)
        coordinates.append(v)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Parameters:')
    parser.add_argument('-p', type=str, default='map.png', metavar='ImagePath',
                        action='store', dest='imagePath',
                        help='File path of the map image')
    parser.add_argument('-o', type=int, default=40, metavar='offset',
                        action='store', dest='offset',
                        help='Step size in RRT algorithm')
    parser.add_argument('-f', type=str, default=" ", metavar='END',
                        action='store', dest='End',
                        help='The target object of the Navigation')
    args = parser.parse_args()

    try:
        os.system("rm -rf RRT_Path")
    except:
        print("Directory is not exist")
    os.mkdir("RRT_Path")

    target = args.End
    rrt = RRTree(args.imagePath, args.offset, 10000)

    coordinates = []
    print("Set the start point and goal point by double click on the image")
    print("Or press ESC to exit")
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', draw_circle)
    while 1:
        cv2.imshow('image', rrt.map)
        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            break
    rrt.start = (coordinates[0], coordinates[1])
    rrt.goal = (coordinates[2], coordinates[3])
    path = rrt.rrt_path(target)
    np.save('path', path)
