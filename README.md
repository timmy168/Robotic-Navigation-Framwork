# A Robot Navigation Framework
This work based on a semantic map to plan the path by RRT algorithm, navigate the agent to the desired taret object.

# Preparation
(1) Habitat Simulation Environment

# 2D semantic map construction
(1) Prepair a point cloud data with a .npy file for points data, and a .npy file for colors data.
(2) Set up the file path of the data.
(3) Run 2D_semantic_map.py to generate 2D semantic map
(4) 2D semantic map results:
![image](https://github.com/timmy168/Robotic-Navigation-Framwork/blob/main/map.png)

# RRT Algorithm
(1) With the generated 2D semantic map, run rrt.py or bi-rrt.py to generate path.
(2) If you want to plan the path between two random points, run the code and double click on the image, the first point will be the starting point, and the second point will be the goal.
(3) If you want to set the goal to specific object, using the parser -f , the initial objects are "refrigerator", "rack", "cushion", "lamp", "stair", "cooktop". Same as step(2), double click on any where to be the starting point, then double click any where to start planning.
(4) The image of the map with planned path is save in the generated directory /RRT_path and /bi-RRT_path
(5) RRT path planning results
![image](https://github.com/timmy168/Robotic-Navigation-Framwork/blob/main/rrt.jpg)
(6) bi-RRT path planning results
![image](https://github.com/timmy168/Robotic-Navigation-Framwork/blob/main/birrt.jpg)

# Robot Navigation 
(1) run navigation.py to automatically navigate the agent in the habitat simulation environment.
(2) If you want to set the goal to specific object, using the parser -f , the initial objects are "refrigerator", "rack", "cushion", "lamp", "stair", "cooktop".
(3) After navigation, the navigating video is saved in generated directory /video
