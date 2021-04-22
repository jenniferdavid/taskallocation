# To execute

Run Navigation stack for that problem
````
roslaunch navigation_stage maze_5robots_5tasks.launch 
````
Starts the player connection and publishes /coords
````
rosrun navigation_stage stage_robots_node _numRobots:=5 _numTasks:=5  
````
Publishes /DeltaMat - the euclidean distance between all models OR
````
rosrun navigation_stage dist_cost 
````
Publishes /DeltaMat - the path distance between all models
````
rosrun navigation_stage path_cost 
````
Computes the task allocation to the robots using one of the 3 methods
````
rosrun navigation_stage task_allocation_node -heuristic
````
Send robots to the respective positions
````
rosrun navigation_stage send_goals_node 
````
