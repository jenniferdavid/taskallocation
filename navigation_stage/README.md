roscd navigation_stage/src
player multi_robot_task.cfg
roslaunch navigation_stage main_multi.launch 
rosrun navigation_stage stage_robot //starts the player connection and publishes /coords
rosrun navigation_stage dist_cost // publishes /DeltaMat - the euclidean distance between all models
rosrun navigation_stage path_cost // publishes /DeltaMat - the path distance between all models
rosrun navigation_stage heuristic // computes the approximation method based task allocation to the robots.

List of maps:
1. maze
2. willow-garage
3. oneRoom
4. robopark
5. twoRooms
6. cobotLab
7. ridgeback_race
8. hospital (to do)
9. circlemaze
10. playpen
11. playground

