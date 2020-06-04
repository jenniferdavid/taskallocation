roscd navigation_stage/src
player multi_robot_task_5.cfg (player config of the particular problem)
roslaunch navigation_stage maze_5robots_5tasks.launch (navigation stack for that problem)
rosrun navigation_stage stage_robots_node _numRobots:=5 _numTasks:=5  //starts the player connection and publishes /coords
rosrun navigation_stage dist_cost // publishes /DeltaMat - the euclidean distance between all models OR
rosrun navigation_stage path_cost // publishes /DeltaMat - the path distance between all models
rosrun navigation_stage task_allocation_node // computes the task allocation to the robots using one of the 3 methods
rosrun navigation_stage send_goals_node //send robots to the respective positions

List of maps:
1. maze
2. willow-garage
3. oneroom
4. robopark
5. tworooms
6. cobotLab
7. ridgeback_race
8. samplemap
9. circlemaze
10. playpen
11. playground
12. maze2

 name "willow"
  #bitmap "../maps/willow-full-sparse.pgm"
  #size [58.400 52.600 0.500] #for willow.pgm
  #pose [ -26.300 29.200 0.000 90.000 ] #for willow.pgm

  bitmap "../maps/maze.pgm"
  size [60.00 60.00 0.500]
  pose [ -30.00 30.00 0.000 90.000 ]

  #bitmap "../maps/twoRooms1.pgm"
  #size [50.000 50.600 0.500]#for two_Rooms1.pgm
  #pose [ 0.00 9.00 0.000 90.000 ] #translation not proper in rviz - to be fixed

  #bitmap "../maps/my_map.pgm"
  #size [105.200 175.200 0.500]#for my_map.pgm
  #pose [ -43.80 28.80 0.000 90.000 ]

  #bitmap "../maps/robopark2.pgm"
  #size [140.6 100.00 00.00]  #for robo_park.pgm
  #pose [ -45.00 64.58 0.000 90.00] #translation not proper in rviz - to be fixed
