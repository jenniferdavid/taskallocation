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
