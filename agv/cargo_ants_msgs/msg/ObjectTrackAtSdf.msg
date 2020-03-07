# Single object track from SDF UDP message (Volvo, AT)
# Result of internal datafusion made by Volvo AutoBox SW, which uses ibeo and radars


int32 id 			#track id
int32 class_id		#class id of the track:
int32 UNDETERMINED 		= 0
int32 CAR 				= 1
int32 MOTORCYCLE		= 2
int32 TRUCK 			= 3
int32 PEDESTRIAN 		= 4
int32 POLE				= 5
int32 TREE 				= 6
int32 ANIMAL			= 7
int32 GOD				= 8
int32 BICYLCE			= 9
int32 OTHERS			= 10  #unidentified vehicle, piano, speed limit sign, x-sign, ... 

geometry_msgs/Point position 	#Track position (x,y) [m]
geometry_msgs/Point velocity 	#Track velocity (vx,vy) [m/s]
float64 heading 				#Track heading [rad] 
float64[4] covariance			#XY covariance matrix [m2/s2]
