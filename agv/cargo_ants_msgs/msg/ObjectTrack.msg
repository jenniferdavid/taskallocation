# UNDER CONSTRUCTION

int32 type												# Type of target tracked:
                        				  # type equal to 1 for Mobile target (Only go out of the ros node the dynamic tracks)
                         					# type equal to 2 for static target
geometry_msgs/Point position			# Target (x,y) position
geometry_msgs/Point velocity   		# Target (Vx,Vy) velocity
float32[36] covariances						# Target covariance in position, velocity and cross covariance.
int32 id													# Target identifier
float32 probability								# Target probability [0-1]. When close to 1, the target is more probale that is its tracked object.
geometry_msgs/Point object_center # Target center (x,y) position
geometry_msgs/Point object_size 	# Target size (x,y) 
float32 object_orientation 				# Target object orientation 
