# Combination of both types of radar for fusing detections
# UNDER CONSTRUCTION


Header header            # timestamp in the header is the acquisition time of
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis

uint16 sensorId
uint16 obj_id		     # unique object id

float32 longdisp		 # longitudinal displacement	[m]
float32 latdisp			 # lateral displacement [m]
float32 vrellong		 # [309 - (float32 relxdot)] relative longitudinal velocity [m/s]

uint8 rollcount			 # sequence index
float32 rcs			     # radar cross-section [dBm^2]




# Next variables are not common...merge all and define negative value if nonexisting?
# Pros: if object is detected from both type of sensors, we collect more information
# Cons: longer messages... 

#### --------   309

#uint8 length		    # object length: 0-unknown, 1-<0.5m, 2-<2m, 3-<4m, 4-<6m, 5-<10m, 6-<20m, 7-exceeds 20m
#uint8 width				# object width: 0-unknown, 1-<0.5m (pedestrian), 2-<1m (bike), 3-<2m (car),
                        #			4-<3m (truck), 5-<4m, 6-<6m, 7-exceeds 6m
#uint8 measstat			# measurement status: 0-no object, 1-new object, 2-object not measured, 3-object measured
#uint8 existprob			# probability of existance: 0-invalid, 1-<25%, 2-<50%, 3-<75%,
                        #			4-<90%, 5-<99%, 6-<99.9%, 7-<99.99%
#uint8 dynprop			# dynamic property: 0-unclassified, 1-standing, 2-stopped, 3-moving, 4-oncoming

#float32 relxddot		# relative longitudinal acceleration [m/s^2]
#float32 latspeed		# lateral velocity [m/s], value negative->object moves left to right
#uint8 obsprob			# probabilty that the object is an obstacle [%]


#### --------   209

#uint8 index				# index of object in the list

#float32 vrellat			# relative lateral velocity	[m/s]

#float32 lifetime		    # lifetime of track [s]

