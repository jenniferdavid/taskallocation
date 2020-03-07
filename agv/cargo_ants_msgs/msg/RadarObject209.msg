Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis

uint16 sensorId
uint16 obj_id			# unique object id
uint8 index				# index of object in the list
float32 longdisp		# longitudinal displacement	[m]
float32 latdisp			# lateral displacement [m]
float32 vrellong		# relative longitudinal velocity [m/s]
float32 vrellat			# relative lateral velocity	[m/s]
uint8 rollcount			# sequence index
float32 rcs				# radar cross-section [dBm^2]
float32 lifetime		# lifetime of track [s]
                        
