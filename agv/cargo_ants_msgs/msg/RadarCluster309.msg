Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis

uint16 sensorId                         
uint8 id			# target number
float32 dist		# target range	[m]
float32 dist_rms	# target range standard deviation [m]
float32 angle		# target angle	[deg]
float32 angle_rms	# target angle standard deviation [deg]
uint8 angle_stat	# target angle status: 0-expanded target, 1-point target, 2-digital, 3-invalid
float32 vrel		# target relative velocity [m/s]
float32 vrel_rms	# target relative velocity standard deviation [m/s]
uint8 pdh0			# target false alarm probability [%]
float32 length		# target length	[m]
float32 width		# target width	[m]
uint8 type			# target type: 0-no target, 1-oncoming, 2-stationary, 3-traced (same direction)
float32 rcs			# radar cross-section [dBm^2]
