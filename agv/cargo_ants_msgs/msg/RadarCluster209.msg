Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
uint16 sensorId                         
float32 azimut     # azimut [rad]
uint8 index         # track index
uint8 rollcount     # sequence index
float32 range      # range [m]
float32 rcs        # radar cross-section [dBm^2]
float32 vrel       # relative velocity [m/s]
