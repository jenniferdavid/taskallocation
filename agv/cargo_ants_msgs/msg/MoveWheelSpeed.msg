Header header            # timestamp in the header is the acquisition time of
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis

uint8 sensorId    # id of sensor

float32 vx_wheel_fl     # vehicle speed - front left wheel
float32 vx_wheel_fr     # vehicle speed - front right wheel
float32 vx_wheel_rl     # vehicle speed - rear left wheel
float32 vx_wheel_rr     # vehicle speed - rear right wheel
