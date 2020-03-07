# This message contains the vehicle state data for Cargo-ANTs platforms (AGV and AT)
# Values are estimates, computed by fusion of different sensor modalities, mainly on-board the platform. 
# Covariance matrix is also an estimate, providing the degree of uncertainty about such estimate values.

#timestamp and frame id of this vehicle state data
Header header

#vehicle location (x,y,z), with respect to some origin frame located in the terminal yard [m]
geometry_msgs/Point location

#vehicle orientation expressed in quaternion form (qi,qj,qk,qR), with respect to some origin frame located in the terminal yard [.]
geometry_msgs/Quaternion orientation

#vehicle linear velocity, with respect to some origin frame located in the terminal yard [m/s]
geometry_msgs/Point velocity

#vehicle rotational rate, with respect to vehicle frame (X front, Y left, Z up) [rad/s]
geometry_msgs/Point rot_rate

#imu accelerometer bias [m/s2]
geometry_msgs/Point acc_bias

#imu gyro bias [rad/s]
geometry_msgs/Point gyro_bias

#gravity at origin [m/s2]
geometry_msgs/Point gravity

# Covariance matrix of the above estimates (except bias and gravity) ordered as they appear in the message.
# Vector size is 13x13=169
# Matrix is written by rows
float64[169] covariance
