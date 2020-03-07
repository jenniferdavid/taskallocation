# A goal is specified as its (gx,gy) coordinate and heading gth (for
# "goal theta"). Additionally, dr is the radial tolerance, and dth the
# angular tolerance to be achieved. Specifying a negative number for
# either dr or dth disabled that particular tolerance. Thus, if you do
# no care about the heading with which you arrive at a goal, specify
# dth = -1 for example.  The goal message type is intended to be used
# as part of other messages such as Path.msg, thus we do not include
# the frame here as that should be the same for an entire array of
# goals.

float64 gx
float64 gy
float64 gth

# Radial accuracy of the goal. Specifying dr <= 0.0 means that you do
# not care about the goal position at all.
#
float64 dr

# Angular accuracy of the goal. Specifying dth <= 0.0 or >= M_PI means
# that orientation does not matter.
#
float64 dth


#later? bool go_forward
