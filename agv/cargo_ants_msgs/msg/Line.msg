# UNDER CONSTRUCTION

float32[] line_parameters       # homogeneous parameterization of the line: (a,b,c)^T -> ax+by+c=0
float32 error                   # sum of all distances from used points to line

int32 index_first               # index of the range vector of the first point used? or filteredVector?
int32 index_last                # index of the range vector of the first point used? or filteredVector?

Point point_first               # first point of the line
Point point_last                # first point of the line

int32 number_points             # Number of laser points contained in the laser
