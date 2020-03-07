# To be use as OBSTACLE detected in a laser.

bool single_layer_sensor       # True = single layer sensor (i.e: SICK) - False = multiple layer (i.e: IBEO)

int32 id                        # number of detection in scan (-1 if unknown)
string detection_frame_id       # frame ID of the sensor from which the detection comes.

bool dynamic                    # 1 = dynamic object; 0 = not (means that has been associated)
int32 num_points                # number of laser points contained
int32 index_first               # index of the range vector of the first point used? or filteredVector?
int32 index_last                # index of the range vector of the first point used? or filteredVector?

Point point_first               # first point of the cluster
Point point_last                # first point of the cluster

Origin obj_center               # Center point of the object bounding box and orientation
Point obj_size                  # X and Y (width and Height) of the object bounding box
Polyline[] polylines            # Set of points defining the Rectangle of the Bounding Box

Point ref_actual                # point of reference at the moment
Point ref_previous              # previous point of reference (in previous detection)
Point ref_centroid              # reference point to the centroid of the cluster
int32 ref_type                  # reference type in the pre-asso: 8 = corner; 4/2 = line; 1 = centroid;


# Line[] lines                    # Vector of Lines in the cluster
# Corners[] corners               # Vector of Corners in the cluster
