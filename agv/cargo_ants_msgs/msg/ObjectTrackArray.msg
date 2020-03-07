# UNDER CONSTRUCTION

Header header 			 # Timestamp, frame id
ObjectTrack[] detection  # Set of targets being tracked
int32 type				 # Type of target tracked:
                         # type equal to 1 for Mobile target (Only go out of the ros node the dynamic tracks)
                         # type equal to 2 for static target
						 # type equal to 0, means type inside each track.
