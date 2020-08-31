# DWM1001 message

uint64 timestamp			# time since system start (microseconds)
float32[98] distances			# Distances between anchors and tag, each anchor provides 4 measurements (x,y,z coordinate of the anchor with respect to a specific reference frame + distance between the anchor and the tag).
float32[4] positions			# Position of the TAG into the reference system defined by the anchors (x,y,z,Quality)
uint16 anchor_num			# Number of anchors
bool pos_detected			# Is POS present into the payload?
