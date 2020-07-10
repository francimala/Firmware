# DWM1001 message

uint64 timestamp			# time since system start (microseconds)
float32[98] distances			# Distances between anchors and tag
float32[4] positions			# Position of the TAG with respect to the anchors (x,y,z,Quality)
uint16 anchor_num			# Number of anchors
