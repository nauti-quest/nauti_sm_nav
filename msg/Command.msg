# This represents a control message to the LoCO AUV.

Header header
float64 roll 
float64 pitch # Not in use
float64 yaw # -1.0 to 1.0, full thrust yaw left to 0 to full thrust yaw right.
float64 throttle # -1.0 to 1.0, full thrust reverse to 0 to full thrust forward.
float64 heave