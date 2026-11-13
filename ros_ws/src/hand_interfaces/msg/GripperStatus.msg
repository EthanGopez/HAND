# Mirrors the claw firmware's STATUS line: "state=... pulse_us=... force_mv=..."
std_msgs/Header header
string state                       # STOPPED, OPENING, CLOSING, OPEN, CLOSED, CONTACT or FAULT
int32 pulse_us                     # servo pulse width
int32 force_mv                     # raw force-sensor reading in millivolts (not calibrated)
