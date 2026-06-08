# Epoch of GNSS raw observations
std_msgs/Header header
uint16 gps_week
float64 gps_tow          # seconds, time of week
GnssRawObservation[] observations
uint8 num_satellites
float32 pdop
float32 hdop
float32 vdop
