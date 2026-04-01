# Single GNSS raw observation per satellite/signal
uint8 gnss_system       # GNSSSystem enum (1=GPS, 2=GLO, 4=GAL, 8=BDS, 16=QZS)
uint8 prn               # Satellite PRN number
uint8 signal_type       # SignalType enum
float64 pseudorange     # meters
float64 carrier_phase   # cycles
float64 doppler         # Hz
float32 snr             # dB-Hz (carrier-to-noise ratio)
uint8 lock_time_indicator
bool valid
