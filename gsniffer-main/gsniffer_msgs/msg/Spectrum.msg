std_msgs/Header header

uint32 first_channel
uint32 last_channel
uint32[] channels

duration life_time
duration real_time

duration flat_top_time

float64 high_voltage  # power high voltage reading

uint64 counts                # integral counts
uint64 peak_detector_counts  # integral peak detector counts

uint64 integral  # sample integral, computed as the summation of all channels from first_channel to last_channel
float64 cps      # counts per second, computed by dividing the sample integral by the sample period or the life or real
                 # time depending on the configuration

duration dead_time
duration busy_time

float32 detector_temperature
float32 mca_temperature

# Add more fields based on gbs_mca_data.h if needed
