std_msgs/Header header

uint8 DEVICE_TYPE_NONE=0
uint8 DEVICE_TYPE_MCA166=1
uint8 DEVICE_TYPE_MCA527_FULL=2
uint8 DEVICE_TYPE_MCA527_LITE=3
uint8 DEVICE_TYPE_MCA527_OEM=4
uint8 DEVICE_TYPE_MCA527_MICRO=5
uint8 DEVICE_TYPE_MCA527_NANO=6
uint8 device_type

uint16 hardware_version
uint16 firmware_version
uint16 serial_number

uint8 PRESET_NONE=0
uint8 PRESET_REAL=1
uint8 PRESET_LIVE=2
uint8 PRESET_INT=3
uint8 PRESET_AREA=4
uint8 preset

uint32 preset_time

float32 threshold

uint16 lld  # lower level discriminator channel
uint16 uld  # upper level discriminator channel

uint16 coarse_gain
float64 fine_gain

# Detector bias in volts, that sets the high voltage
int32 detector_bias

uint16 pzc
