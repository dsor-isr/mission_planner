# Linear calibration:    channel = (energy - offset) / slope
# Quadratic calibration: channel = (sqrt(slope * slope - 4 * square * (offset - energy)) - slope) / (2 * square)
float32 slope
float32 offset
float32 square
