import numpy as np

# Test these properly later

def radian_to_scaled(val, min_rad, max_rad):
    """Convert radians to DDS [0, 1000] integer."""
    if max_rad - min_rad == 0:
        return 0
    scaled = int(np.clip(1000 * (val - min_rad) / (max_rad - min_rad), 0, 1000))
    return scaled

def scaled_to_radian(val, min_rad, max_rad):
    """Convert DDS [0, 1000] integer to radians."""
    scaled = min_rad + (max_rad - min_rad) * (val / 1000.0)
    #print(f"scaled_to_radian: {val=} {min_rad=} {max_rad=} -> {scaled=}")
    return scaled

def force_to_scaled(val, min_f, max_f):
    """Convert force to DDS [-4000, 4000] integer."""
    if max_f - min_f == 0:
        return 0
    scaled = int(np.clip(8000 * (val - min_f) / (max_f - min_f) - 4000, -4000, 4000))
    return scaled

def scaled_to_force(val, min_f, max_f):
    """Convert DDS [-4000, 4000] integer to force."""
    return min_f + (max_f - min_f) * ((val + 4000) / 8000.0)