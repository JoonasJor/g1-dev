def num_to_range(num, in_min, in_max, out_min, out_max, clamp=True):
    val = out_min + (float(num - in_min) / float(in_max - in_min) * (out_max - out_min))
    if clamp:
        val = max(min(val, max(out_min, out_max)), min(out_min, out_max))
    return val