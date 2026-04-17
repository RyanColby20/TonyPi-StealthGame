# HiwonderSDK/Misc.py

def map(x, in_min, in_max, out_min, out_max):
    print(f"[FAKE Misc] map({x}, {in_min}, {in_max}, {out_min}, {out_max})")
    if in_max - in_min == 0:
        return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
