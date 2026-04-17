# HiwonderSDK/PID.py

class PID:
    def __init__(self, P=0, I=0, D=0):
        print(f"[FAKE PID] Created PID(P={P}, I={I}, D={D})")
        self.P = P
        self.I = I
        self.D = D
        self.SetPoint = 0
        self.output = 0

    def clear(self):
        print("[FAKE PID] clear() called")
        self.output = 0

    def update(self, value):
        print(f"[FAKE PID] update(value={value})")
        # Fake output: small proportional response
        self.output = (value - self.SetPoint) * 0.1
