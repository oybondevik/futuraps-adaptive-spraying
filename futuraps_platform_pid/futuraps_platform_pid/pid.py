import math


class PID:
    def __init__(self, kp, ki, kd, i_max=float("inf")):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_max = abs(i_max)
        self.i = 0.0
        self.prev_e = None

    def reset(self):
        self.i = 0.0
        self.prev_e = None

    def step(self, e, dt):
        if dt <= 0:
            raise ValueError("dt must be positive")
        self.i += e * dt
        self.i = max(min(self.i, self.i_max), -self.i_max)

        de = 0.0 if self.prev_e is None else (e - self.prev_e) / dt
        self.prev_e = e
        return self.kp * e + self.ki * self.i + self.kd * de


def clamp(x, lo, hi):
    return max(lo, min(x, hi))


def wrap_to_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a
