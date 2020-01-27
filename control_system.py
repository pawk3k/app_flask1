import matplotlib.pyplot as plt
from utils import *
import math

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.e_sum = 0
        self.last_e = 0

    def __call__(self, e, dt):
        self.e_sum += e*dt
        v = self.Kp*e + self.Ki*self.e_sum + self.Kd * (e - self.last_e) / dt
        self.last_e = e
        return v

    def reset(self):
        self.e_sum = 0
        self.last_e = 0

class Model:
    def __init__(self, pid, transform):
        self.pid=pid
        self.trans=transform

    def feed(self, e, dt, f_max):
        return self.trans(self.pid(e, dt), f_max)
    
    def reset(self):
        self.pid.reset()

def exp_reach(alpha):
    def fun(v, v_max):
        v = max(v, 0)
        if v < 100:
            return (1-math.exp(-v * alpha))*v_max
        return v_max
    return fun

def tanh_approx():
    def fun(v, v_max):
        return 0.5*(math.tanh(0.5*(v-0.5))+1)*v_max
    return fun


m1 = Model(PID(1, 0.1, 0.9), exp_reach(2.3))#tanh_approx())
m11 = Model(PID(1, 0.1, 0.9), lambda x, xm: clamp(x, 0, xm))
m2 = Model(PID(50, 0.05, 50), lambda x, xm: clamp(x+750*9.81, 0, xm))

def control_system(t0, tmax, dt=0.01, m=750, b=180.9, y0=0, v0=0, target=100, f_max=10000, ref_fun=rise(3.5), fuel_loss=0):
    g = 9.81
    model = Model(PID(50, 0.05, 50), lambda x, xm: clamp(x+m*g, 0, xm))
    model.reset()
    ref_fun = prepare(ref_fun, t0, y0, target)
    m_min = 100

    y1 = y0 - v0 * dt
    ts, ys, rhs, ls = [], [], [], []
    t = min(t0, 0)
    y = y0
    r_h = ref_fun(t)
    n = int((tmax-t) // dt) + 1
    l = 0
    for _ in range(n):
        if t >= t0:
            ts += [t]
            ys += [y]
            rhs += [r_h]
            ls += [l]
        t += dt
        if m > m_min:
            m -= fuel_loss * dt
        r_h = ref_fun(t)

        e = r_h - y0
        l = model.feed(e, dt, f_max)

        y = (m*(2*y0 - y1) + (l-m*g)*(dt**2) - b*abs(y0-y1)*(y0-y1)) / m  # drag b is bidirectional
        y = max(y, 0)
        y1 = y0
        y0 = y
    return ts, ys, rhs

import sys
def plot(t0, tmax, **kwargs):
    ts, ys, rhs = control_system(t0, tmax, **kwargs)
    plt.title("Wysokość balonu w funkcji czasu")
    plt.xlabel("t [s]")
    plt.ylabel("y [m]")
    plt.plot(ts, rhs, ":", label="wysokość optymalna")
    plt.plot(ts, ys, label="wysokość rzeczywista")
    #plt.plot(ts, ls)
    plt.legend()

if __name__=="__main__":
    plot(-5, 200, y0=0, ref_fun=levels(1, [(0, 10), (40, 20), (120, 5), (160, 15)])) #ref_fun=rise(3.5))
    plt.show()
