from flask import Response, Flask, render_template, request

import base64
import matplotlib.pyplot as plt
from random import *
import io
import json


from random import random

import matplotlib.pyplot as plt
import io
import json
from flask import Flask, render_template, request, Response


class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.e_sum = 0
        self.last_e = 0

    def __call__(self, e, dt):
        self.e_sum += e * dt
        v = self.Kp * e + self.Ki * self.e_sum + self.Kd * (e - self.last_e) / dt
        self.last_e = e
        return v


dt = 0.01
g = 9.81


def control_system(t0, tmax, m=750, b=180.9, y0=0, v0=0, target=100, v_opt=3.5, f_max=3000, use_r_h=True):
    global dt, g
    tmax = int(tmax)
    m = int(m)
    b = float(b)
    y0 = int(y0)
    v0 = int(v0)
    target = int(target)
    v_opt = float(v_opt)
    f_max = int(f_max)
    use_r_h = bool(use_r_h)
    n = int((tmax - t0) // dt)
    if use_r_h:
        r_h = y0
    else:
        r_h = target
        v_opt = 0.0

    rhs = [r_h, r_h + v_opt * dt]
    t = t0
    f_add = m * g
    t0, t1 = 0, dt
    y1 = y0 + v0 * dt
    ts, ys = [t0, t1], [y0, y1]
    fs = [0, 0]
    vs = [v0, v0]
    pid = PID(50, 0.05, 50)
    for _ in range(n):
        t += dt
        if use_r_h:
            if r_h < target:
                r_h += v_opt * dt
                if r_h >= target: use_r_h = False
            if r_h > target:
                r_h -= v_opt * dt
                if r_h <= target: use_r_h = False
        rhs += [r_h]
        e = r_h - y1
        f = pid(e, dt)
        l = max(min(f, f_max) + f_add, 0) - m * g
        y = (m * (2 * y1 - y0) + l * (dt ** 2) - b * abs(y1 - y0) * (y1 - y0)) / m  # drag b is bidirectional
        y = max(y, 0)
        ts += [t]
        ys += [y]
        y0 = y1
        y1 = y

    return ts, ys, rhs, fs, vs


def plot1(t0, tmax, **kwargs):
    fig, ax = plt.subplots( nrows=1, ncols=1 )
    ts, ys, rhs, fs, vs = control_system(t0, tmax, **kwargs)
    plt.title("Wysokość balonu w funkcji czasu")
    plt.xlabel("t [s]")
    plt.ylabel("y(t) [m]")
    plt.plot(ts, rhs, ":", label="wysokość optymalna")
    plt.plot(ts, ys, label="wysokość rzeczywista")
    plt.legend()
    buf = io.BytesIO()
    fig.savefig(buf, format='svg')
    return buf.getvalue().decode('utf-8')

def random_plot(n):
    fig, ax = plt.subplots(nrows=1, ncols=1)
    xy = [random() for _ in range(2 * n)]
    ax.plot(xy[:n], xy[n:])
    buf = io.BytesIO()
    fig.savefig(buf, format='svg')
    return buf.getvalue().decode('utf-8')


app = Flask(__name__)
app.config['FLASK_DEBUG'] = True



@app.route('/')
def main():
    return render_template('index.html')


@app.route('/data', methods=['POST'])
def mm():
    resp = json.loads(request.data.decode('utf-8'))
    # print(resp)
    # n = int(resp["npoints"])
    plot = plot1(0, **resp)
    return Response(plot)


if __name__ == "__main__":
    app.config['FLASK_DEBUG'] = True
    app.run(debug=True)

