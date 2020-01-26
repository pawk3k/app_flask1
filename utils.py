def clamp(val, a, b):
    return min(max(val, a), b)

def steady_rise(speed, t0, t, h0, h1):
    sign = 1 if h1 > h0 else -1
    rise_time = sign * (h1-h0)/speed
    return (h0 + sign*speed*t if t < rise_time else h1) if t > 0 else h0

def rise(speed):
    def fun(t0, t, h0, h1):
        return steady_rise(speed, t0, t, h0, h1)
    return fun

def step(_, t, h0, h1, t_step=0):
    return h0 if t < t_step else h1

def step_at(t_step):
    def fun(_, t, h0, h1):
        return step(_, t, h0, h1, t_step)
    return fun
    
def prepare(fun, t0, h0, h1):
    def fun2(t):
        return fun(t0, t, h0, h1)
    return fun2

def save(t0, tmax, **kwargs):
    ts, ys, rhs, fs, vs = control_system(t0, tmax, **kwargs)
    with open("out.txt", "w") as file:
        for x in zip(ts, rhs, ys):
            file.write("\t".join([str(round(i,2)) for i in x]) + "\n")

def levels(speed, lv):
    def fun(t0, t, h0, h1):
        if t < lv[0][0]:
            return 0
        for i in range(len(lv)):
            if i == len(lv)-1 or lv[i][0] <= t < lv[i+1][0]:
                last = lv[i-1] if i > 0 else (0, 0)
                return steady_rise(speed, 0, t-lv[i][0], last[1], lv[i][1])
        return 0
    return fun

# a, h1; b, h2
# 0-a: 0, a-b ^h1, b- ^h2