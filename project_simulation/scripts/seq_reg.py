from matplotlib.pyplot import plot, show
from time import time
import numpy as np
import control_utils
import control

PLANT_NUMERATOR = [1]
PLANT_DENOMINATOR = [1, 0]


class PrevStep:
    y = 0
    t = time()


def make_step(sys, u, t):
    ti, yi = control.forced_response(sys, U=u, T=[PrevStep.t, t], X0=PrevStep.y * PLANT_DENOMINATOR[0])
    PrevStep.t = ti[-1]
    PrevStep.y = yi[-1]
    return PrevStep.t, PrevStep.y


def test_1():
    Y = [PrevStep.y]
    timeseries = [0]
    init_time = time()
    PrevStep.t = init_time
    time_now = time()
    # sys = control.tf(PLANT_NUMERATOR, PLANT_DENOMINATOR)
    sys = control_utils.Integrator(PrevStep.y, time_now)
    while time_now - init_time < 3:
        # Ti, Yi = make_step(sys, 1, time_now)
        Yi = sys.update_state(1, time_now)
        timeseries.append(time_now - init_time)
        Y.append(Yi)
        time_now = time()

    plot(timeseries, Y, 'b--', linewidth=3, label='tf')
    show()

test_1()
