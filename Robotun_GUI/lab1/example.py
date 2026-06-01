import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

from robotun_lib.ode.ode_solver import ODE, ODE_TYPE
from robotun_lib.model.free_fall import free_fall
from robotun_lib.ctrl.pid_ctrl import pid_ctrl

th_pid = pid_ctrl(Kp = 40.0, Ki = 0.0, Kd = 20.0, target = 0.0)

velocity_pid = pid_ctrl(Kp = 0.0085, Ki = 0.001, Kd = 0.0, target = 0.0)

position_pid = pid_ctrl(Kp = 0.07, Ki = 0.0, Kd = 0.07, target = 0.0)

arduino_theta_pid = pid_ctrl(Kp = 1000.0, Ki = 0.01, Kd = 100.0, target = 0.0)

arduino_velocity = pid_ctrl(Kp = 1.0, Ki = 0.0, Kd = 0.0, target = 0.0)

object = free_fall()
u_history = []

def limit(v, lim):
    if v > lim:
        return lim
    elif v < -lim:
        return -lim
    else:
        return v
    
def low_pass_filter(alpha, init):
    y = init

    def _inner(x):
        nonlocal y
        y = alpha * x + (1 - alpha) * y
        return y

    return _inner

filtered_measurement = low_pass_filter(0.9934 , 0.0)

def derivate(state, step, t, dt):
    dth, th, dphi, phi = state

    # velocity_target = position_pid.get_ctrl(phi, dt)
    # velocity_pid.set_target(velocity_target)

    # th_target = velocity_pid.get_ctrl(dphi, dt)
    # th_pid.set_target(th_target)

    # # measured_th = filtered_measurement(th)
    # measured_th = th
    # u = -th_pid.get_ctrl(measured_th, dt)
    # u = limit(u, 255)
    # u_history.append(u)


    _dphi, _dth = object.get_delta2(state)
    
    return [_dth, dth, _dphi, dphi]

def ctrl_func(state, dt):
    dth, th, dphi, phi = state

    u = arduino_theta_pid.get_ctrl(th, dt)
    u = limit(u, 300)

    return [dth, th, u, phi]

if __name__ == "__main__":
    times = np.linspace(0, 10, 500)
    ode = ODE([0.0, np.pi/4, 0.0, 0.0], times, derivate, ODE_TYPE.RK4_CTRL)
    ode.set_ctrl_func(ctrl_func)
    solution = ode.solve()
    theta = solution[:,1]
    w = solution[:,2]
    phi = solution[:,3]
    wheel_x = phi * object.r

    spot_r = 0.7 * object.r
    wheel_spot_x = wheel_x + spot_r * np.cos(phi - np.pi / 2)
    wheel_spot_y = object.r - spot_r * np.sin(phi - np.pi / 2)

    mass_x = wheel_x + object.l * np.cos(theta - np.pi / 2)
    mass_y = object.r - object.l * np.sin(theta - np.pi / 2)

    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-40, 40), ylim=(-1.5, 1.5))
    ax.set_aspect('equal')
    ax.grid()

    line, = ax.plot([], [], 'k-', lw=2)
    wheel = plt.Circle((0.0, object.r), object.r, color='black', fill=False, lw=2)
    wheel_spot = plt.Circle((0.0, spot_r), 0.02, color='red')
    mass = plt.Circle((0.0, 0.0), 0.1, color='black')

    def init():
        return []


    def animate(i):
        wheel.set_center((wheel_x[i], object.r))
        wheel_spot.set_center((wheel_spot_x[i], wheel_spot_y[i]))
        mass.set_center((mass_x[i], mass_y[i]))
        line.set_data([wheel_x[i], mass_x[i]], [object.r, mass_y[i]])
        patches = [line, ax.add_patch(wheel), ax.add_patch(wheel_spot), ax.add_patch(mass)]
        return patches


    ani = animation.FuncAnimation(fig, animate, np.arange(1, len(solution)),
                                  interval=25, blit=True, init_func=init)


    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, autoscale_on=True)
    th_line, = ax2.plot(times, solution[:-1, 1], label="theta")
    # phi_line, = ax2.plot(times, solution[:-1, 3], label="phi")
    # w_line, = ax2.plot(times, solution[:-1, 2], label="w")
    # u_line, = plt.plot(times, u_history[::4], label="u")
    ax2.legend([th_line], ['theta'])
    ax2.grid()

    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111, autoscale_on=True)
    w_line, = ax3.plot(times, solution[:-1, 2], label="w")
    # phi_line, = ax2.plot(times, solution[:-1, 3], label="phi")
    # w_line, = ax2.plot(times, solution[:-1, 2], label="w")
    # u_line, = plt.plot(times, u_history[::4], label="u")
    ax3.legend([th_line], ['w'])
    ax3.grid()

    plt.grid(True)
    plt.show()