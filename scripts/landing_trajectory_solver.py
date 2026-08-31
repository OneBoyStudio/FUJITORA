import casadi as cas
import numpy as np
#import matplotlib.pyplot as plt
import sys
import os
from pathlib import Path

#from matplotlib.gridspec import GridSpec

if sys.frozen == True:
    filenm = Path(sys.executable).stem
    current_dir = Path(sys.executable).resolve().parent.parent.parent
else:
    filenm = Path(__file__).stem
    current_dir = Path(__file__).resolve().parent

INPUT_PATH = os.path.abspath(os.path.join(current_dir, "..", "guidance", "input.csv"))
OUTPUT_TEMP_PATH = os.path.abspath(os.path.join(current_dir, "..", "guidance", "trajectory_out.tmp"))
OUTPUT_PATH = os.path.abspath(os.path.join(current_dir, "..", "guidance", "trajectory_out.csv"))

def optimize_trajectory():

    print(current_dir)
    if not os.path.exists(INPUT_PATH):
        sys.exit(1)

    input_parse = np.loadtxt(INPUT_PATH, delimiter=',')

    print(input_parse)

    m_dry = input_parse[6]
    m_liquid = input_parse[7]

    I_sp = input_parse[10] # this is calculated wrt gravity on earth
    g_planet = 3.721
    alpha = 1 / (I_sp * 9.81) #always use gravity on earth even if g_planet is not earth

    T_max = input_parse[8]
    T_min = input_parse[9]

    gamma = np.radians(75)
    tan_gamma = np.tan(gamma)

    # boundary values
    x_i = np.array([input_parse[0], input_parse[1], input_parse[2], input_parse[3], input_parse[4], input_parse[5], (m_liquid + m_dry)])
    x_f = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    a_1 = g_planet
    a_2 = (T_max / (m_dry + m_liquid)) - g_planet
    v_max = ((a_2*(x_i[5]**2) + 2 * a_1 * a_2 * x_i[2]) / (a_1 + a_2))**0.5

    total_time = ((v_max - x_i[5]) / a_1) + (v_max / a_2)
    dt = 1
    N = int(1 + (total_time/dt))

    #symbollic variables
    x_sym = cas.MX.sym('x', 7)
    u_sym = cas.MX.sym('u', 3)

    p_dot = x_sym[3:6] #velocity
    v_dot = (u_sym / x_sym[6]) + cas.MX([0.0, 0.0, -g_planet]) #acceleration
    m_dot = -alpha * ((u_sym.T @ u_sym) + 1e-5)**0.5

    x_dot = cas.vertcat(p_dot, v_dot, m_dot)

    f = cas.Function('f', [x_sym, u_sym], [x_dot])

    def x_step(xsym, usym):
        k1 = f(xsym, usym)
        k2 = f(xsym + (0.5 * dt * k1), usym)
        k3 = f(xsym + (0.5 * dt * k2), usym)
        k4 = f(xsym + (dt * k3), usym)

        xnext = xsym + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        return xnext

    x_step_rk4 = cas.Function('step_rk4', [x_sym, u_sym], [x_step(x_sym, u_sym)])

    x_sym_list = []
    u_sym_list = []

    for k in range(N):
        x_sym_list.append(cas.MX.sym(f'X_{k}', 7))

    for k in range(N - 1):
        u_sym_list.append(cas.MX.sym(f'U_{k}', 3))

    g = []
    lbg = []
    ubg = []

    lbz = []
    ubz = []

    J = 0.0

    g.append(x_sym_list[0] - x_i)
    lbg.extend([0.0] * 7)
    ubg.extend([0.0] * 7)

    for k in range(N - 1):
        x_prediction = x_step_rk4(x_sym_list[k], u_sym_list[k])
        defect = x_sym_list[k + 1] - x_prediction
        g.append(defect)
        lbg.extend([0.0] * 7)
        ubg.extend([0.0] * 7)

        J += ((u_sym_list[k].T @ u_sym_list[k]) / T_max**2) * dt

        p_k = x_sym_list[k][0:3]
        glideslope = p_k[0]**2 + p_k[1]**2 - (p_k[2] * tan_gamma)**2
        g.append(glideslope)
        lbg.append(-np.inf)
        ubg.append(0.0)

        thrust_squared = u_sym_list[k].T @ u_sym_list[k]
        g.append(thrust_squared)
        lbg.append(T_min**2)
        ubg.append(T_max**2)

    g.append(x_sym_list[N - 1][:6] - x_f)
    lbg.extend([0.0] * 6)
    ubg.extend([0.0] * 6)

    for k in range(N):
        lbz.extend([-np.inf, -np.inf, 0.0, -np.inf, -np.inf, -np.inf, m_dry])
        ubz.extend([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, (m_dry + m_liquid)])

    for k in range(N - 1):
        lbz.extend([-T_max, -T_max, 0.0])
        ubz.extend([T_max, T_max, T_max])

    z_decision_vector = cas.vertcat(*x_sym_list, *u_sym_list)
    g_vector = cas.vertcat(*g)

    nlp = {
        'x': z_decision_vector,
        'f': J,
        'g': g_vector
    }

    opts = {'ipopt.print_level': 1, 'print_time': True, 'ipopt.tol': 1e-5, 'ipopt.constr_viol_tol': 1e-5}
    solver = cas.nlpsol('solver', 'ipopt', nlp, opts)

    z_0 = []
    for k in range(N):
        a = k / (N - 1)
        lerp_x = (1.0 - a) * x_i[:6] + a * x_f
        z_0.extend(lerp_x)
        z_0.append(m_dry + m_liquid)

    for k in range(N - 1):
        z_0.extend([0.0, 0.0, (m_dry + m_liquid) * g_planet])

    sol = solver(x0=z_0, lbx=lbz, ubx=ubz, lbg=lbg, ubg=ubg)

    z_opt = sol['x'].full().flatten()

    num_state_vars = N * 7
    x_opt = z_opt[:num_state_vars].reshape((N, 7))
    u_opt = z_opt[num_state_vars:].reshape((N - 1, 3))

    time_ax = np.linspace(0, (N - 1) * dt, N).reshape((N, 1))[:N - 1]
    trajectory_out = np.hstack([time_ax, x_opt[:N - 1, :6], u_opt])

    np.savetxt(OUTPUT_TEMP_PATH, trajectory_out, delimiter=',', fmt='%.6f')
    if os.path.exists(OUTPUT_PATH):
        os.remove(OUTPUT_PATH)
    os.rename(OUTPUT_TEMP_PATH, OUTPUT_PATH)

    '''print("\n--- OPTIMIZATION COMPLETE ---")
    print(f"Final Altitude: {x_opt[-1, 2]:.4f} m")
    print(f"Final Velocity: {x_opt[-1, 5]:.4f} m/s")'''

if __name__ == "__main__":
    optimize_trajectory()

'''
# Graphing
fig = plt.figure(layout='constrained')
gs = GridSpec(2, 2, figure=fig)

#trajectory
ax1 = fig.add_subplot(gs[:, 0], projection='3d')
ax1.plot(xs=x_opt[:, 0], ys=x_opt[:, 1], zs=x_opt[:, 2])

index = np.arange(0, N - 1, 4)
ax1.quiver(
    x_opt[index, 0],
    x_opt[index, 1],
    x_opt[index, 2],
    u_opt[index, 0] * 50 / (m_dry + m_liquid),
    u_opt[index, 1] * 50 / (m_dry + m_liquid),
    u_opt[index, 2] * 50 / (m_dry + m_liquid),
    color='red'
)

ax1.set_xlabel('X Distance (m)')
ax1.set_ylabel('Y Distance (m)')
ax1.set_zlabel('Z Distance (m)')

ax1.set_title('Optimized Trajectory (m)')

#mass
ax2 = fig.add_subplot(gs[0, 1])
ax2.plot(np.arange(0, N , 1), x_opt[:, 6])

ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Mass (kg)')

ax2.set_title('Mass Depletion Through Time')

#velocity
ax3 = fig.add_subplot(gs[1, 1])
ax3.plot(np.arange(0, N , 1), x_opt[:, 3], color='red', label='x velocity')
ax3.plot(np.arange(0, N , 1), x_opt[:, 4], color='yellow', label='y velocity')
ax3.plot(np.arange(0, N , 1), x_opt[:, 5], color='green', label='z velocity')

ax3.legend()
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Velocity (m/s)')

ax3.set_title('Velocity Magnitude (m/s) in Each Direction Through Time')

plt.show()
'''