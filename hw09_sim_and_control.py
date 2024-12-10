## The template python file for hw09 can be a bit confusing. 
## Please feel free to only use whatever makes sense to you and 
## delete the rest if you don't find it helpful.

# %% [markdown] 
# # Homework 9

# %%
import dynamics as dyn
from visualization import VizScene
from scipy.integrate import solve_ivp
from scipy.io import loadmat
import time
from matplotlib import pyplot as pl

import numpy as np


# %% [markdown]
# # Problem #1

# %%
# set up model 

# defining kinematic parameters for three-link planar robot
dh = [[0, 0, 0.2, 0],
     [0, 0, 0.2, 0],
     [0, 0, 0.2, 0]]

joint_type = ['r', 'r', 'r']

link_masses = [1, 0.5, 0.3]

# defining three different centers of mass, one for each link
r_coms = [np.array([-0.1, 0, 0]), 
          np.array([-0.1, 0, 0]), 
          np.array([-0.1, 0, 0])]

# all terms except Izz are zero because they don't matter in the 
# equations, Ixx, and Iyy are technically non-zero, we just don't 
# rotate about those axes so it doesn't matter. 
link_inertias = [np.diag([0, 0, 0.01]), 
                 np.diag([0, 0, 0.01]), 
                 np.diag([0, 0, 0.01])]

# the viscous friction coefficients for B*q_dot are:
B = np.diag([0.8, 0.3, 0.2])  # but you will need to either include
                              # these in your dynamic arm constructor,
                              # or just use them as a global variable
                              # in your EOM function for numerical 
                              # integration.

F = lambda qd: B*qd
g = np.array([0, -9.81, 0])

# my assumption is that you are using your RNE dynamics
arm1 = dyn.SerialArmDyn(dh,
                    jt=joint_type,
                    mass=link_masses,
                    r_com=r_coms,
                    link_inertia=link_inertias)


# %% [markdown]
# part a) - calculate joint torques here using your RNE, or E-L equations
# %%
data = loadmat('desired_accel.mat')

t = data['t'].flatten()
q = data['q']

qd = np.zeros((len(t), arm1.n))
qdd = np.zeros((len(t), arm1.n))

# you will need to take the derivative of q to get qd and qdd, 
# you can use np.gradient with q and the time vector "t"
# TODO - calc torques for every time step given q, qd, qdd

for i in range(0, len(q[0, :])):
    qd[:, i] = np.gradient(q[:, i], t)
    qdd[:, i] = np.gradient(qd[:, i], t)

torque = np.zeros((len(t), arm1.n)) # arm.get_M(q)*qdd # + arm.get_G(q, qd)*qd + arm.get_G(q) + F(qd)

for i in range(0, len(q[:, 0])):
    M = arm1.get_M(q[i, :])
    C = arm1.get_C(q[i, :], qd[i, :])
    G = arm1.get_G(q[i, :], g)

    torque[i, :] = M@qdd[i, :] + C@qd[i, :] + G + B@qd[i, :]

# If you have a vector of all torques called "torque", 
# you can use the following code to plot:
pl.figure()
for i in range(arm1.n):
    pl.subplot(arm1.n, 1, i+1)
    pl.plot(t, torque[:,i])
    pl.ylabel('joint '+str(i+1))
    pl.grid()
pl.xlabel('time (s)')
pl.tight_layout()
pl.show()



# %% [markdown]
# part b) - perform numerical integration as specified in  problem statement


# %%

# these are simplified equations of motion (since we assume that motor torque = 0)
def robot_grav(t, x):
    q_dot = x[0:arm1.n]
    q = x[arm1.n:]
    x_dot = np.zeros(2*arm1.n)

    x_dot[0:arm1.n] = np.linalg.inv(arm1.get_M(q)) @ (
        - arm1.get_C(q, q_dot) @ q_dot \
        - arm1.get_G(q, g) \
        - B@q_dot
    )
    x_dot[arm1.n:] = q_dot

    # TODO - define your EOM function here (that returns x_dot) 

    return x_dot

# TODO - perform numerical integration here using "solve_ivp". 
# When finished, you can use the plotting code below to help you. 
q0 = np.zeros(2*arm1.n)
sol = solve_ivp(robot_grav, [t[0], t[-1]], q0)

# making an empty figure
fig = pl.figure() 

# plotting the time vector "t" versus the solution vector for 
# the three joint positions, entry 3-6 in sol.y
pl.plot(sol.t, sol.y[arm1.n:].T)
pl.ylabel('joint positions (rad)')
pl.xlabel('time (s)')
pl.title('three-link robot falling in gravity field')
pl.grid()
pl.show()


# now show the actual robot being simulated in pyqtgraph, this will only 
# work if you have found the integration solution, and already run 
# the appropriate launch file:
# %%
# visualizing the robot acting under gravity 
# tf = 10
# num_steps = 100
# viz = VizScene()
# time_to_run = tf
# refresh_rate = num_steps/tf
# viz.add_arm(arm)

# qs = sol.y[arm.n:].T.flatten()

# for i in range(int(refresh_rate * time_to_run)):
#     viz.update(qs=[qs[i]])
#     time.sleep(1.0/refresh_rate)
    
# viz.hold()


# %% [markdown]
# # Problem #2a - PD controller

kd = 1.0 * np.eye(3)
kp = np.diag([1.0, 1.0, 10.0])

# Define the dynamics function (x_dot = f(x,u)) for integration
def eomPD(t, x, u, qdd_des, qd_des, q_des, arm):
    x_dot = np.zeros(2*arm.n)
    q_dot = x[0:arm.n]
    q = x[arm.n:]
    # TODO - calculate torque from a controller function (as in demo for one DoF)
    #      - then calculate qdd from that applied torque and other torques (C and G)
    e = q_des - q
    ed = qd_des - q_dot

    x_dot[0:arm.n] = np.linalg.inv(arm.get_M(q)) @ \
                    (kd @ ed + kp @ e - arm.get_C(q, q_dot)*q_dot - B@q_dot + arm.get_G(q, g))
    x_dot[arm.n:] = q_dot

    return x_dot


# you can define any q_des, qd_des, and qdd_des you want, but feel free to use this
# code below if it makes sense to you. I'm just defining q as a function of time
# and then taking the symbolic derivative. 
import sympy as sp
from IPython.display import display, Math
from sympy.physics.vector.printing import vlatex

t = sp.symbols('t')
q_des_sp = sp.Matrix([sp.cos(2*sp.pi*0.5*t),
                      sp.cos(2*sp.pi*0.05*t),
                      sp.cos(2*sp.pi*0.005*t)])
qd_des_sp = q_des_sp.diff(t)
qdd_des_sp = qd_des_sp.diff(t)

display(Math(r'q_{des} = ' + vlatex(q_des_sp) +\
             r', \quad \dot{q}_{des} = ' + vlatex(qd_des_sp) +\
             r', \quad \ddot{q}_{des}' + vlatex(qdd_des_sp)))

# turn them into numpy functions so that they are faster and return
# the right data type. Now we can call these functions at any time "t" 
# in the "eom" function. 
q_des = sp.lambdify(t, q_des_sp, modules='numpy')
qd_des = sp.lambdify(t, qd_des_sp, modules='numpy')
qdd_des = sp.lambdify(t, qdd_des_sp, modules='numpy')


def eom(t, x, u, arm):
    x_dot = np.zeros(2*arm.n)
    q_dot = x[0:arm.n]
    q = x[arm.n:]
    # TODO - calculate torque from a controller function (as in demo for one DoF)
    #      - then calculate qdd from that applied torque and other torques (C and G)

    x_dot[0:arm.n] = np.linalg.inv(arm.get_M(q)) @ \
                    (u - arm.get_C(q, q_dot)*q_dot - arm.get_G(q, g) - B@q)
    x_dot[arm.n:] = q_dot

    return x_dot

def tau_PD(x, qdd_des, qd_des, q_des, arm):
    q_dot = x[0:arm.n]
    q = x[arm.n:]
    e = q_des - q
    ed = qd_des - q_dot
    tau = kp @ e + kd @ ed + arm.get_G(q, g)
    return tau

# %%
# TODO define three different control functions and numerically integrate for each one. 
# If "sol" is output from simulation, plotting can look something like this:
t = data['t'].flatten()
# eom1 = lambda t, x: eomPD(t, x, 0, qdd_des(t).flatten(), qd_des(t).flatten(), q_des(t).flatten(), arm1)

def eom1(t, x):
    tau = tau_PD(x, 
                 qdd_des(t).flatten(), 
                 qd_des(t).flatten(), 
                 q_des(t).flatten(), 
                 arm1)
    return eom(t, x, tau, arm1)

sol = solve_ivp(eom1, [t[0], t[-1]], q0)

pl.figure()
title = "PD + G control, case 1"
for i in range(arm1.n):
    pl.subplot(arm1.n, 1, i+1)
    pl.plot(sol.t, sol.y[arm1.n+i,:].T, label='actual')
    pl.plot(sol.t, q_des(sol.t)[i,:].T, '--', label='commanded')
    pl.legend()
    pl.ylabel('joint '+str(i+1))
pl.xlabel('time (s)')
pl.suptitle(title)
pl.tight_layout()
pl.subplots_adjust(top=0.88)
pl.show()

# %% [markdown]
# # Problem #2b - Feed Forward Controller

kd = 1.0 * np.eye(3)
kp = 1.0 * np.eye(3)

def eomFF(t, x, u, qdd_des, qd_des, q_des, arm):
    x_dot = np.zeros(2*arm.n)
    q_dot = x[0:arm.n]
    q = x[arm.n:]
    # TODO - calculate torque from a controller function (as in demo for one DoF)
    #      - then calculate qdd from that applied torque and other torques (C and G)
    e = q_des - q
    ed = qd_des - q_dot

    x_dot[0:arm.n] = qdd_des - np.linalg.inv(arm.get_M(q)) @ \
                    (-kd @ ed - kp @ e)
    x_dot[arm.n:] = q_dot

    return x_dot

def tau_FF(x, qdd_des, qd_des, q_des, arm):
    q_dot = x[0:arm.n]
    q = x[arm.n:]
    e = q_des - q
    ed = qd_des - q_dot
    tau = kp @ e + kd @ ed + arm.get_G(q, g) +\
            arm.get_M(q_des) @ qdd_des +\
            arm.get_C(q_des, qd_des) @ qd_des +\
            B @ qd_des
    return tau

display(Math(r'q_{des} = ' + vlatex(q_des_sp) +\
             r', \quad \dot{q}_{des} = ' + vlatex(qd_des_sp) +\
             r', \quad \ddot{q}_{des}' + vlatex(qdd_des_sp)))

# %%
# TODO define three different control functions and numerically integrate for each one. 
# If "sol" is output from simulation, plotting can look something like this:
t = data['t'].flatten()
eom1 = lambda t, x: eomFF(t, x, 0, qdd_des(t).flatten(), qd_des(t).flatten(), q_des(t).flatten(), arm1)

# def eom1(t, x):
#     tau = tau_FF(x, 
#                  qdd_des(t).flatten(), 
#                  qd_des(t).flatten(), 
#                  q_des(t).flatten(), 
#                  arm1)
#     return eom(t, x, tau, arm1)

sol = solve_ivp(eom1, [t[0], t[-1]], q0)

pl.figure()
title = "Feed forward with PD control, case 2"
for i in range(arm1.n):
    pl.subplot(arm1.n, 1, i+1)
    pl.plot(sol.t, sol.y[arm1.n+i,:].T, label='actual')
    pl.plot(sol.t, q_des(sol.t)[i,:].T, '--', label='commanded')
    pl.legend()
    pl.ylabel('joint '+str(i+1))
pl.xlabel('time (s)')
pl.suptitle(title)
pl.tight_layout()
pl.subplots_adjust(top=0.88)
pl.show()

# %% [markdown]
# # Problem #2c - Computed torque with PD control
def eomTorque(t, x, u, qdd_des, qd_des, q_des, arm):
    x_dot = np.zeros(2*arm.n)
    q_dot = x[0:arm.n]
    q = x[arm.n:]
    # TODO - calculate torque from a controller function (as in demo for one DoF)
    #      - then calculate qdd from that applied torque and other torques (C and G)
    e = q_des - q
    ed = qd_des - q_dot

    x_dot[0:arm.n] = qdd_des + (kd @ ed + kp @ e)
    x_dot[arm.n:] = q_dot

    return x_dot

def tau_torque(x, qdd_des, qd_des, q_des, arm):
    q_dot = x[0:arm.n]
    q = x[arm.n:]
    e = q_des - q
    ed = qd_des - q_dot
    tau = np.linalg.inv(arm.get_M(q)) @ (qdd_des + kp @ e + kd @ ed) +\
            arm.get_G(q, g) +\
            arm.get_C(q_des, qd_des) @ qd_des +\
            B @ qd_des
    return tau

display(Math(r'q_{des} = ' + vlatex(q_des_sp) +\
             r', \quad \dot{q}_{des} = ' + vlatex(qd_des_sp) +\
             r', \quad \ddot{q}_{des}' + vlatex(qdd_des_sp)))

# %%
# TODO define three different control functions and numerically integrate for each one. 
# If "sol" is output from simulation, plotting can look something like this:
t = data['t'].flatten()
eom1 = lambda t, x: eomTorque(t, x, 0, qdd_des(t).flatten(), qd_des(t).flatten(), q_des(t).flatten(), arm1)

# def eom1(t, x):
#     tau = tau_torque(x, 
#                  qdd_des(t).flatten(), 
#                  qd_des(t).flatten(), 
#                  q_des(t).flatten(), 
#                  arm1)
#     return eom(t, x, tau, arm1)

sol = solve_ivp(eom1, [t[0], t[-1]], q0)

pl.figure()
title = "Computed torque with PD control, case 3"
for i in range(arm1.n):
    pl.subplot(arm1.n, 1, i+1)
    pl.plot(sol.t, sol.y[arm1.n+i,:].T, label='actual')
    pl.plot(sol.t, q_des(sol.t)[i,:].T, '--', label='commanded')
    pl.legend()
    pl.ylabel('joint '+str(i+1))
pl.xlabel('time (s)')
pl.suptitle(title)
pl.tight_layout()
pl.subplots_adjust(top=0.88)
pl.show()

# %% [markdown]
# # Problem #3

# If we make a "new" arm object, and call it a different variable name, we can use it 
# for control, while we simulate with the original model (or vice versa). Here's a way 
# to add some noise to our parameters:
percent_err = 0.10

# masses of each link with some error. 
link_masses = [np.random.uniform(low = link_masses[0]*(1-percent_err), high = link_masses[0]*(1+percent_err)),
               np.random.uniform(low = link_masses[1]*(1-percent_err), high = link_masses[1]*(1+percent_err)),
               np.random.uniform(low = link_masses[2]*(1-percent_err), high = link_masses[2]*(1+percent_err))]

# defining three different centers of mass, one for each link
r_coms = [np.array([np.random.uniform(low = -0.1*(1+percent_err), high = -0.1*(1-percent_err)), 0, 0]), 
          np.array([np.random.uniform(low = -0.1*(1+percent_err), high = -0.1*(1-percent_err)), 0, 0]),
          np.array([np.random.uniform(low = -0.1*(1+percent_err), high = -0.1*(1-percent_err)), 0, 0])]

arm2 = dyn.SerialArmDyn(dh,
                    jt=joint_type,
                    mass=link_masses,
                    r_com=r_coms,
                    link_inertia=link_inertias)

# %%
# redo all functions with new arm object

# plot arm with Torque control
eom1 = lambda t, x: eomTorque(t, x, 0, qdd_des(t).flatten(), qd_des(t).flatten(), q_des(t).flatten(), arm2)
sol = solve_ivp(eom1, [t[0], t[-1]], q0)

pl.figure()
title = "Arm with Varied Parameters: Computed torque + PD control"
for i in range(arm2.n):
    pl.subplot(arm2.n, 1, i+1)
    pl.plot(sol.t, sol.y[arm2.n+i,:].T, label='actual')
    pl.plot(sol.t, q_des(sol.t)[i,:].T, '--', label='commanded')
    pl.legend()
    pl.ylabel('joint '+str(i+1))
pl.xlabel('time (s)')
pl.suptitle(title)
pl.tight_layout()
pl.subplots_adjust(top=0.88)
pl.show()
# %%
# plot arm with PD control
def eom1(t, x):
    tau = tau_PD(x, 
                 qdd_des(t).flatten(), 
                 qd_des(t).flatten(), 
                 q_des(t).flatten(), 
                 arm1)
    return eom(t, x, tau, arm2)

sol = solve_ivp(eom1, [t[0], t[-1]], q0)

pl.figure()
title = "Arm with Varied Parameters: Computed torque + PD control"
for i in range(arm2.n):
    pl.subplot(arm2.n, 1, i+1)
    pl.plot(sol.t, sol.y[arm2.n+i,:].T, label='actual')
    pl.plot(sol.t, q_des(sol.t)[i,:].T, '--', label='commanded')
    pl.legend()
    pl.ylabel('joint '+str(i+1))
pl.xlabel('time (s)')
pl.suptitle(title)
pl.tight_layout()
pl.subplots_adjust(top=0.88)
pl.show()
# %%
