import numpy as numpy
import matplotlib.pyplot as plt
import sys
import pandas
from casadi import *
import do_mpc

from ModelloAUV import *
from ModelloUSV import *
from ControlloreAUV import *
from ControlloreUSV import *

auv = MyAUVModel()
usv = MyUSVModel()

x0 = 10
y0 = 10
z0 = 10

def line(t):# [ 0.00000000e+00  0.00000000e+00  5.33333333e-03 -3.55555556e-05] 0.25
    x = x0 + 2/3*0.01*t
    y = y0 - 0.01*t
    z = z0 + 0.01*t
    return x,y,z

x0_1 = np.array([x0,y0,z0,      #position
                 0,0,0,        #orientation
                 0,0,0,        #speed
                 0,0,0])       #angular speed

x0_2 = np.array([0,0,0,        # x, y, yaw
                 0,0,0])       # speeds

sp = [0,0,10,
      0,0,0]

V = [0,0]

mpc1 = AUVController(auv, sp)
mpc2 = USVController(usv, x0_1, V)

estimator1 = do_mpc.estimator.StateFeedback(auv.model)
estimator2 = do_mpc.estimator.StateFeedback(usv.model)

simulator1 = do_mpc.simulator.Simulator(auv.model)
simulator2 = do_mpc.simulator.Simulator(usv.model)

p_num = simulator2.get_p_template()
p_num['Vx'] = V[0]
p_num['Vy'] = V[1]

def p_fun(t_now):
    return p_num

simulator2.set_p_fun(p_fun)
#simulator1.set_param(mxstep = 1000)

tvp_template1 = simulator1.get_tvp_template()
tvp_template2 = simulator2.get_tvp_template()


simulator1.set_tvp_fun(tvp_template1)
simulator2.set_tvp_fun(tvp_template2)

params_simulator = {
    'integration_tool': 'idas',
    'abstol': 1e-10,
    'reltol': 1e-10,
    't_step': 0.1,
}

simulator1.set_param(**params_simulator)
simulator2.set_param(**params_simulator)

mpc1.mpc.x0 = x0_1
mpc2.mpc.x0 = x0_2

estimator1.x0 = x0_1
estimator2.x0 = x0_2

simulator1.x0 = x0_1
simulator2.x0 = x0_2



#simulator1.set_p_fun(p_fun)

simulator1.setup()
simulator2.setup()

mpc1.mpc.set_initial_guess()
mpc2.mpc.set_initial_guess()

mpc_graphics1 = do_mpc.graphics.Graphics(mpc1.mpc.data)
sim_graphics1 = do_mpc.graphics.Graphics(simulator1.data)

mpc_graphics2 = do_mpc.graphics.Graphics(mpc2.mpc.data)
sim_graphics2 = do_mpc.graphics.Graphics(simulator2.data)

fig1, ax1 = plt.subplots(3, sharex=True)
fig2, ax2 = plt.subplots(3, sharex=True)

fig1.align_ylabels
fig2.align_ylabels

for g in [sim_graphics1, mpc_graphics1]:
    # Plot the angle positions (phi_1, phi_2, phi_2) on the first axis:
    g.add_line(var_type='_x', var_name='x', axis=ax1[0])
    g.add_line(var_type='_x', var_name='y', axis=ax1[0])
    g.add_line(var_type='_x', var_name='z', axis=ax1[0])
    g.add_line(var_type='_x', var_name='u', axis=ax1[0])
    g.add_line(var_type='_x', var_name='v', axis=ax1[0])
    g.add_line(var_type='_x', var_name='w', axis=ax1[0])
    
    g.add_line(var_type='_x', var_name='phi', axis=ax1[2])
    g.add_line(var_type='_x', var_name='theta', axis=ax1[2])
    g.add_line(var_type='_x', var_name='psi', axis=ax1[2])
    g.add_line(var_type='_x', var_name='p', axis=ax1[2])
    g.add_line(var_type='_x', var_name='q', axis=ax1[2])
    g.add_line(var_type='_x', var_name='r', axis=ax1[2])

    g.add_line(var_type='_u', var_name='u_1', axis=ax1[1])
    g.add_line(var_type='_u', var_name='u_2', axis=ax1[1])
    g.add_line(var_type='_u', var_name='u_3', axis=ax1[1])
    g.add_line(var_type='_u', var_name='u_4', axis=ax1[1])
    g.add_line(var_type='_u', var_name='u_5', axis=ax1[1])
    g.add_line(var_type='_u', var_name='u_6', axis=ax1[1])
    g.add_line(var_type='_u', var_name='u_7', axis=ax1[1])
    g.add_line(var_type='_u', var_name='u_8', axis=ax1[1])

    # Plot the set motor positions (phi_m_1_set, phi_m_2_set) on the second axis:

ax1[0].set_ylabel('Position [m], velocity [m/s]')
ax1[1].set_ylabel('Input [N]')
ax1[2].set_ylabel('Angle [rad]')

for g in [sim_graphics2, mpc_graphics2]:
    # Plot the angle positions (phi_1, phi_2, phi_2) on the first axis:
    g.add_line(var_type='_x', var_name='x', axis=ax2[0])
    g.add_line(var_type='_x', var_name='y', axis=ax2[0])
    g.add_line(var_type='_x', var_name='u', axis=ax2[0])
    g.add_line(var_type='_x', var_name='v', axis=ax2[0])
    
    g.add_line(var_type='_x', var_name='psi', axis=ax2[2])
    g.add_line(var_type='_x', var_name='r', axis=ax2[2])

    g.add_line(var_type='_u', var_name='u1', axis=ax2[1])
    g.add_line(var_type='_u', var_name='u2', axis=ax2[1])
    # Plot the set motor positions (phi_m_1_set, phi_m_2_set) on the second axis:

ax2[0].set_ylabel('Position [m], velocity [m/s]')
ax2[1].set_ylabel('Input [N]')
ax2[2].set_ylabel('Angle [rad]')


plot1 = []
plot2 = []

setpoint1 = []
setpoint2 = []

u0_1 = np.zeros((8,1))
u0_2 = np.zeros((2,1))
j = 0

n_sim = 100

function_line = True
firt_itr = True

rot1_count = 0
rot2_count = 0


phi_0 = arctan(x0_1[2] / (x0_2[0] - x0_1[0]))
theta_0 = arctan(x0_1[2] / (x0_2[1] - x0_1[1]))

for i in range(n_sim):

    print("\t\t\t\t\t\t\t\t\t\t\t\t{}/{}".format(i,n_sim))    

    u0_1 = mpc1.mpc.make_step(x0_1)
    u0_2 = mpc2.mpc.make_step(x0_2)

    y_next_1 = simulator1.make_step(u0_1)
    y_next_2 = simulator2.make_step(u0_2)

    x0_1 = estimator1.make_step(y_next_1)
    x0_2 = estimator2.make_step(y_next_2)

    if (mpc1.x_setp >= 5):
        function_line = False
    if (function_line):
        linea = line(i)
        tz = i
    else:
        linea = 5, 0, mpc1.z_setp


    phi_0 = arctan(x0_1[2] / (x0_2[0] - x0_1[0]))
    theta_0 = arctan(x0_1[2] / (x0_2[1] - x0_1[1]))

    #phi_0 = arctan(x0_1[2] / (x0_1[0]))
    #theta_0 = arctan(x0_1[2] / (x0_1[1]))

    mpc1.x_setp, mpc1.y_setp, mpc1.z_setp = linea
    mpc1.phi_setp = phi_0
    mpc1.theta_setp = theta_0
    mpc2.x_setp = x0_1[0]
    mpc2.y_setp = x0_1[1]

lines1 = (sim_graphics1.result_lines['_x', 'x']+
        sim_graphics1.result_lines['_x', 'y']+
        sim_graphics1.result_lines['_x', 'z']+
        sim_graphics1.result_lines['_x', 'u']+
        sim_graphics1.result_lines['_x', 'v']+
        sim_graphics1.result_lines['_x', 'w']
        )
ax1[0].legend(lines1,'xyzuvw',title='position')

lines1 = (sim_graphics1.result_lines['_u', 'u_1']+
        sim_graphics1.result_lines['_u', 'u_2']+
        sim_graphics1.result_lines['_u', 'u_3']+
        sim_graphics1.result_lines['_u', 'u_4']+
        sim_graphics1.result_lines['_u', 'u_5']+
        sim_graphics1.result_lines['_u', 'u_6']+
        sim_graphics1.result_lines['_u', 'u_7']+
        sim_graphics1.result_lines['_u', 'u_8']
        )

ax1[1].legend(lines1,'12345678',title='input')

lines1 = (sim_graphics1.result_lines['_x', 'phi']+
        sim_graphics1.result_lines['_x', 'theta']+
        sim_graphics1.result_lines['_x', 'psi']+
        sim_graphics1.result_lines['_x', 'p']+
        sim_graphics1.result_lines['_x', 'q']+
        sim_graphics1.result_lines['_x', 'r'])
ax1[2].legend(lines1,'φθψpqr',title='euler angles')
sim_graphics1.plot_results()

sim_graphics1.reset_axes()

lines2 = (sim_graphics2.result_lines['_x', 'x']+
        sim_graphics2.result_lines['_x', 'y']+
        sim_graphics2.result_lines['_x', 'u']+
        sim_graphics2.result_lines['_x', 'v']
        )
ax2[0].legend(lines2,'xyuv',title='position')

lines2 = (sim_graphics2.result_lines['_u', 'u1']+
        sim_graphics2.result_lines['_u', 'u2']
        )

ax2[1].legend(lines2,'12',title='input')

lines2 = (
        sim_graphics2.result_lines['_x', 'psi']+
        sim_graphics2.result_lines['_x', 'r'])
ax2[2].legend(lines2,'ψr',title='euler angles')
sim_graphics2.plot_results()

sim_graphics2.reset_axes()





plt.show()



