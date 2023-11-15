import numpy as np
from casadi import *

import do_mpc

model_type = 'continuous'
class MyUSVModel():
    def __init__(self):
        self.model = do_mpc.model.Model(model_type)

        #paramteres

        m = 14.5            #boat mass with no payload/batteries
        mb = 4.8            #total battery weight
        mp = 0              #total payload weight

        M = m + mb + mp     #total weight

        ang = 0.53
        d = 0.285

        Ix = 0.6
        Iy = 0.7
        Iz = 1.6

        Dx = 1.5
        Dy = 1.5
        Dz = 1.5





        #position
        x = self.model.set_variable('_x', 'x')
        y = self.model.set_variable('_x', 'y')

        #yaw angle
        psi = self.model.set_variable('_x','psi')

        #lin vel
        u = self.model.set_variable('_x', 'u')
        v = self.model.set_variable('_x', 'v')

        #lin acc
        u_dot = self.model.set_variable('_z', 'u_dot')
        v_dot = self.model.set_variable('_z', 'v_dot')

        #yaw vel
        r = self.model.set_variable('_x', 'r')

        #yaw acc
        r_dot = self.model.set_variable('_z', 'r_dot')

        #input
        u1 = self.model.set_variable('_u', 'u1')
        u2 = self.model.set_variable('_u', 'u2')

        #current in inertial frame
        #Vx = self.model.set_variable('_p','Vx')
        #Vy = self.model.set_variable('_p','Vy')
        Vx = 0
        Vy = 0


        #Set point
        x_sp = self.model.set_variable('_tvp', 'x_sp')
        y_sp = self.model.set_variable('_tvp', 'y_sp')
        psi_sp = self.model.set_variable('_tvp', 'psi_sp')

        x_2 = self.model.set_variable('_tvp', 'x_2')
        y_2 = self.model.set_variable('_tvp', 'y_2')
        

        #dynamics


        tu = (u1 + u2)*sin(ang)**2
        tv = (u2 - u1)*sin(ang)*cos(ang)
        tr = (u2 - u1)*d*cos(ang)

        tu = (u1 + u2)
        tv = 0
        tr = (u2 - u1)*d


        self.model.set_rhs('x', u*cos(psi) - v*sin(psi) + Vx)
        self.model.set_rhs('y', u*sin(psi) + v*cos(psi) + Vy)
        self.model.set_rhs('psi', r)


        self.model.set_rhs('u', u_dot)
        self.model.set_rhs('v', v_dot)
        self.model.set_rhs('r', r_dot)



        f_1 = tu + r*v*Iy/Ix - u*Dx/Ix
        f_2 = tv - u*Ix/Iy - v*Dy/Iy
        f_3 = tr - r*Dz/Iz + v*u*(Iy-Ix)/Iz

        #f_1 = tu + r - u
        #f_2 = tv - u - v
        #f_3 = tr - r + v
        
        dynamics = vertcat(f_1,f_2,f_3)

        self.model.set_alg('dynamics', dynamics)

        self.model.setup()

