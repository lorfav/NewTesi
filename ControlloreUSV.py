import do_mpc
from casadi import *
import numpy as np

class USVController():

    def __init__(self, model, setPoints, V):
        self.x_setp = setPoints[0]
        self.y_setp = setPoints[1]
        self.psi_setp = setPoints[2]
        self.V = V

        self.x_2 = self.x_setp
        self.y_2 = 0

        self.mpc = do_mpc.controller.MPC(model.model)

        setup_mpc = {
            'n_horizon':20,
            't_step':0.1,
            'n_robust':2,
            'store_full_solution':True
        }

        self.mpc.set_param(**setup_mpc)

        
        state = model.model.x
        input = model.model.u
        tvp = model.model.tvp

        mterm = (model.model.x['x']**2 + state['y']**2)
        
        lterm = ((tvp['x_sp'] - state['x'])**2 + (tvp['y_sp'] - state['y'])**2)

        mterm = ((tvp['x_sp'] - state['x'])**2 + (tvp['y_sp'] - state['y'])**2)

        Vx_array = np.array([0,0,0])
        Vy_array = np.array([0,0,0])
        #self.mpc.set_uncertainty_values(Vx = Vx_array, Vy = Vy_array)



        self.mpc.set_tvp_fun(self.tvp_fun)

        self.mpc.set_rterm(
            u1 = 0.1,
            u2 = 0.1,
        )
    
        self.mpc.set_objective(mterm = mterm, lterm = lterm)



        self.mpc.bounds['lower', '_u', 'u1'] = -6.2
        self.mpc.bounds['lower', '_u', 'u2'] = -6.2
        
        
        self.mpc.bounds['upper', '_u', 'u1'] =  6.2
        self.mpc.bounds['upper', '_u', 'u2'] =  6.2


        self.mpc.setup()
            

    
    def tvp_fun(self, t_now):
        tvp_template = self.mpc.get_tvp_template()
        for k in range(21):
            tvp_template['_tvp', k, 'x_sp'] =  self.x_setp
            tvp_template['_tvp', k, 'y_sp'] =  self.y_setp
            tvp_template['_tvp', k, 'psi_sp'] = self.psi_setp

            tvp_template['_tvp',k,'x_2'] =  self.x_2
            tvp_template['_tvp',k,'y_2'] =  self.y_2

            self.x_setp = self.x_2
            self.y_setp = self.y_2
        print("set x = {}".format(self.x_setp))
        print(self.y_setp)
        #print(self.state['x'])

        return tvp_template
    

        
    





