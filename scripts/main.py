import numpy as np
import math
from utils.plot import Plot
from dynamics.Quadrotor import Quadrotor
from MPC.MPCController import AltitudeMPC, AttitudeMPC, PositionMPC
from motor_model.motor_model import MotorModel

class Trajectory:
    def __init__(self, sim_time=10.0, dt = 0.02):
        self.sim_time = sim_time
        self.dt = dt
        self.ref = self.desiredTrajectory()
        # self.ref = self.desiredTrajectory_eight()

        self.x_ref = np.array(self.ref)[:,0]
        self.y_ref = np.array(self.ref)[:,1]
        self.z_ref = np.array(self.ref)[:,2]
        self.psi_ref = np.array(self.ref)[:,3]
    
    def desiredTrajectory(self):
        ref = []
        for i in range(int(self.sim_time/self.dt)):
            t = i*self.dt
            x = 5*math.sin(2*math.pi*t/10)
            y = 5*math.cos(2*math.pi*t/10)
            z = -2
            yaw = 2*math.pi*t/10
            ref.append([x,y,z,yaw])
        return ref
    
    def desiredTrajectory_eight(self):
        ref = []
        for i in range(int(self.sim_time / self.dt)):
            t = i * self.dt
            x = 5 * math.sin(2 * math.pi * t / 10)  # Same periodic x motion
            y = 5 * math.sin(4 * math.pi * t / 10)  # Modified y motion for figure-8
            z = -2  # Linear motion in z
            yaw = 2 * math.pi * t / 10  # Smooth yaw change
            ref.append([x, y, z, yaw])
        return ref
        
    def desired_altitude(self, quad, idx, N_):
        # initial state / last state
        x_ = np.zeros((N_+1, 2))
        u_ = np.zeros((N_, 1))

        z_ref_ = self.z_ref[idx:(idx+N_)]
        length = len(z_ref_)
        if length < N_:
            z_ex = np.ones(N_ - length)*z_ref_[-1]
            z_ref_ = np.concatenate((z_ref_, z_ex), axis=None)
        
        dz_ref_ = np.diff(z_ref_)
        dz_ref_ = np.concatenate((quad.dpos[2], dz_ref_), axis=None)
        
        ddz_ref_ = np.diff(dz_ref_)
        ddz_ref_ = np.concatenate((ddz_ref_[0], ddz_ref_), axis=None)

        thrust_ref_ = (quad.g - ddz_ref_)*quad.mq
        
        x_ = np.array([z_ref_, dz_ref_]).T
        x_ = np.concatenate((np.array([[quad.pos[2], quad.dpos[2]]]), x_), axis=0)
        u_ = np.array([thrust_ref_]).T
        # print(x_)
        # print(u_)
        return x_, u_

    def desired_position(self, quad, idx, N_, thrust):
        # initial state / last state
        x_ = np.zeros((N_+1, 4))
        u_ = np.zeros((N_, 2))

        x_ref_ = self.x_ref[idx:(idx+N_)]
        y_ref_ = self.y_ref[idx:(idx+N_)]
        length = len(x_ref_)
        if length < N_:
            x_ex = np.ones(N_ - length)*x_ref_[-1]
            x_ref_ = np.concatenate((x_ref_, x_ex), axis=None)

            y_ex = np.ones(N_ - length)*y_ref_[-1]
            y_ref_ = np.concatenate((y_ref_, y_ex), axis=None)

        dx_ref_ = np.diff(x_ref_)
        dx_ref_ = np.concatenate((quad.dpos[0], dx_ref_), axis=None)
        dy_ref_ = np.diff(y_ref_)
        dy_ref_ = np.concatenate((quad.dpos[1], dy_ref_), axis=None)

        ddx_ref_ = np.diff(dx_ref_)
        ddx_ref_ = np.concatenate((ddx_ref_[0], ddx_ref_), axis=None)
        ddy_ref_ = np.diff(dy_ref_)
        ddy_ref_ = np.concatenate((ddy_ref_[0], ddy_ref_), axis=None)

        the_ref_ = np.arcsin(np.clip(ddx_ref_*quad.mq/thrust, -1, 1))
        phi_ref_ = -np.arcsin(np.clip(ddy_ref_*quad.mq/thrust, -1, 1))

        x_ = np.array([x_ref_, y_ref_, dx_ref_, dy_ref_]).T
        x_ = np.concatenate((np.array([[quad.pos[0], quad.pos[1], quad.dpos[0], quad.dpos[1]]]), x_), axis=0)
        u_ = np.array([phi_ref_, the_ref_]).T
        
        # print(x_)
        # print(u_)
        return x_, u_

    def desired_attitude(self, quad, idx, N_, phid, thed):
        # initial state / last state
        x_ = np.zeros((N_+1, 6))
        u_ = np.zeros((N_, 3))

        phi_ref_ = phid
        the_ref_ = thed

        psi_ref_ = self.psi_ref[idx:(idx+N_)]
        length = len(psi_ref_)
        if length < N_:
            psi_ex = np.ones(N_ - length)*psi_ref_[-1]
            psi_ref_ = np.concatenate((psi_ref_, psi_ex), axis=None)

        dphi_ref_ = np.diff(phi_ref_)
        dphi_ref_ = np.concatenate((quad.dori[0], dphi_ref_), axis=None)
        dthe_ref_ = np.diff(the_ref_)
        dthe_ref_ = np.concatenate((quad.dori[1], dthe_ref_), axis=None)
        dpsi_ref_ = np.diff(psi_ref_)
        dpsi_ref_ = np.concatenate((quad.dori[2], dpsi_ref_), axis=None)

        ddphi_ref_ = np.diff(dphi_ref_)
        ddphi_ref_ = np.concatenate((ddphi_ref_[0], ddphi_ref_), axis=None)
        ddthe_ref_ = np.diff(dthe_ref_)
        ddthe_ref_ = np.concatenate((ddthe_ref_[0], ddthe_ref_), axis=None)
        ddpsi_ref_ = np.diff(dpsi_ref_)
        ddpsi_ref_ = np.concatenate((ddpsi_ref_[0], ddpsi_ref_), axis=None)

        tau_phi_ref_ = (quad.Ix*ddphi_ref_ - dthe_ref_*dpsi_ref_*(quad.Iy-quad.Iz))/quad.la
        tau_the_ref_ = (quad.Iy*ddthe_ref_ - dphi_ref_*dpsi_ref_*(quad.Iz-quad.Ix))/quad.la
        tau_psi_ref_ =  quad.Iz*ddpsi_ref_ - dphi_ref_*dthe_ref_*(quad.Ix-quad.Iy)

        x_ = np.array([phi_ref_, the_ref_, psi_ref_, dphi_ref_, dthe_ref_, dpsi_ref_]).T
        x_ = np.concatenate((np.array([[quad.ori[0], quad.ori[1], quad.ori[2], quad.dori[0], quad.dori[1], quad.dori[2]]]), x_), axis=0)
        u_ = np.array([tau_phi_ref_, tau_the_ref_, tau_psi_ref_]).T

        # print(x_)
        # print(u_)
        return x_, u_

def main():
    quad = Quadrotor()
    motor_model = MotorModel()
    
    dt = 0.02
    N = 50
    sim_time = 10.0
    iner = 0

    traj = Trajectory(sim_time, dt)

    al = AltitudeMPC(quad, T=dt, N=N)
    po = PositionMPC(quad, T=dt, N=N)
    at = AttitudeMPC(quad, T=dt, N=N)

    his_thrust = []; his_tau_phi = []; his_tau_the = []; his_tau_psi = []
    his_time = []
    his_motor_speeds = np.zeros((4,1))
    his_forces_and_torques = np.zeros((4,1))

    while iner - sim_time/dt < 0.0:
        # Solve altitude -> thrust
        next_al_trajectories, next_al_controls = traj.desired_altitude(quad, iner, N)
        thrusts = al.solve(next_al_trajectories, next_al_controls)

        # Solve position -> phid, thed
        next_po_trajectories, next_po_controls = traj.desired_position(quad, iner, N, thrusts)
        phids, theds = po.solve(next_po_trajectories, next_po_controls, thrusts)

        # Solve attitude -> tau_phi, tau_the, tau_psi
        next_at_trajectories, next_at_controls = traj.desired_attitude(quad, iner, N, phids, theds)
        tau_phis, tau_thes, tau_psis = at.solve(next_at_trajectories, next_at_controls)

        # motor speeds 
        motor_speed = motor_model.calculate_motor_speed(thrust=thrusts[0], 
                                                         torque_roll=tau_phis[0],
                                                         torque_pitch=tau_thes[0],
                                                         torque_yaw=tau_psis[0])
        
        forces_and_torques = motor_model.calculate_forces_n_torques(motor_speed)
        
        # without propeller model
        # quad.updateConfiguration(thrusts[0], 
        #                          tau_phis[0], 
        #                          tau_thes[0], 
        #                          tau_psis[0], dt)
        
        # with propeller model
        quad.updateConfiguration(float(forces_and_torques[0]), 
                                 float(forces_and_torques[1]), 
                                 float(forces_and_torques[2]), 
                                 float(forces_and_torques[3]), dt)
    
        # Store values
        his_thrust.append(thrusts[0])
        his_tau_phi.append(tau_phis[0])
        his_tau_the.append(tau_thes[0])
        his_tau_psi.append(tau_psis[0])
        his_time.append(iner*dt)
        
        his_motor_speeds = np.append(his_motor_speeds, motor_speed, axis = 1)
        his_forces_and_torques = np.append(his_forces_and_torques, forces_and_torques, axis=1)
        iner += 1
    print(his_motor_speeds.shape)
    quad_vel = np.array(quad.vel)

    # plot all the states
    Plot.plot(his_time, his_thrust, his_tau_phi, his_tau_the, his_tau_psi, 
         quad_vel, quad.path, traj.ref, his_motor_speeds, his_forces_and_torques)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Cannot run main function. An error occurred: {e}")
