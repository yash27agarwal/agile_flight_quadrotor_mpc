import matplotlib.pyplot as plt
import numpy as np
class Plot:

    @staticmethod
    def plot(his_time, his_thrust, his_tau_phi, his_tau_the, his_tau_psi, 
             quad_vel, quad_path, ref_path, his_motor_speeds, his_forces_and_torques):
    
        # Plot Drone
        plot = Plotting("Quadrotor")
        plot.plot_path(quad_path)
        plot.plot_path(ref_path)

        # Plot control
        plt.figure()
        plt.suptitle("Force and torques plots without propeller model")
        plt.subplot(221)
        plt.plot(his_time, his_thrust)
        plt.title("The total thrust")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [N]")
        plt.grid(True)
        plt.tight_layout()

        plt.subplot(222)
        plt.plot(his_time, his_tau_phi)
        plt.title("The tau phi")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [N.m]")
        plt.grid(True)
        plt.tight_layout()

        plt.subplot(223)
        plt.plot(his_time, his_tau_the)
        plt.title("The tau theta")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [N.m]")
        plt.grid(True)
        plt.tight_layout()

        plt.subplot(224)
        plt.plot(his_time, his_tau_psi)
        plt.title("The tau psi")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [N.m]")
        plt.grid(True)
        plt.tight_layout()

        # Velocity profile
        plt.figure()
        plt.suptitle("Velocity Profile")
        plt.subplot(221)
        plt.plot(his_time, quad_vel[1:, 0])
        plt.title("vel_x")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [m/s]")
        plt.grid(True)
        plt.tight_layout()

        plt.subplot(222)
        plt.plot(his_time, quad_vel[1:, 1])
        plt.title("vel_y")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [m/s]")
        plt.grid(True)
        plt.tight_layout()

        plt.subplot(223)
        plt.plot(his_time, quad_vel[1:, 2])
        plt.title("vel_z")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [m/s]")
        plt.grid(True)
        plt.tight_layout()

        plt.figure()
        plt.suptitle("Angular Velocity Profile")
        plt.subplot(221)
        plt.plot(his_time, quad_vel[1:, 3])
        plt.title("dphi")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")
        plt.grid(True)
        plt.tight_layout()

        plt.subplot(222)
        plt.plot(his_time, quad_vel[1:, 4])
        plt.title("dthe")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")
        plt.grid(True)
        plt.tight_layout()

        plt.subplot(223)
        plt.plot(his_time, quad_vel[1:, 5])
        plt.title("dpsi")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")
        plt.grid(True)
        plt.tight_layout()
        
        plt.figure()
        plt.suptitle("Motor Speeds")
        plt.subplot(221)
        plt.plot(his_time, his_motor_speeds[0, 1:])
        plt.title("Motor 0")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")
        plt.grid(True)
        plt.tight_layout()

        plt.subplot(222)
        plt.plot(his_time, his_motor_speeds[1, 1:])
        plt.title("Motor 1")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")
        plt.grid(True)
        plt.tight_layout()

        plt.subplot(223)
        plt.plot(his_time, his_motor_speeds[2, 1:])
        plt.title("Motor 2")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")
        plt.grid(True)
        plt.tight_layout()

        plt.subplot(224)
        plt.plot(his_time, his_motor_speeds[3, 1:])
        plt.title("Motor 3")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")
        plt.grid(True)
        plt.tight_layout()
        plt.title("Motor Speed")

        # Plot control
        plt.figure()
        plt.suptitle("Force and Torque Plots with propeller model")
        plt.tight_layout()
        plt.grid(True)
        plt.subplot(221)
        plt.plot(his_time, his_forces_and_torques[0, 1:])
        plt.title("The total thrust")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [N]")
        plt.grid(True)
        plt.tight_layout()

        plt.subplot(222)
        plt.plot(his_time, his_forces_and_torques[1, 1:])
        plt.title("The tau phi")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [N.m]")
        plt.grid(True)
        plt.tight_layout()

        plt.subplot(223)
        plt.plot(his_time, his_forces_and_torques[2, 1:])
        plt.title("The tau theta")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [N.m]")
        plt.grid(True)
        plt.tight_layout()

        plt.subplot(224)
        plt.plot(his_time, his_forces_and_torques[3, 1:])
        plt.title("The tau psi")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [N.m]")
        plt.grid(True)
        plt.tight_layout()
        
        plt.show()

 
class Plotting:
    def __init__(self, name, xlim=[-10,10], ylim=[-10,10], zlim=[0,10], is_grid=True):
        self.fig = plt.figure()
        self.ax = plt.axes(projection ='3d')
        self.ax.set_title(name)
        self.ax.grid(is_grid)
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        self.ax.set_zlim(zlim)
        self.ax.set_xlabel('x [m]')
        self.ax.set_ylabel('y [m]')
        self.ax.set_zlabel('z [m]')
    
    def plot_path(self, path):
        path = np.array(path)
        self.ax.plot(path[:,0], path[:,1], -path[:,2])