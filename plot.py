import matplotlib.pyplot as plt
from Plotting import Plotting

class Plot:

    @staticmethod
    def plot(his_time, his_thrust, his_tau_phi, his_tau_the, his_tau_psi, 
         quad_vel, quad_path, ref_path, his_motor_speeds):
    
        # Plot Drone
        plot = Plotting("Quadrotor")
        plot.plot_path(quad_path)
        plot.plot_path(ref_path)

        # Plot control
        plt.figure()
        plt.subplot(221)
        plt.plot(his_time, his_thrust)
        plt.title("The total thrust")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [N]")

        plt.subplot(222)
        plt.plot(his_time, his_tau_phi)
        plt.title("The tau phi")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [N.m]")

        plt.subplot(223)
        plt.plot(his_time, his_tau_the)
        plt.title("The tau theta")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [N.m]")

        plt.subplot(224)
        plt.plot(his_time, his_tau_psi)
        plt.title("The tau psi")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [N.m]")

        # Velocity profile
        plt.figure()
        plt.subplot(221)
        plt.plot(his_time, quad_vel[1:, 0])
        plt.title("vel_x")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [m/s]")

        plt.subplot(222)
        plt.plot(his_time, quad_vel[1:, 1])
        plt.title("vel_y")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [m/s]")

        plt.subplot(223)
        plt.plot(his_time, quad_vel[1:, 2])
        plt.title("vel_z")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [m/s]")

        plt.figure()
        plt.subplot(221)
        plt.plot(his_time, quad_vel[1:, 3])
        plt.title("dphi")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")

        plt.subplot(222)
        plt.plot(his_time, quad_vel[1:, 4])
        plt.title("dthe")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")

        plt.subplot(223)
        plt.plot(his_time, quad_vel[1:, 5])
        plt.title("dpsi")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")
        
        plt.figure()
        plt.subplot(221)
        plt.plot(his_time, his_motor_speeds[0, 1:])
        plt.title("Motor 0 speed (rad/s)")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")

        plt.subplot(222)
        plt.plot(his_time, his_motor_speeds[1, 1:])
        plt.title("Motor 1 speed (rad/s)")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")

        plt.subplot(223)
        plt.plot(his_time, his_motor_speeds[2, 1:])
        plt.title("Motor 2 speed (rad/s)")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")

        plt.subplot(224)
        plt.plot(his_time, his_motor_speeds[3, 1:])
        plt.title("Motor 3 speed (rad/s)")
        plt.xlabel("Time [s]")
        plt.ylabel("Value [rad/s]")

        plt.show()