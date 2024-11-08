import numpy as np
import math

class Quadrotor:
    def __init__(self, 
                 mass=0.302, 
                 gravity=9.81, 
                 inertia=np.diag([3.9195e-4, 4.0515e-4, 6.3890e-3]), 
                 motor_constant: float = 2.9265e-7, 
                 moment_constant: float = 0.0162,
                 arm_length: float = 0.0775):
        self.mass = mass
        self.arm_length = arm_length
        self.gravity = gravity
        self.inertia = inertia
        thrust_coeff = motor_constant
        torque_coeff = motor_constant * moment_constant
        self.allocation_matrix = np.array([[thrust_coeff, thrust_coeff, thrust_coeff, thrust_coeff],
                                           [0, -arm_length*thrust_coeff, 0, arm_length*thrust_coeff],
                                           [arm_length*thrust_coeff, 0, -arm_length*thrust_coeff, 0],
                                           [torque_coeff, -torque_coeff, torque_coeff, -torque_coeff]])
        
        self.state = {
            'position': np.zeros(3),  # [x, y, z]
            'velocity': np.zeros(3),  # [vx, vy, vz]
            'orientation': np.zeros(3),  # Euler angles [roll, pitch, yaw]
            'angular_velocity': np.zeros(3)  # [p, q, r]
        }

        # Setup constraints for quadrotor
        self.max_z = 0
        self.max_phi = 1.0; self.min_phi = -self.max_phi
        self.max_the = 1.0; self.min_the = -self.max_the

        self.max_dx = 15.0; self.min_dx = -self.max_dx
        self.max_dy = 15.0; self.min_dy = -self.max_dy
        self.max_dz = 15.0; self.min_dz = -self.max_dz
        self.max_dphi = math.pi/2; self.min_dphi = -self.max_dphi
        self.max_dthe = math.pi/2; self.min_dthe = -self.max_dthe
        self.max_dpsi = math.pi/2; self.min_dpsi = -self.max_dpsi

        self.max_thrust = 15.0; self.min_thrust = 0.0
        self.max_tau_phi = 10.0; self.min_tau_phi = -self.max_tau_phi
        self.max_tau_the = 10.0; self.min_tau_the = -self.max_tau_the
        self.max_tau_psi = 10.0; self.min_tau_psi = -self.max_tau_psi

    def motor_to_forces_and_torques(self, motor_speeds):
        # Convert motor speeds (rad/s) to forces using thrust coefficient
        U = self.allocation_matrix @ motor_speeds**2  
        U1 = U[0, 0]
        U2 = U[1, 0]
        U3 = U[2, 0]
        U4 = U[3, 0]
        return U1, U2, U3, U4

    def update_state(self, motor_speeds, dt=0.01):
        # Calculate forces and torques
        U1, U2, U3, U4 = self.motor_to_forces_and_torques(motor_speeds)

        # Update linear acceleration
        ddx = U1/self.mass*(np.cos(self.state['orientation'][0])*np.sin(self.state['orientation'][1])*np.cos(self.state['orientation'][2]) + \
                            np.sin(self.state['orientation'][0])*np.sin(self.state['orientation'][2]))
        ddy = U1/self.mass*(np.cos(self.state['orientation'][0])*np.sin(self.state['orientation'][1])*np.sin(self.state['orientation'][2]) - \
                            np.sin(self.state['orientation'][0])*np.cos(self.state['orientation'][2]))
        ddz = self.gravity - U1/self.mass*(np.cos(self.state['orientation'][0])*np.cos(self.state['orientation'][1]))

        # Calculate and update linear acceleration
        acceleration = np.array([ddx, ddy, ddz])
        self.state['velocity'] += acceleration * dt
        self.state['position'] += self.state['velocity'] * dt

        # Update angular acceleration
        ddphi = (self.state['angular_velocity'][1]*self.state['angular_velocity'][2]*\
                 (self.inertia[1,1] - self.inertia[2,2]) + U2)/self.inertia[0,0]
        ddthe = (self.state['angular_velocity'][2]*self.state['angular_velocity'][0]*\
                 (self.inertia[2,2] - self.inertia[0,0]) + U3)/self.inertia[1,1]
        ddpsi = (self.state['angular_velocity'][0]*self.state['angular_velocity'][1]*\
                 (self.inertia[0,0] - self.inertia[1,1]) + U4)/self.inertia[2,2]
        
        # Calculate and update angular acceleration
        angular_acceleration = np.array([ddphi, ddthe, ddpsi])
        self.state['angular_velocity'] += angular_acceleration * dt
        self.state['orientation'] += self.state['angular_velocity'] * dt

        return self.state

# # Example usage
# motor_speeds = np.array([400, 400, 400, 400]).reshape(4,1)  
# quadrotor = Quadrotor()
# print(quadrotor.update_state(motor_speeds))
