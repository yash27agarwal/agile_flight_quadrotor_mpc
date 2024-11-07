import numpy as np
import warnings

class MotorModel:
    def __init__(self, motor_constant: float = 2.9265e-7, 
                 moment_constant: float = 0.0162,
                 arm_length: float = 0.0775):
        thrust_coeff = motor_constant
        torque_coeff = motor_constant * moment_constant
        allocation_matrix = np.array([[thrust_coeff, thrust_coeff, thrust_coeff, thrust_coeff],
                                     [0, -arm_length*thrust_coeff, 0, arm_length*thrust_coeff],
                                     [arm_length*thrust_coeff, 0, -arm_length*thrust_coeff, 0],
                                     [torque_coeff, -torque_coeff, torque_coeff, -torque_coeff]])
        
        assert np.linalg.det(allocation_matrix) < 10e-6, f'control allocation matrix is singular, its value is {np.linalg.det(allocation_matrix)}'

        self.inv_allocation_matrix = np.linalg.inv(allocation_matrix)
        # print("inv allocation shape: ", self.inv_allocation_matrix.shape)

    def calculate_motor_speed(self, thrust, torque_roll, torque_pitch, torque_yaw):
        U = np.array([thrust, torque_roll, torque_pitch, torque_yaw]).reshape(4, 1)
        motor_speeds_sq = self.inv_allocation_matrix @ U
        if np.any(motor_speeds_sq < 0):
            warnings.warn("Negative values encountered in motor speeds squared, replacing with zero.")
        
        
        motor_speeds_sq = np.maximum(motor_speeds_sq, 0)

        motor_speeds = np.clip(np.sqrt(motor_speeds_sq), 0, 3000)
        return motor_speeds
    
