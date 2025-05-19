from simple_pid import PID
import numpy as np

class DronePIDController:
    def __init__(self, kp=0.1, ki=0.01, kd=0.05, dt=0.1):
        # Create two independent PID controllers for x and y
        self.pid_x = PID(kp, ki, kd, setpoint=0)
        self.pid_y = PID(kp, ki, kd, setpoint=0)

        # Output limits (increased for more responsive control but still safe)
        self.pid_x.output_limits = (-2.0, 2.0)
        self.pid_y.output_limits = (-2.0, 2.0)

        self.pid_x.sample_time = dt
        self.pid_y.sample_time = dt
        
        # Add low-pass filter for control outputs
        self.prev_control = np.array([0.0, 0.0])
        self.smoothing_factor = 0.7  # Adjustable: higher = smoother but slower response
        
        # Add velocity ramping parameters
        self.max_accel = 0.2  # Maximum acceleration per step
        self.prev_velocity = np.array([0.0, 0.0])

    def compute_control(self, offset):
        """
        Given offset (dx, dy) from image center, return control signals.
        """
        dx, dy = offset
        ux = self.pid_x(dx)  # Control for x-axis
        uy = self.pid_y(dy)  # Control for y-axis
        
        # Raw PID output
        raw_control = np.array([ux, uy])
        
        # Apply smoothing using low-pass filter
        smooth_control = self.smooth_control(raw_control)
        
        # Apply acceleration limiting
        limited_control = self.limit_acceleration(smooth_control)
        
        return limited_control
    
    def smooth_control(self, raw_control):
        """Apply exponential smoothing to control signal"""
        smoothed = self.smoothing_factor * self.prev_control + (1 - self.smoothing_factor) * raw_control
        self.prev_control = smoothed
        return smoothed
    
    def limit_acceleration(self, control):
        """Limit acceleration to prevent jerky movements"""
        # Calculate desired acceleration
        desired_accel = control - self.prev_velocity
        
        # Limit acceleration magnitude
        accel_magnitude = np.linalg.norm(desired_accel)
        if accel_magnitude > self.max_accel and accel_magnitude > 0:
            desired_accel = desired_accel * (self.max_accel / accel_magnitude)
        
        # Apply limited acceleration
        limited_control = self.prev_velocity + desired_accel
        
        # Store for next iteration
        self.prev_velocity = limited_control
        
        return limited_control
    
    def reset(self):
        """Reset controller state"""
        self.pid_x.reset()
        self.pid_y.reset()
        self.prev_control = np.array([0.0, 0.0])
        self.prev_velocity = np.array([0.0, 0.0])