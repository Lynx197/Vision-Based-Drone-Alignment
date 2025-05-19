import numpy as np

class Drone2D:
    def __init__(self, initial_pos=(0.0, 0.0), drift_rate=4):
        """
        Simulates a 2D drone with drift.
        
        Args:
            initial_pos (tuple): Initial (x, y) position of the drone.
            drift_rate (float): Maximum random drift per axis per time step.
        """
        self.position = np.array(initial_pos, dtype=np.float64)
        self.velocity = np.zeros(2)
        self.drift_rate = drift_rate
        self.drift = np.zeros(2)
        
        # Add physical constraints for more realistic simulation
        self.max_velocity = 10.0  # Maximum velocity magnitude
        self.mass = 1.0  # Drone mass (kg)
        self.drag_coefficient = 0.05  # Air resistance factor
        
        # Drone state history for analysis
        self.position_history = [self.position.copy()]
        self.velocity_history = [self.velocity.copy()]
        self.acceleration_history = [np.zeros(2)]

    def apply_control(self, control_input, dt=0.01):
        """
        Applies control input and updates the drone's position with drift.
        Uses more realistic physics model to ensure smooth movement.
        
        Args:
            control_input (np.array): (x, y) control input (e.g., velocity command).
            dt (float): Time step.
        """
        # Simulate drift accumulation
        self.drift += np.random.uniform(-self.drift_rate, self.drift_rate, size=2) * dt
        
        # Calculate forces (F = ma)
        # Thrust force from control input
        thrust_force = control_input * self.mass
        
        # Drag force (opposes velocity)
        if np.linalg.norm(self.velocity) > 0:
            drag_force = -self.drag_coefficient * self.velocity * np.linalg.norm(self.velocity)
        else:
            drag_force = np.zeros(2)
        
        # Net force
        net_force = thrust_force + drag_force
        
        # Calculate acceleration (F = ma -> a = F/m)
        acceleration = net_force / self.mass
        
        # Update velocity using acceleration
        self.velocity += acceleration * dt
        
        # Apply velocity limits for stability
        velocity_magnitude = np.linalg.norm(self.velocity)
        if velocity_magnitude > self.max_velocity:
            self.velocity = self.velocity * (self.max_velocity / velocity_magnitude)
        
        # Update position with velocity and drift
        self.position += (self.velocity + self.drift) * dt
        
        # Store history
        self.position_history.append(self.position.copy())
        self.velocity_history.append(self.velocity.copy())
        self.acceleration_history.append(acceleration.copy())

    def get_position(self):
        """Returns the current position of the drone (with drift included)."""
        return self.position.copy()

    def reset(self):
        """Resets the drone state (position, velocity, drift)."""
        self.position = np.zeros(2)
        self.velocity = np.zeros(2)
        self.drift = np.zeros(2)
        self.position_history = [self.position.copy()]
        self.velocity_history = [self.velocity.copy()]
        self.acceleration_history = [np.zeros(2)]