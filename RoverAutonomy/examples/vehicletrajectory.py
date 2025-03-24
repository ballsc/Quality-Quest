import numpy as np
import matplotlib.pyplot as plt

# Function to calculate vehicle trajectory
def calculate_trajectory(initial_position, velocity, steering_angle, timesteps, dt):
    # Convert steering angle from degrees to radians
    steering_angle_rad = np.radians(steering_angle)

    # Vehicle parameters
    L = 2.5  # Wheelbase length (meters)
    
    # Lists to store trajectory points
    x_positions = [initial_position[0]]
    y_positions = [initial_position[1]]
    
    # Initial heading angle (facing right along the x-axis)
    theta = 0.0
    
    for _ in range(timesteps):
        # Update position using basic kinematic equations
        x = x_positions[-1]
        y = y_positions[-1]

        # Calculate new heading angle
        theta += (velocity / L) * np.tan(steering_angle_rad) * dt

        # Update positions
        x += velocity * np.cos(theta) * dt
        y += velocity * np.sin(theta) * dt
        
        # Append new positions to lists
        x_positions.append(x)
        y_positions.append(y)

    return x_positions, y_positions

# Parameters
initial_position = (0, 0)  # Initial position (x, y)
velocity = 10.0            # Velocity in meters per second
steering_angle = 15        # Steering wheel angle in degrees
timesteps = 50             # Number of timesteps
dt = 0.1                   # Time step in seconds

# Calculate trajectory
x_positions, y_positions = calculate_trajectory(initial_position, velocity, steering_angle, timesteps, dt)

# Plot the trajectory
plt.plot(x_positions, y_positions, marker='o')
plt.title('Vehicle Trajectory')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.axis('equal')
plt.grid()
plt.show()
