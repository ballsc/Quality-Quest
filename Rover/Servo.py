from pymavlink import mavutil
import time

# Constants for RC values
RC_NEUTRAL = 1500
RC_MIN = 1000
RC_MAX = 2000

# Connection parameters
PORT = "COM5"  # Adjust as per your CubePilot's port
BAUDRATE = 115200

# Connect to the CubePilot Rover
print("Connecting to CubePilot Rover...")
master = mavutil.mavlink_connection(PORT, baud=BAUDRATE)

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected to system {master.target_system}, component {master.target_component}")

def send_rc_command(left_motor, right_motor, propeller_1=1500, propeller_2=1500):
    """
    Sends RC override commands to the specified channels.
    """
    # Enforce throttle limits
    left_motor = max(min(left_motor, RC_MAX), RC_MIN)
    right_motor = max(min(right_motor, RC_MAX), RC_MIN)
    propeller_1 = max(min(propeller_1, RC_MAX), RC_MIN)
    propeller_2 = max(min(propeller_2, RC_MAX), RC_MIN)

    # Send RC override command
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        left_motor,  # Channel 1 - Left motor
        right_motor, # Channel 3 - Right motor
        propeller_1, # Channel 4 - Propeller 1
        0,           # Channel 5 - Unused
        propeller_2, # Channel 6 - Propeller 2
        0, 0, 0, 0   # Channels 7 to 10 - Unused
    )

# Sending commands to demonstrate movements
print("Sending command to turn left...")
send_rc_command(RC_NEUTRAL - 100, RC_NEUTRAL + 100)
time.sleep(2)  # Command is active for 2 seconds

print("Sending command to turn right...")
send_rc_command(RC_NEUTRAL + 100, RC_NEUTRAL - 100)
time.sleep(2)  # Command is active for 2 seconds

# Clear RC overrides
print("Resetting RC commands to neutral...")
send_rc_command(1500, 1500, 1500, 1500)

print("Experiment complete!")
