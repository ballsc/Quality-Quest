from pymavlink import mavutil
import time

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

# Function to change flight mode
def set_mode(mode_name):
    if mode_name not in master.mode_mapping():
        print(f"Mode {mode_name} not found!")
        return

    mode_id = master.mode_mapping()[mode_name]
    
    # Send MAVLink command to change mode
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    print(f"Requested mode change to {mode_name}...")

    # Wait until mode change is confirmed
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        current_mode = mavutil.mode_string_v10(msg)
        if current_mode == mode_name:
            print(f"Mode successfully changed to {mode_name}")
            break
        time.sleep(1)

# Step 1: Change mode to HOLD
set_mode("HOLD")

# Step 2: Wait for 30 seconds
print("Holding position for 30 seconds...")
time.sleep(30)

# Step 3: Change mode back to MANUAL
set_mode("MANUAL")

print("Experiment complete!")
