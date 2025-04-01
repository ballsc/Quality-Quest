from pymavlink import mavutil
import time
import sys

PORT = "COM5" 
BAUDRATE = 115200  

#  Connect to the CubePilot Rover
print("Connecting to CubePilot Rover...")
master = mavutil.mavlink_connection(PORT, baud=BAUDRATE)

#  Wait for a heartbeat from the flight controller
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

#  Function to request MAVLink messages
def request_message(message_id):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        message_id, 0, 0, 0, 0, 0, 0
    )

#  Request firmware version
def get_firmware_version():
    print("Requesting firmware version...")
    request_message(mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
    
    msg = master.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=5)
    if msg:
        firmware_version = msg.flight_sw_version
        major = (firmware_version >> 24) & 0xFF
        minor = (firmware_version >> 16) & 0xFF
        patch = (firmware_version >> 8) & 0xFF
        print(f"Firmware Version: {major}.{minor}.{patch}")
    else:
        print("Failed to retrieve firmware version.")

#  Request battery status
def get_battery_status():
    msg = master.recv_match(type="SYS_STATUS", blocking=True, timeout=5)
    if msg:
        voltage = msg.voltage_battery / 1000.0  # Convert mV to V
        print(f"Battery Voltage: {voltage:.2f}V")
    else:
        print("Battery status request failed.")



# Request attitude (roll, pitch, yaw)
def get_attitude():
    msg = master.recv_match(type="ATTITUDE", blocking=True, timeout=5)
    if msg:
        print(f"Attitude - Roll: {msg.roll:.2f}, Pitch: {msg.pitch:.2f}, Yaw: {msg.yaw:.2f}")
    else:
        print("Attitude request failed.")

# Request RC input channels
def get_rc_channels():
    msg = master.recv_match(type="RC_CHANNELS_RAW", blocking=True, timeout=5)
    if msg:
        print(f"RC Input - CH1: {msg.chan1_raw}, CH3: {msg.chan3_raw}, CH4: {msg.chan4_raw}, CH6: {msg.chan6_raw}")
    else:
        print("RC input request failed.")

# Request system status (armed/disarmed)
def get_system_status():
    msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    if msg:
        mode = msg.base_mode
        armed_status = "Armed" if mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "Disarmed"
        print(f"System Status: {armed_status}")
    else:
        print("System status request failed.")

# Call all data functions before arming
get_firmware_version()
get_battery_status()
get_attitude()
get_rc_channels()
get_system_status()

# Check if already armed
def is_armed():
    msg = master.recv_match(type="HEARTBEAT", blocking=True)
    return msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

# Arm the vehicle
print("Arming rover...")
master.arducopter_arm()
time.sleep(1)

#  Wait until armed
while not is_armed():
    print("Waiting for arming...")
    time.sleep(1)

print("Rover armed! Holding for 60 seconds...")
time.sleep(60)  # Keep armed for 60 seconds


print("Disarming rover...")
master.arducopter_disarm()
time.sleep(1)

#  Wait until disarmed
while is_armed():
    print("Waiting for disarm...")
    time.sleep(1)

print("Rover disarmed. Done!")