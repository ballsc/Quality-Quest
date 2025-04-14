from pymavlink import mavutil

def check_arm_status(connection_string='/dev/ttyUSB0', baudrate=57600, timeout=10):
    # Connect to the MAVLink vehicle
    master = mavutil.mavlink_connection(connection_string, baud=baudrate)
    master.wait_heartbeat(timeout=timeout)
    print(f"Heartbeat received from system {master.target_system}")

    print("Listening for arming failure messages...")
    try:
        while True:
            msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=timeout)
            if msg is None:
                print("No status text received.")
                break

            text = msg.text.lower()
            severity = msg.severity

            if "prearm" in text or "arm" in text or "failsafe" in text:
                print(f"[Severity {severity}] {msg.text}")

    except KeyboardInterrupt:
        print("Stopped listening.")

check_arm_status()