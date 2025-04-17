#!/usr/bin/env python3

import serial, time, csv, os
from datetime import datetime

# === Settings ===
COM_PORT = 'COM9'  # CHANGE THIS to your correct port
BAUD_RATE = 115200
COLLECT_DURATION = 60  # seconds for sensor data collection
TOTAL_DURATION = 180  # time window before force clos

# === File Setup ===
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
filename = f"sensor_log_{timestamp}.csv"
filepath = os.path.join(os.getcwd(), filename)

# === Serial Setup ===
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

print(f"Logging to {filename} and initiating Arduino test...")

with open(filepath, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Timestamp', 'pH', 'TDS'])

    try:
        # === Send Start Command to Arduino ===
        ser.write(b'START\n')

        start_time = time.time()
        while time.time() - start_time < COLLECT_DURATION:
            line = ser.readline().decode().strip()
            if line:
                try:
                    ph, tds = line.split(",")
                    now = datetime.now().isoformat()
                    print(f"{now} | pH: {ph} | TDS: {tds}")
                    writer.writerow([now, ph, tds])
                    file.flush()
                except ValueError:
                    print("Malformed line:", line)
        
        # === Send Stop/Empty Command to Arduino ===
        ser.write(b'STOP\n')
        print("Collection complete. Sent STOP command to Arduino.")

        # Wait a few seconds to allow pump out
        time.sleep(TOTAL_DURATION - COLLECT_DURATION)

    except KeyboardInterrupt:
        print("Logging interrupted by user.")

    finally:
        ser.close()
        print("Serial connection closed.")
