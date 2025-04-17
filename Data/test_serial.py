
#!/usr/bin/env python3

import serial, time, csv, os
from datetime import datetime

# === Settings ===
COM_PORT = 'COM9'  # CHANGE THIS to your correct port
BAUD_RATE = 115200
FILL_DURATION = 45    # Time to fill beaker (seconds)
COLLECT_DURATION = 20 # Time to collect data
DRAIN_DURATION = 45   # Time to empty water

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
        print(f"Sent START. Waiting {FILL_DURATION} seconds for beaker to fill...")
        time.sleep(FILL_DURATION)

        # === Collect sensor data ===
        print(f"Collecting sensor data for {COLLECT_DURATION} seconds...")
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

        # === Stop and drain ===
        ser.write(b'STOP\n')
        print(f"Sent STOP. Waiting {DRAIN_DURATION} seconds to drain beaker...")
        time.sleep(DRAIN_DURATION)

    except KeyboardInterrupt:
        print("Logging interrupted by user.")

    finally:
        ser.close()
        print("Serial connection closed.")
