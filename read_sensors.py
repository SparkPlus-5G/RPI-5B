import serial
import json
import time
import csv
import os
from datetime import datetime

# ---- Configuration ----
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
CSV_FILE = 'sensor_log.csv'


def setup_csv():
    """Initialize CSV file with headers if it doesn't exist."""
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp_iso',
                'timestamp_ms',
                'ultrasonic1_cm',
                'ultrasonic2_cm',
                'mic_vol',
                'moisture_pct',
                'gas_val',
                'temperature_c',
                'humidity_pct'
            ])
        print(f"Created {CSV_FILE}")


def log_to_csv(data):
    """Append a row of sensor data to the CSV file."""
    try:
        with open(CSV_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.now().isoformat(),
                data.get('timestamp_ms', 0),
                data.get('ultrasonic1_cm', -1),
                data.get('ultrasonic2_cm', -1),
                data.get('mic_vol', 0),
                data.get('moisture_pct', 0),
                data.get('gas_val', 0),
                data.get('temperature_c', None),
                data.get('humidity_pct', None)
            ])
    except Exception as e:
        print(f"Error writing to CSV: {e}")


def main():
    setup_csv()

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Pico on {SERIAL_PORT}")
        print(f"Logging data to {os.path.abspath(CSV_FILE)}\n")

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()

                if not line:
                    continue

                try:
                    data = json.loads(line)
                    print("Received:", data)
                    log_to_csv(data)

                except json.JSONDecodeError:
                    print("Invalid JSON:", line)

            time.sleep(0.01)

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        print("Check Pico connection and port name")
    except KeyboardInterrupt:
        print("\nExiting logger...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()