
import serial
import json
import time
import csv
import os
from datetime import datetime

# ---- Configuration ----
SERIAL_PORT = '/dev/ttyACM0'  # Commonly /dev/ttyACM0 for Pico over USB
BAUD_RATE = 115200
CSV_FILE = 'sensor_log.csv'

def setup_csv():
    """Initialize CSV file with headers if it doesn't exist."""
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'timestamp_ms', 
                'ultrasonic1_cm', 'ultrasonic2_cm', 
                'mic_vol', 'moisture_pct', 'gas_val'
            ])
        print(f"Created {CSV_FILE}")

def log_to_csv(data):
    """Append a row of data to the CSV file."""
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
                data.get('gas_val', 0)
            ])
    except Exception as e:
        print(f"Error writing to CSV: {e}")

def main():
    setup_csv()
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Pico on {SERIAL_PORT}")
        print(f"Logging data to {os.path.abspath(CSV_FILE)}")
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                try:
                    data = json.loads(line)
                    print(f"Received: {data}")
                    log_to_csv(data)
                    
                except json.JSONDecodeError:
                    if line: # only print if not empty
                        print(f"Invalid JSON: {line}")
                    
            time.sleep(0.01) # Reduced sleep for faster reading

    except serial.SerialException as e:
        print(f"Error connecting to serial port: {e}")
        print("Check if Pico is connected and port is correct (e.g., /dev/ttyACM0 or /dev/ttyUSB0)")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
