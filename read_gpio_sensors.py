import time
import board
import busio
import digitalio
import smbus2
import bme280
from gpiozero import InputDevice, Button
from datetime import datetime
import csv
import os
import math

# ---- Configuration ----
# I2C
I2C_BUS = 1
MPU6500_ADDR = 0x68
BME280_ADDR = 0x76 # Or 0x77

# GPIO Pins (BCM numbering)
VIBRATION_PIN = 6
LDR_PIN = 12

# CSV
CSV_FILE = 'rpi_gpio_log.csv'

# ---- MPU6500 Registers ----
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

class MPU6500:
    def __init__(self, bus, address=MPU6500_ADDR):
        self.bus = bus
        self.address = address
        self._init_sensor()
        
    def _init_sensor(self):
        # Wake up
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0)
        
    def read_raw_data(self, addr):
        # Read two bytes
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr+1)
        value = ((high << 8) | low)
        if(value > 32768):
            value = value - 65536
        return value

    def get_data(self):
        # Read Accelerometer
        acc_x = self.read_raw_data(ACCEL_XOUT_H) / 16384.0
        acc_y = self.read_raw_data(ACCEL_YOUT_H) / 16384.0
        acc_z = self.read_raw_data(ACCEL_ZOUT_H) / 16384.0
        
        # Read Gyroscope
        gyro_x = self.read_raw_data(GYRO_XOUT_H) / 131.0
        gyro_y = self.read_raw_data(GYRO_YOUT_H) / 131.0
        gyro_z = self.read_raw_data(GYRO_ZOUT_H) / 131.0
        
        return {
            'acc_x': round(acc_x, 2), 'acc_y': round(acc_y, 2), 'acc_z': round(acc_z, 2),
            'gyro_x': round(gyro_x, 2), 'gyro_y': round(gyro_y, 2), 'gyro_z': round(gyro_z, 2)
        }

def setup_csv():
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 
                'temp_bme', 'hum_bme', 'pres_bme',
                'acc_x', 'acc_y', 'acc_z',
                'gyro_x', 'gyro_y', 'gyro_z',
                'vibration', 'light_detected'
            ])

def log_to_csv(data):
    try:
        with open(CSV_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.now().isoformat(),
                data.get('temp_bme'), data.get('hum_bme'), data.get('pres_bme'),
                data.get('acc_x'), data.get('acc_y'), data.get('acc_z'),
                data.get('gyro_x'), data.get('gyro_y'), data.get('gyro_z'),
                data.get('vibration'), data.get('light_detected')
            ])
    except Exception as e:
        print(f"CSV Error: {e}")

def main():
    setup_csv()
    
    # 1. Setup I2C
    try:
        bus = smbus2.SMBus(I2C_BUS)
        mpu = MPU6500(bus)
        # BME280 parameters
        bme_params = bme280.load_calibration_params(bus, BME280_ADDR)
    except Exception as e:
        print(f"I2C Setup Error: {e}")
        return

    # 2. Setup Digital Inputs
    vibration_sensor = InputDevice(VIBRATION_PIN, pull_up=False) 
    ldr_sensor = InputDevice(LDR_PIN, pull_up=False) 

    print("Sensors Initialized (MPU, BME/BMP, Vib, LDR). Logging to CSV...")

    while True:
        data = {}
        
        # --- I2C Sensors ---
        try:
            # BME280
            bme_data = bme280.sample(bus, BME280_ADDR, bme_params)
            data['temp_bme'] = round(bme_data.temperature, 2)
            data['hum_bme'] = round(bme_data.humidity, 2)
            data['pres_bme'] = round(bme_data.pressure, 2)
            
            # MPU6500
            mpu_data = mpu.get_data()
            data.update(mpu_data)
            
        except Exception as e:
            pass # Keep looping

        # --- Digital Sensors ---
        try:
            data['vibration'] = vibration_sensor.value
            data['light_detected'] = ldr_sensor.value # 0 or 1
        except Exception as e:
            print(f"GPIO Error: {e}")


        # Log and Print
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Check BME Chip ID if possible (0x60=BME280, 0x58=BMP280)
        try:
            chip_id = bus.read_byte_data(BME280_ADDR, 0xD0)
            is_bmp = (chip_id == 0x58) 
            chip_str = f"ID:0x{chip_id:02X}"
            sensor_name = "BMP" if is_bmp else "BME"
        except:
            chip_str = "ID:??"
            is_bmp = False
            sensor_name = "BME"

        # BME/BMP
        if is_bmp:
             # BMP280 has no humidity
             bme_str = f"{sensor_name}({chip_str}):{data.get('temp_bme')}C/{data.get('pres_bme')}hPa"
        else:
             bme_str = f"{sensor_name}({chip_str}):{data.get('temp_bme')}C/{data.get('hum_bme')}%/{data.get('pres_bme')}hPa"
             
        if not data.get('temp_bme'):
            bme_str = f"{sensor_name}:ERR"

        # MPU (Acc X only for brevity)
        mpu_str = f"MPU-X:{data.get('acc_x')}" if data.get('acc_x') is not None else "MPU:ERR"
        # Vibration
        vib_str = f"Vib:{data.get('vibration')}"
        # LDR
        ldr_str = f"LDR:{data.get('light_detected')}"
        
        print(f"[{timestamp}] {bme_str} | {mpu_str} | {vib_str} | {ldr_str}")
        
        log_to_csv(data)

        time.sleep(1) # Back to 1s as DHT is gone

if __name__ == "__main__":
    main()
