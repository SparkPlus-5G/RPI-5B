import time
import board
import busio
import digitalio
import adafruit_dht
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
DHT_PIN = board.D4
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
                'temp_dht', 'hum_dht',
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
                data.get('temp_dht'), data.get('hum_dht'),
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

    # 2. Setup DHT22
    # Note: DHT22 on Pi5 might be flaky with adafruit lib alone due to timing
    dht_device = adafruit_dht.DHT22(DHT_PIN)

    # 3. Setup Digital Inputs
    # Using gpiozero for Pi 5 compatibility (requires lgpio/rpi-lgpio)
    vibration_sensor = InputDevice(VIBRATION_PIN, pull_up=False) # Usually active high? Check sensor
    ldr_sensor = InputDevice(LDR_PIN, pull_up=False) # Digital D0 usually active low/high threshold

    print("Sensors Initialized. Logging to CSV...")

    while True:
        data = {}
        
        # --- DHT22 ---
        try:
            data['temp_dht'] = dht_device.temperature
            data['hum_dht'] = dht_device.humidity
        except RuntimeError as error:
            # DHT common transient errors
            # print(error.args[0])
            pass
        except Exception as error:
            dht_device.exit()
            raise error

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
            print(f"I2C Read Error: {e}")

        # --- Digital Sensors ---
        try:
            data['vibration'] = vibration_sensor.value
            data['light_detected'] = ldr_sensor.value # 0 or 1
        except Exception as e:
            print(f"GPIO Error: {e}")

        # Log and Print
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Build status string for all 5 sensors
        # 1. DHT
        dht_str = f"DHT:{data.get('temp_dht')}C/{data.get('hum_dht')}%" if data.get('temp_dht') else "DHT:ERR"
        # 2. BME
        bme_str = f"BME:{data.get('temp_bme')}C/{data.get('hum_bme')}%/{data.get('pres_bme')}hPa" if data.get('temp_bme') else "BME:ERR"
        # 3. MPU (Acc X only for brevity)
        mpu_str = f"MPU-X:{data.get('acc_x')}" if data.get('acc_x') is not None else "MPU:ERR"
        # 4. Vibration
        vib_str = f"Vib:{data.get('vibration')}"
        # 5. LDR
        ldr_str = f"LDR:{data.get('light_detected')}"
        
        print(f"[{timestamp}] {dht_str} | {bme_str} | {mpu_str} | {vib_str} | {ldr_str}")
        
        log_to_csv(data)

        time.sleep(1)

if __name__ == "__main__":
    main()