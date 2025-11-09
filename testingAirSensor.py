import serial
import time

ser = serial.Serial("COM5", 9600, timeout=1)
time.sleep(2)

print("Reading Arduino data...")
while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode(errors='ignore').strip()
        print(f"Raw: {line}")
