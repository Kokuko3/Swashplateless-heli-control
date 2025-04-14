import serial
import time
import csv

port = "COM3"
baud_rate = 115200
data = []
timestamps = []
duration = 10

try:
    ser = serial.Serial(port, baud_rate, timeout=1)
    print(f"Connected to {port} at {baud_rate} baud")
    start_time = time.time()
    while (time.time() - start_time) < duration:
        if ser.in_waiting > 0:
            try:
                raw = ser.readline().decode('utf-8').strip()
                value = float(raw)
                current_time = time.time() - start_time
                timestamps.append(current_time)
                data.append(value)
            except ValueError:
                pass 
    with open('data_log.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time (s)', 'Value'])
        writer.writerows(zip(timestamps, data))
    print(f"Data collection complete. Saved {len(data)} entries to 'data_log.csv'.")

except serial.SerialException as e:
    print(f"Error: {e}")
except KeyboardInterrupt:
    print("Exiting early...")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")
