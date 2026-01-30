import serial #pip install pyserial
import time
import matplotlib.pyplot as plt
from collections import deque

# Settings
SETPOINT = 26.0
MAX_POINTS = 300
PORT = 'COM11'
BAUD = 115200

# Serial setup
hSerial = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(0.5)
hSerial.reset_input_buffer()

# Plot setup
plt.ion()
fig, ax = plt.subplots()
ax.set_xlabel("Samples")
ax.set_ylabel("Temperature (°C)")
ax.set_title("Real-time temperature plot")

data = deque(maxlen=MAX_POINTS)
x = deque(maxlen=MAX_POINTS)

line, = ax.plot([], [], '.', markersize=4, label="Temperature")

# Setpoint
sp_line = ax.axhline(
    SETPOINT,
    color='r',
    linestyle='--',
    linewidth=2,
    label=f"Setpoint = {SETPOINT} °C"
)

ax.legend()

sample_idx = 0

try:
    while True:
        raw = hSerial.readline().decode(errors="ignore").strip()
        if not raw:
            continue

        values = raw.split(',')

        for v in values:
            if v == "":
                continue
            try:
                temp = float(v)
            except ValueError:
                continue

            data.append(temp)
            x.append(sample_idx)
            sample_idx += 1

        line.set_data(x, data)

        # Update y-axis only
        ax.relim()
        ax.autoscale_view(scaley=True, scalex=False)

        # Keep x-axis fixed to last MAX_POINTS
        ax.set_xlim(max(0, sample_idx - MAX_POINTS), sample_idx)

        plt.pause(0.01)

except KeyboardInterrupt:
    print("Stopped")

finally:
    hSerial.close()


