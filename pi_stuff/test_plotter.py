import serial, time, collections
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


SERIAL_PORT = "/dev/ttyUSB0"
BAUD = 115200

# open serial
ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
time.sleep(0.2); ser.reset_input_buffer()

# store last N points
N = 2000  # show about 2.5 sec of data at 800 Hz
buf = collections.deque([0]*N, maxlen=N)

fig, ax = plt.subplots()
line, = ax.plot(range(N), [0]*N)
ax.set_ylim(0, 1023)
ax.set_title("Arduino PBT Simulator Output")
ax.set_ylabel("ADC value (0–1023)")

def update(frame):
    # read several samples each frame
    for _ in range(20):
        line_raw = ser.readline().decode(errors="ignore").strip()
        if not line_raw:
            continue
        try:
            val = int(line_raw)
        except ValueError:
            continue
        buf.append(val)
    line.set_ydata(buf)
    return line,

ani = FuncAnimation(fig, update, interval=50, blit=True)
plt.show()
