import serial
import matplotlib
matplotlib.use('TkAgg')  # Ou 'Qt5Agg', se preferir
import matplotlib.pyplot as plt
from collections import deque
from mpl_toolkits.mplot3d import Axes3D  # Necessário para plots 3D

ser = serial.Serial('/dev/ttyACM0', 115200)
print("Reading data...")

window_size = 100

plt.ion()

# Use deque with a maximum length of 100 to store the last 100 samples
magx_data, magy_data, magz_data, magB_data = deque(maxlen=window_size), deque(maxlen=window_size), deque(maxlen=window_size), deque(maxlen=window_size)

while True:
    line = ser.readline().decode('utf-8').strip()
    print(line)
    try:
        data = line.split(',')
        print('len data=', len(data))
        if len(data) == 4:
            magx = float(data[0])
            magy = float(data[1])
            magz = float(data[2])
            magB = float(data[3])
            
            # Append data to the deques
            magx_data.append(magx)
            magy_data.append(magy)
            magz_data.append(magz)
            magB_data.append(magB)
            
            # Clear and plot the data
            plt.clf()
           
            # --- Subplot 1: projeção no plano XY ---
            ax1 = plt.subplot(1, 2, 1)
            ax1.scatter(magx_data, magy_data, c='b', s=5)
            ax1.set_title("Projeção no Plano XY")
            ax1.set_xlabel("MagX")
            ax1.set_ylabel("MagY")
            ax1.grid(True)
            ax1.axis('equal')

            # --- Subplot 2: projeção no plano ZY ---
            ax2 = plt.subplot(1, 2, 2)
            ax2.scatter(magz_data, magy_data, c='r', s=5)
            ax2.set_title("Projeção no Plano ZY")
            ax2.set_xlabel("MagZ")
            ax2.set_ylabel("MagY")
            ax2.grid(True)
            ax2.axis('equal')

            plt.tight_layout()
            plt.pause(0.01)
    except ValueError:
        print("Invalid data received, skipping...")

ser.close()
# Note: Make sure to adjust the serial port and baud rate as per your setup.
