# Header
import serial
import matplotlib
matplotlib.use('TkAgg')  # Ou 'Qt5Agg', se preferir
import matplotlib.pyplot as plt
from collections import deque
from mpl_toolkits.mplot3d import Axes3D  # Necessário para plots 3D
import numpy as np

# Euclidean distance function
def euclidean_distance(v1, v2):
    """
    Calcula a distância euclidiana entre dois vetores.

    Parâmetros:
        v1, v2 : array-like (listas ou arrays NumPy)

    Retorna:
        float : distância euclidiana entre v1 e v2
    """
    v1 = np.array(v1)
    v2 = np.array(v2)
    
    return np.linalg.norm(v1 - v2)

# Plot stream data function
def show_stream_data():
    """
    Lê dados do magnetômetro via serial e plota as projeções XY e ZY em tempo real.
    """
    ser = serial.Serial('/dev/ttyACM0', 115200)
    print("Reading data...")

    window_size = 100

    # Use deque with a maximum length of 100 to store the last 100 samples
    magx_data, magy_data, magz_data, magB_data = deque(maxlen=window_size), deque(maxlen=window_size), deque(maxlen=window_size), deque(maxlen=window_size)

    plt.ion()

    while True:
        line = ser.readline().decode('utf-8').strip()
        print(line)
        try:
            data = line.split(',')
            magx_data.append(float(data[0]))
            magy_data.append(float(data[1]))
            magz_data.append(float(data[2]))
            # Clear and plot the data
            plt.clf()# --- Subplot 1: projeção no plano XY ---
            ax1 = plt.subplot(1, 2, 1)
            ax1.scatter(magx_data, magy_data, c='b', s=5)
            ax1.set_title("Projeção no Plano XY")
            ax1.set_xlabel("MagX")
            ax1.set_ylabel("MagY")
            ax1.grid(True)
            ax1.axis('equal')
            # --- Subplot 2: projeção no plano ZY ---
            ax2 = plt.subplot(1, 2, 2)
            ax2.scatter(magx_data, magz_data, c='r', s=5)
            ax2.set_title("Projeção no Plano ZY")
            ax2.set_xlabel("MagX")
            ax2.set_ylabel("MagZ")
            ax2.grid(True)
            ax2.axis('equal')
            plt.tight_layout()
            plt.pause(0.01)
        except ValueError:
            print("Invalid data received, skipping...")
    ser.close()
    input("Press Enter to continue...")

# Function to collect magnetometer samples
def coleta_amostras_mag(numSamples=100):
    """
    Coleta amostras do magnetômetro via serial.

    Retorna:
        np.ndarray : matriz de amostras coletadas
    """
    ser = serial.Serial('/dev/ttyACM0', 115200)
    print("Reading data...")
    print("Moving the sensor in a figure-eight pattern...")
    cols = 3  # MagX, MagY, MagZ
    sample = np.zeros((cols,))
    matrix = np.zeros((numSamples, cols))
    count = 0
    while count < numSamples:
        line = ser.readline().decode('utf-8').strip()
        print("Amostra:", count+1, "data:", line)
        try:
            data = line.split(',')
            sample[0] = float(data[0])
            sample[1] = float(data[1])
            sample[2] = float(data[2])
            matrix[count, :] = sample
            count += 1
        except ValueError:
            print("Invalid data received, skipping...")
    ser.close()
    print("Data collection complete.")
    print(matrix)
    # salva a matriz em um arquivo .npy
    np.save('magnetometer_samples.npy', matrix)
    print("Samples saved to 'magnetometer_samples.npy'")
    return matrix

# calibração do magnetômetro
def calibra_mag():
    """
    Implementa a lógica de calibração do magnetômetro.
    Parâmetros:
        matrix : np.ndarray : matriz de amostras coletadas
    """
    # Lê os dados do arquivo magnetometer_samples.npy
    data = np.load('magnetometer_samples.npy')
    print("Loaded data for calibration:")
    print(data)
    # Implementa a lógica de calibração aqui

    return 0

# main function
def main():
    # -----------------------------------------------------------------------
    # TODO: [x] Plot amostras do magnetômetro e plota as projeções XY e ZX.
    # TODO: [x] Coleta amostras do magnetômetro e plota as projeções XY e ZX.
    # TODO: [x] Salva as amostras coletadas em uma matriz em arquivo.
    # TODO: [ ] Implementa a lógica da calibração.
    # TODO: [ ] Salva os dados de calibração em um arquivo.
    # TODO: [ ] Testa a calibração com novos dados.
    # -----------------------------------------------------------------------
    #show_stream_data()
    #coleta_amostras_mag()
    calibra_mag()
    return 0

# Entry point
if __name__ == "__main__":
    main()

