import numpy as np
import matplotlib.pyplot as plt
from scipy import pi
from scipy.fftpack import fft

x = np.loadtxt('poke_data.csv', delimiter='\n', unpack=True)


N = 2000;
frequency = np.linspace (0.0, 512, int (N/2))

freq_data = fft(x)
y = 2/N * np.abs (x [0:np.int (N/2)])

plt.plot(frequency, y)
plt.title('Frequency domain Signal')
plt.xlabel('Frequency in Hz')
plt.ylabel('Amplitude')
plt.show()