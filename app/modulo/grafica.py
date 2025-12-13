import math
import cmath
import matplotlib.pyplot as plt

class TransformadaFourier:
    def _init_(self, muestras, frecuencia_muestreo):
        self.muestras = muestras
        self.frecuencia_muestreo = frecuencia_muestreo
        self.N = len(muestras)
        self.coeficientes = self._calcular_dft()
    
    def _calcular_dft(self):
        XK = []
        for K in range(self.N):
            suma = 0 + 0j 
            for n in range(self.N):
                exponente = -2j * cmath.pi * n * K / self.N
                suma += self.muestras[n] * cmath.exp(exponente)
            XK.append(suma)
        return XK

def graficar(señal, tf, fs, t):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
    
    # Gráfico 1: Señal original
    ax1.plot(t, señal, 'b-', linewidth=2)
    ax1.set_xlabel('Tiempo (s)')
    ax1.set_ylabel('Amplitud')
    ax1.set_title('Señal Original')
    ax1.grid(True, alpha=0.3)
    
    # Gráfico 2: Espectro de magnitud
    magnitudes = [abs(coef) for coef in tf.coeficientes]
    mitad = tf.N // 2
    frecuencias = [k * fs / tf.N for k in range(mitad)]
    ax2.plot(frecuencias, magnitudes[:mitad], 'r-', linewidth=2)
    ax2.set_xlabel('Frecuencia (Hz)')
    ax2.set_ylabel('Magnitud')
    ax2.set_title('Espectro de Frecuencia')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

if name == "main":
    fs = 100
    t = [i/fs for i in range(100)]
    señal = [math.sin(2 * math.pi * 2 * t_i) for t_i in t]
    
    tf = TransformadaFourier(señal, fs)
    graficar(señal, tf, fs, t)