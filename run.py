from app.lector.lector_2 import Signal
from app.fourier.transformada import TransformadaFourier
from app.modulo.grafica import graficar
import math

signal = Signal()
linea = signal.parse_signal()

if not linea:
    fs = 100
    t = [i/fs for i in range(100)]
    linea = [math.sin(2 * math.pi * 2 * t_i) for t_i in t]
    print("Usando señal de ejemplo: seno de 2 Hz")

print(f"Longitud de la señal: {len(linea)}")
print(f"Primeros 5 valores de la señal: {linea[:5]}")

transformada = TransformadaFourier(muestras=linea, frecuencia_muestreo=100)
valores = transformada.mostrar_valores()
print(valores)

print("\n=== Análisis de la Transformada ===")
print(f"Número de muestras (N): {len(linea)}")
print(f"Frecuencia de muestreo: 100 Hz")

coeficientes = transformada.coeficientes
print(f"\nCoeficientes más relevantes:")
for k in range(5):  
    magnitud = abs(coeficientes[k])
    fase = math.degrees(math.atan2(coeficientes[k].imag, coeficientes[k].real))
    print(f"X[{k}]: Magnitud = {magnitud:.4f}, Fase = {fase:.2f}°")

magnitudes = [abs(coef) for coef in coeficientes]
indice_max = magnitudes.index(max(magnitudes[:len(magnitudes)//2]))
frecuencia_max = indice_max * 100 / len(linea)
print(f"\nFrecuencia principal detectada: {frecuencia_max:.2f} Hz (índice {indice_max})")

try:
   
    t_vector = [i/100 for i in range(len(linea))]
    graficar(linea, transformada, 100, t_vector)
except Exception as e:
    print(f"Error al graficar: {e}")
    print("Asegúrate de que la función graficar esté definida correctamente")