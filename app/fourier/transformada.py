import math
import cmath

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

    def mostrar_valores(self):
        print("Valores específicos de la DFT:")
        print(f"N = {self.N}, Fs = {self.frecuencia_muestreo} Hz")
        print("\nPrimeros 5 coeficientes:")
        for i in range(5):
            coef = self.coeficientes[i]
            print(f"K={i}: {coef} | Magnitud: {abs(coef)}")


if __name__ == "__main__":
    fs = 100
    t = [i/fs for i in range(100)]
    señal = [math.sin(2 * math.pi * 2 * t_i) for t_i in t]
    
    tf = TransformadaFourier(señal, fs)
    tf.mostrar_valores()