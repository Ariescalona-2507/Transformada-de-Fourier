from machine import ADC, Pin, Timer
import array, math, utime, micropython

class CapturadorOnda:
    ADC_MAX = 65535.0
    VOLTS_PER_BIT = 3.3 / 65535.0
    
    def __init__(self, pin_adc=26, fs=500, buffer_size=128):
        self._adc = ADC(Pin(pin_adc))
        self._fs = fs
        self._buffer = array.array('f', [0.0] * buffer_size)
        self._indice = 0
        self._timer = Timer()
        self._periodo_us = 1000000 // fs
        self._ultimo_tiempo = utime.ticks_us()
    
    def _convertir_normalizado(self, raw):
        return ((raw * self.VOLTS_PER_BIT) - 1.65) * 0.6060606060606061
    
    def capturar(self, duracion_ms=None):
        num_muestras = len(self._buffer) if duracion_ms is None else min(
            len(self._buffer), (duracion_ms * self._fs) // 1000)
        
        if num_muestras != len(self._buffer):
            self._buffer = array.array('f', [0.0] * num_muestras)
        
        inicio = utime.ticks_us()
        for i in range(num_muestras):
            self._buffer[i] = self._convertir_normalizado(self._adc.read_u16())
            if i < num_muestras - 1:
                while utime.ticks_diff(utime.ticks_us(), inicio) < (i + 1) * self._periodo_us:
                    pass
        
        self._indice = num_muestras
        return self._buffer
    
    def _callback_muestreo(self, timer):
        if self._indice < len(self._buffer):
            self._buffer[self._indice] = self._convertir_normalizado(self._adc.read_u16())
            self._indice += 1
    
    def iniciar_async(self):
        self._indice = 0
        self._timer.init(freq=self._fs, mode=Timer.PERIODIC, callback=self._callback_muestreo)
    
    def detener(self):
        self._timer.deinit()
        return self._buffer[:self._indice]
    
    def rms(self):
        if self._indice == 0: return 0.0
        return math.sqrt(sum(x * x for x in self._buffer[:self._indice]) / self._indice)
    
    def frecuencia(self):
        if self._indice < 4: return 0.0
        cruces = sum(1 for i in range(1, self._indice) 
                    if self._buffer[i-1] <= 0 < self._buffer[i] or 
                       self._buffer[i-1] >= 0 > self._buffer[i])
        return cruces / (2.0 * (self._indice / self._fs)) if cruces >= 2 else 0.0
    
    def graficar(self, ancho=80, alto=20):
        if self._indice == 0: return
        muestras = self._buffer[:self._indice]
        min_v, max_v = min(muestras), max(muestras)
        rango = max_v - min_v
        if rango == 0: return
        
        matriz = [[' ']*ancho for _ in range(alto)]
        paso = max(1, self._indice // ancho)
        
        for x in range(ancho):
            idx = min(x * paso, self._indice - 1)
            y = int(((muestras[idx] - min_v) / rango) * (alto - 1))
            matriz[alto - 1 - y][x] = '*'
        
        print("\n" + "=" * ancho)
        for fila in matriz: print(''.join(fila))
        print("=" * ancho)
        print(f"Min:{min_v:.3f} Max:{max_v:.3f} RMS:{self.rms():.3f}")

# Ejemplo de uso encapsulado
class SistemaAdquisicion:
    def _init_(self, pin=26):
        self.capturador = CapturadorOnda(pin, fs=1000, buffer_size=256)
        self._configurar()
    
    def _configurar(self):
        micropython.alloc_emergency_exception_buf(100)
    
    def medir(self, duracion_ms=100):
        datos = self.capturador.capturar(duracion_ms)
        return {
            'muestras': len(datos),
            'rms': self.capturador.rms(),
            'frecuencia': self.capturador.frecuencia(),
            'datos': datos
        }
    
    def monitorear(self, intervalo_ms=1000):
        while True:
            self.medir(intervalo_ms)
            self.capturador.graficar()
            utime.sleep_ms(100)

# Uso mínimo
if __name__ == "__main__":
    sistema = SistemaAdquisicion()
    resultado = sistema.medir(50)
    print(f"RMS: {resultado['rms']:.3f}, Frec: {resultado['frecuencia']:.1f}Hz")