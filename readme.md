# Transformada-de-Fourier

**Estructura General del Código:**
El código implementa una Transformada Discreta de Fourier (DFT) mediante una clase en Python. La DFT es un algoritmo fundamental en procesamiento de señales digitales que convierte una señal del dominio del tiempo al dominio de la frecuencia.

Análisis Detallado por Módulo:
* Módulos Importados:

      import math
      import cmath

* math: Proporciona funciones matemáticas estándar para operaciones con números reales.

* Usado para: math.sin(), math.pi en la generación de la señal de prueba.

* cmath: Extensión del módulo math para números complejos.

* Usado para: cmath.exp() para exponenciales complejas y cmath.pi para precisión en cálculos complejos.

**Clase TransformadaFourier:**
 * Método __init__ (Constructor)

        def __init__(self, muestras, frecuencia_muestreo):

* Función: Inicializa la instancia de la clase con los parámetros esenciales.

* muestras: Lista de valores de la señal en el dominio del tiempo.

        def _calcular_dft(self):

**Implementación de la fórmula matemática:**

X[k]= 
n=0
∑
N−1
​
 x[n]⋅e 
−j2π 
N
nk
​

 
 
**Algoritmo implementado:**

* Doble bucle anidado: Complejidad O(N²)

* Bucle externo: Para cada K (frecuencia/índice de frecuencia)

* Bucle interno: Para cada n (índice de tiempo)

**Cálculo del exponente complejo:**

exponente = -2j * cmath.pi * n * K / self.N

**Sumatoria acumulativa:**

suma += self.muestras[n] * cmath.exp(exponente)

**Variables clave:**

* XK: Lista que almacena los coeficientes complejos de la DFT.

* K: Índice de frecuencia (0 a N-1).

* n: Índice de tiempo (0 a N-1).

**Método mostrar_valores**

        def mostrar_valores(self):

* Propósito: Visualización de resultados.

* Muestra información general (N, Fs).

* Presenta los primeros 5 coeficientes complejos y sus magnitudes.

* Magnitud: Calculada con abs(coef) - representa la amplitud de cada componente frecuencial. 

**Bloque Principal (__main__)**

        if __name__ == "__main__":

**Generación de Señal de Prueba:**

fs = 100  # Frecuencia de muestreo: 100 Hz
t = [i/fs for i in range(100)]  # Vector tiempo: 0 a 0.99 segundos
señal = [math.sin(2 * math.pi * 2 * t_i) for t_i in t]

**Características de la señal:**

* Tipo: Senoidal pura

* Frecuencia: 2 Hz

* Duración: 1 segundo (100 muestras a 100 Hz)

* Forma: x(t)=sin(2π⋅2⋅t)

**Creación de Objeto y Ejecución:**

        tf = TransformadaFourier(señal, fs)
        tf.mostrar_valores()

**Lector:**

**Visión General:**

Este código implementa un sistema completo de adquisición de señales analógicas para MicroPython, diseñado específicamente para microcontroladores como Raspberry Pi Pico. Permite capturar, procesar y visualizar señales en tiempo real.

## **Análisis Detallado por Módulo:**

### **Módulos Importados:**

        from machine import ADC, Pin, Timer
        import array, math, utime, micropython

* machine: Módulo específico de MicroPython para control de hardware.

* ADC: Convertidor Analógico-Digital para leer voltajes.

* Pin: Control de pines GPIO.

* Timer: Temporizadores hardware para operaciones periódicas.

* array: Implementación eficiente de arrays tipados.

* Usado para buffers de muestras con tipo específico ('f' = float).

* math: Operaciones matemáticas básicas (sqrt para cálculos RMS).

* utime: Funciones de tiempo específicas de MicroPython.

* ticks_us(): Tiempo en microsegundos con alta resolución.

* sleep_ms(): Pausas en milisegundos.

* micropython: Funciones específicas del intérprete.

* alloc_emergency_exception_buf(): Reserva memoria para excepciones en interrupciones.

### **Constantes y Constructor**

ADC_MAX = 65535.0
VOLTS_PER_BIT = 3.3 / 65535.0

**Constantes:**
* ADC_MAX = 65535.0
* VOLTS_PER_BIT = 3.3 / 65535.0

**Constructor:**

        def __init__(self, pin_adc=26, fs=500, buffer_size=128):

**Parámetros:**

* pin_adc=26: Pin ADC por defecto (GPIO26 en Raspberry Pi Pico).

* fs=500: Frecuencia de muestreo (500 Hz).

* buffer_size=128: Tamaño del buffer circular.

**Inicialización:**

* self._adc = ADC(Pin(pin_adc)): Configura el ADC en el pin especificado.

* self._periodo_us = 1000000 // fs: Calcula período entre muestras en µs.

* self._buffer: Buffer circular de floats para almacenar muestras.

## **Método de Conversión:**

        def _convertir_normalizado(self, raw):
            return ((raw * self.VOLTS_PER_BIT) - 1.65) * 0.6060606060606061

**Proceso de normalización:**

* raw * self.VOLTS_PER_BIT: Convierte valor ADC a voltios (0-3.3V).

- 1.65: Resta offset de 1.65V (centra señal en 0V).

* 0.6060606060606061: Factor de escala para normalizar a ±1V.

0.606060... = 20/33 ≈ 0.606
Justificación: Si la entrada máxima es ±2.72V, factor = 1/2.72 ≈ 0.3676, pero aquí usa 0.606.

**Métodos de Captura**

* Captura Síncrona:

        def capturar(self, duracion_ms=None):

* Funcionamiento: Captura bloqueante con control preciso de timing.

**Control de tiempo:**

        while utime.ticks_diff(utime.ticks_us(), inicio) < (i + 1) * self._periodo_us:
            pass

* Usa ticks_diff() para comparación de tiempo segura (evita overflow).

* Mantiene muestreo isócrono exacto.

**Captura Asíncrona:**

        def _callback_muestreo(self, timer):

* Mecanismo: Interrupción por timer periódico.

* Ventajas: No bloquea CPU durante captura.

* Limitaciones: Mayor jitter en timing.


## **Procesamiento de Señal**

**Cálculo RMS:**

        def rms(self):
            return math.sqrt(sum(x * x for x in self._buffer[:self._indice]) / self._indice)

* Fórmula: RMS= 
N
1
​
 ∑ 
i=1
N
​
 x 
i
2
* Aplicación: Medida de amplitud eficaz de señales AC.


**Estimación de Frecuencia:**
        def frecuencia(self):
            cruces = sum(1 for i in range(1, self._indice) 
                        if self._buffer[i-1] <= 0 < self._buffer[i] or 
                        self._buffer[i-1] >= 0 > self._buffer[i])

* Método: Conteo de cruces por cero.
* Fórmula: f= 
2×T 
total
​
 
N 
cruces
​

**Visualización ASCII**

        def graficar(self, ancho=80, alto=20):

**Algoritmo de graficación:**

* Normalización: Escala valores al rango [0, alto-1].

* Muestreo: paso = max(1, self._indice // ancho) - reduce resolución horizontal.

* Matriz de caracteres: Crea grid de alto × ancho.

* Dibujado: Coloca '*' en posición calculada.

* Rotación vertical: alto - 1 - y para orientación correcta (Y positivo arriba).

#
# **Raspberry**

## **Visión General**

Este código implementa un generador de señales periódicas en MicroPython, diseñado para crear formas de onda comunes utilizadas en procesamiento de señales y pruebas de sistemas electrónicos. El código es eficiente y adecuado para ejecutarse en microcontroladores con recursos limitados.

## **Análisis Detallado por Módulo**

* Módulos Importados

        from machine import ADC, Pin, PWM
        import math, array, utime

* ADC: Convertidor Analógico-Digital (no utilizado en este código)

* Pin: Control de pines GPIO (no utilizado en este código)

* PWM: Modulación por Ancho de Pulso (no utilizado en este código)

### **Módulos estándar:**

* math: Proporciona funciones matemáticas (sin, abs)

* array: Crea arrays tipados para eficiencia de memoria

* utime: Funciones de tiempo (no utilizado en este código)

### **Estructura del Código**

**Configuración Global**

        fs, N = 500, 128

* Parámetros:

* fs = 500: Frecuencia de muestreo (500 Hz)

* N = 128: Número de muestras a generar

**Vector de Tiempo**

        t = array.array('f', [i/fs for i in range(N)])

Matemáticamente: t[i]= i/fs i=0,1,2,...,N−1

**Ejemplo con valores reales:**

*   Para i\=0i\=0: t\[0\]\=0/500\=0.0t\[0\]\=0/500\=0.0 segundos

*   Para i\=1i\=1: t\[1\]\=1/500\=0.002t\[1\]\=1/500\=0.002 segundos

*   Para i\=127i\=127: t\[127\]\=127/500\=0.254t\[127\]\=127/500\=0.254 segundos


**Características del array:**

*   Tipo: `'f'` (float de 32 bits)
    
*   Tamaño: 128 elementos
    
*   Rango temporal: 0 a 0.254 segundos
    
*   Resolución temporal: Δt\=1/fs\=0.002Δt\=1/fs\=0.002 segundos


## **Función `generar_señal` \- Análisis Matemático**

### **Parámetros de Entrada**

        def generar_señal(tipo, f, a=1):
            p = 1/f  # Período fundamental

**Parámetros:**

*   `tipo`: Tipo de señal ("seno", "cuad", "tri", "sierra")
    
*   `f`: Frecuencia en Hz
    
*   `a`: Amplitud (valor pico)
    

**Variable calculada:**

*   `p`: Período en segundos (p\=1fp\=f1​)
    

### **Señal Senoidal**

        if tipo \== "seno":
            return array.array('f', \[a*math.sin(6.283*f*t[i\]) for i in range(N)\])

**Ecuación matemática:** x(t)\=a⋅sin⁡(2πft)x(t)\=a⋅sin(2πft)

**Detalles:**

*   `6.283` es una aproximación de 2π2π (6.283185307179586)
    
*   Frecuencia angular: ω\=2πfω\=2πf
    
*   Número de ciclos completos en la señal: ciclos\=f⋅N/fsciclos\=f⋅N/fs
    

**Ejemplo para f=50Hz, fs=500Hz, N=128:**

*   Muestras por ciclo: fs/f\=500/50\=10fs/f\=500/50\=10 muestras
    
*   Ciclos completos: 128/10\=12.8128/10\=12.8 ciclos
    

### **Señal Cuadrada**

        if tipo \== "cuad":
            return array.array('f', \[a if (t[i\]%p) < p/2 else \-a for i in range(N)\])

**Matemáticamente:**

x(t)\={asi (tmod  p)<p/2−aen otro casox(t)\={a−a​si (tmodp)<p/2en otro caso​

**Explicación:**

*   `t[i] % p`: Calcula el tiempo dentro del período actual (valor entre 0 y p)
    
*   `< p/2`: Evalúa si estamos en la primera mitad del período
    
*   Ciclo de trabajo: 50% (igual tiempo en alto y bajo)
    

**Ejemplo visual:**


        Alto (a):  |¯¯¯¯¯¯|     |¯¯¯¯¯¯|     |¯¯¯¯¯¯|

        Bajo (-a): |     |\_\_\_\_\_|     |\_\_\_\_\_|     |\_\_\_\_

         0   p/2   p   3p/2  2p   5p/2  3p

### **Señal Triangular**

        if tipo \== "tri":
            return array.array('f', \[a*(4*abs((t[i\]%p)/p\-0.5)\-1) for i in range(N)\])

**Descomposición matemática:**

1.  `(t[i] % p) / p`: Tiempo normalizado al período (0 a 1)
    
2.  `... - 0.5`: Centra en 0 (rango: -0.5 a 0.5)
    
3.  `abs(...)`: Valor absoluto (rango: 0 a 0.5)
    
4.  `4 * ...`: Escala a (0 a 2)
    
5.  `... - 1`: Ajusta a (-1 a 1)
    
6.  `a * ...`: Aplica amplitud
    

**Forma simplificada:**

x(t)\=a(4∣tmod  pp−0.5∣−1)x(t)\=a(4​ptmodp​−0.5​−1)

**Características:**

*   Pendiente constante positiva y negativa
    
*   Puntos de discontinuidad en derivada (pero función continua)
    

### **Señal Sierra (Diente de Sierra)**

        if tipo \== "sierra":
            return array.array('f', \[a*(2*((t[i\]%p)/p)\-1) for i in range(N)\])

**Descomposición:**

1.  `(t[i] % p) / p`: Tiempo normalizado (0 a 1)
    
2.  `2 * ...`: Escala a (0 a 2)
    
3.  `... - 1`: Desplaza a (-1 a 1)
    
4.  `a * ...`: Aplica amplitud
    

**Ecuación:**

x(t)\=a(2⋅(tmod p/ p)−1)

**Características:**

*   Ascenso lineal continuo
    
*   Caída instantánea (discontinuidad)
    
*   Útil para barridos de frecuencia
    

### **Caso por Defecto**

        return array.array('f', \[0\]\*N)

*   Retorna señal nula si el tipo no es reconocido

## **Generación y Visualización**

        valores \= generar\_señal("seno", 50, 1)

        for valor in valores:
            print(f"{valor:.4f}")

**Proceso:**

1.  Genera señal senoidal de 50Hz, amplitud 1
    
2.  Itera por los 128 valores
    
3.  Imprime cada valor con 4 decimales
    

**Salida esperada (primeras líneas):**

    0.0000
    0.5878
    0.9511
    0.9511
    0.5878
    0.0000
    -0.5878
    -0.9511
    ...


#
# Signal

## **Visión General**

Este código implementa una clase para leer y procesar señales enviadas a través de comunicación serial. Está diseñado para interfaces con dispositivos embebidos que envían datos en formato de lista de números.

## **Análisis Detallado del Código**

### **Importación de Módulos**

        from serial import Serial

*   **`serial`**: Módulo para comunicación serie (pyserial).
    
*   **`Serial`**: Clase principal para crear conexiones serie.
    
*   **Propósito**: Establecer comunicación con dispositivos externos.
    

### **Clase `Signal`**

#### **Constructor (`__init__`)**:

        def \_\_init\_\_(self, baudrate: int \= 9600, port: str \= "COM3", timeout\=1):

**Parámetros**:

*   `baudrate=9600`: Velocidad de transmisión (bits/segundo).
    
*   `port="COM3"`: Puerto serie (Windows). En Linux sería `/dev/ttyUSB0`.
    
*   `timeout=1`: Tiempo máximo de espera para lectura (segundos).
    

**Inicialización**:

*   Crea objeto `Serial` con configuración proporcionada.
    
*   Abre conexión inmediatamente con el puerto.
    

#### **Método `leer_linea`**:

        def leer\_linea(self) \-\> str:
            linea \= self.ser.readline().strip()
            return linea.decode("utf-8").strip()

**Flujo de ejecución**:

1.  `self.ser.readline()`: Lee bytes hasta encontrar `\n` (nueva línea).
    
2.  `.strip()`: Elimina espacios y saltos de línea.
    
3.  `.decode("utf-8")`: Convierte bytes a string Unicode.
    
4.  `.strip()`: Limpia espacios residuales.
    

**Retorno**: String con la línea leída.

#### **Método `parse_signal`**:

        def parse\_signal(self) \-\> list\[float\]:
            linea \= self.leer\_linea()
            if line.startswith("\[") and lineaa.endswith("\]"):
                valores \= linea\[1:-1\].split(",")
                respuesta \=\[float(valor) for valor in valores\]
                return respuesta

**Proceso**:

1.  Lee línea del puerto serie.
    
2.  **NOTA**: Hay errores tipográficos:
    
    *   `line` debería ser `linea`
        
    *   `lineaa` debería ser `linea`
        
3.  Verifica si la línea es una lista entre corchetes.
    
4.  Extrae contenido entre `[` y `]`.
    
5.  Divide por comas.
    
6.  Convierte cada valor a `float`.
    

**Retorno**: Lista de números flotantes o falla silenciosa.

## **Formato de Datos Esperado**

**Formato correcto**:

    \[1.23, 4.56, -7.89, 0.12\]

**Características**:

*   Inicia con `[` y termina con `]`
    
*   Valores separados por comas
    
*   Números pueden incluir decimales y signos
    
*   Sin espacios (o serán eliminados por `strip()`)

#
# Gráfica

## **Visión General**

Este código implementa una **Transformada Discreta de Fourier (DFT)** desde cero y proporciona visualización de señales en tiempo y frecuencia. Es una herramienta educativa para entender los fundamentos del procesamiento de señales digitales.

## **Análisis de Módulos Importados**

        import math
        import cmath
        import matplotlib.pyplot as plt

### **Módulo `math`**

*   **Propósito**: Operaciones matemáticas básicas con números reales
    
*   **Uso en el código**:
    
    *   `math.sin()`: Generar señal senoidal de prueba
        
    *   `math.pi`: Constante π para cálculos de frecuencia
        
*   **Característica**: No maneja números complejos
    

### **Módulo `cmath`**

*   **Propósito**: Funciones matemáticas para números complejos
    
*   **Uso en el código**:
    
    *   `cmath.exp()`: Exponencial compleja para la DFT
        
    *   `cmath.pi`: Precisión en cálculos complejos
        
*   **Importancia**: Esencial para cálculos de Fourier
    

### **Módulo `matplotlib.pyplot`**

*   **Propósito**: Visualización gráfica de datos
    
*   **Uso en el código**:
    
    *   `plt.subplots()`: Crear múltiples gráficos
        
    *   `plt.plot()`: Trazar curvas
        
    *   `plt.show()`: Mostrar gráficos
        
*   **Alias**: Se usa `plt` como nombre corto
    

## **Clase `TransformadaFourier`**

### **Constructor (con error)**

        def \_init\_(self, muestras, frecuencia\_muestreo):

**Error crítico**: Debe ser `__init__` con doble guión bajo.

**Parámetros**:

*   `muestras`: Array/list con valores de la señal temporal
    
*   `frecuencia_muestreo`: Frecuencia de muestreo (Hz)
    

**Atributos inicializados**:

*   `self.muestras`: Copia de la señal de entrada
    
*   `self.frecuencia_muestreo`: Fs almacenada
    
*   `self.N`: Número de muestras (longitud)
    
*   `self.coeficientes`: Resultados DFT (calculados automáticamente)
    

### **Método `_calcular_dft`**

**Implementación directa de la fórmula DFT**:

X\[k\]\=∑n\=0N−1x\[n\]⋅e−j2πknNX\[k\]\=n\=0∑N−1​x\[n\]⋅e−j2πNkn​

**Algoritmo O(N²)**:

        for K in range(self.N):        \# Para cada frecuencia
            suma \= 0 + 0j              \# Acumulador complejo
            for n in range(self.N):    \# Para cada muestra temporal
                exponente \= \-2j \* cmath.pi \* n \* K / self.N
                suma += self.muestras\[n\] \* cmath.exp(exponente)
            XK.append(suma)

**Explicación del exponente**:

*   `-2j * cmath.pi` \= $-j2\pi$ (fase negativa para análisis)
    
*   `n * K / self.N` \= $\frac{kn}{N}$ (índice normalizado)
    
*   Resultado: $e^{-j2\pi\frac{kn}{N}}$ (fasor rotatorio)
    

## **4. Función `graficar`**

### **4.1. Configuración de Subplots**

        fig, (ax1, ax2) \= plt.subplots(1, 2, figsize\=(12, 4))

*   **1 fila, 2 columnas**: Gráficos lado a lado
    
*   **figsize=(12,4)**: 12 pulgadas ancho × 4 alto
    

### **4.2. Gráfico 1: Dominio Temporal**

python

ax1.plot(t, señal, 'b-', linewidth\=2)

*   **Eje X**: Vector tiempo `t` (segundos)
    
*   **Eje Y**: Valores de amplitud de la señal
    
*   **'b-'**: Línea azul continua
    
*   **Propósito**: Ver forma de onda original
    

### **4.3. Gráfico 2: Dominio Frecuencial**

        magnitudes \= \[abs(coef) for coef in tf.coeficientes\]
        mitad \= tf.N // 2
        frecuencias \= \[k * fs / tf.N for k in range(mitad)\]
        ax2.plot(frecuencias, magnitudes\[:mitad\], 'r-', linewidth\=2)

**Procesamiento de espectro**:

1.  **Magnitudes**: Valor absoluto de coeficientes complejos
    
2.  **Mitad del espectro**: Debido a simetría conjugada (señales reales)
    
3.  **Vector frecuencias**:
    
    *   Frecuencia de cada bin: $f_k = k \cdot \frac{Fs}{N}$
        
    *   Rango: 0 a $\frac{Fs}{2}$ (frecuencia de Nyquist)
        

**Propiedades del espectro**:

*   **Resolución frecuencial**: $\Delta f = \frac{Fs}{N}$ = 1 Hz
    
*   **Máxima frecuencia representable**: $\frac{Fs}{2}$ = 50 Hz
    

## **Bloque Principal (`__main__`)**

### **Configuración de Prueba**

        fs \= 100  \# Frecuencia de muestreo: 100 Hz
        t \= \[i/fs for i in range(100)\]  \# Vector tiempo: 0 a 0.99 segundos
        señal \= \[math.sin(2 * math.pi * 2 * t_i) for t_i in t\]

**Señal generada**:

*   **Tipo**: Senoidal pura
    
*   **Frecuencia**: 2 Hz
    
*   **Amplitud**: 1 (unidad)
    
*   **Duración**: 1 segundo
    
*   **Muestras**: 100 puntos
    

### **Ejecución del Análisis**

python

tf \= TransformadaFourier(señal, fs)
graficar(señal, tf, fs, t)

**Resultado esperado**:

1.  **Gráfico temporal**: Seno de 2 Hz, 2 ciclos completos
    
2.  **Gráfico frecuencial**: Pico en 2 Hz, simétrico en 98 Hz (no mostrado)

