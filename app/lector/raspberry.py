from machine import ADC, Pin, PWM
import math, array, utime

fs, N = 500, 128
t = array.array('f', [i/fs for i in range(N)])

def generar_señal(tipo, f, a=1):
    p = 1/f
    if tipo == "seno":
        return array.array('f', [a*math.sin(6.283*f*t[i]) for i in range(N)])
    if tipo == "cuad":
        return array.array('f', [a if (t[i]%p) < p/2 else -a for i in range(N)])
    if tipo == "tri":
        return array.array('f', [a*(4*abs((t[i]%p)/p-0.5)-1) for i in range(N)])
    if tipo == "sierra":
        return array.array('f', [a*(2*((t[i]%p)/p)-1) for i in range(N)])
    return array.array('f', [0]*N)

valores = generar_señal("seno", 50, 1)

for valor in valores:
    print(f"{valor:.4f}")