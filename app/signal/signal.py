from serial import Serial

class Signal:
    def __init__(self, baudrate: int = 9600, port: str = "COM3", timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)

    def leer_linea(self) -> str:
        linea = self.ser.readline().strip()
        return linea.decode("utf-8").strip()

    def parse_signal(self) -> list[float]:
        linea = self.leer_linea()
        if line.startswith("[") and lineaa.endswith("]"):
            valores = linea[1:-1].split(",")
            respuesta =[float(valor) for valor in valores]
            return respuesta 