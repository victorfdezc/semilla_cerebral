#!/usr/bin/env python3
import serial
import time

def main():
    # Configuración del puerto serie
    PUERTO = "/dev/ttyACM0"   # Ajusta según tu sistema
    BAUDRATE = 9600
    TIMEOUT = 1               # segundos

    # Parámetros de temporización y conteo
    DELAY_AFTER_R0 = 3.0      # seg. que mantiene R + Z 0
    INTERVAL = 1.0            # seg. entre comandos Z incrementales
    STEP_N = 10               # incremento de N
    MAX_N = 100               # valor máximo de N
    DELAY_AFTER_S = 2.0       # seg. tras enviar 'S'

    # Abre el puerto serie una sola vez
    ser = serial.Serial(PUERTO, BAUDRATE, timeout=TIMEOUT)
    time.sleep(5)  # espera que arranque Arduino

    print("Iniciando bucle de comandos. Ctrl+C para detener.")

    try:
        while True:
            # 1) R y Z 0
            ser.write(b'R\n')
            ser.write(b'Z 0\n')
            time.sleep(DELAY_AFTER_R0)

            # 2) Z N, con N = STEP_N, 2*STEP_N, … hasta MAX_N
            for n in range(STEP_N, MAX_N + 1, STEP_N):
                ser.write(f"Z {n}\n".encode("utf-8"))
                time.sleep(INTERVAL)

            # 3) Parada y espera
            ser.write(b'S\n')
            time.sleep(DELAY_AFTER_S)

    except KeyboardInterrupt:
        print("\nDetenido por usuario, cerrando puerto.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
