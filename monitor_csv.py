"""Monitor serial que guarda CSV automaticamente.

Uso (con el monitor de PlatformIO CERRADO):
    python monitor_csv.py COM14

Escribe los comandos igual que en el monitor (c, l, a, o, p 1200 1250, e, m, v, r 10, k 2 0.5 0.8, g, s).
  - Al mandar c / o / e / m / v / g  -> se abre un CSV nuevo en la carpeta datos/
  - Al mandar s, o cuando el ESP32 dice FIN/STOP -> se cierra y queda guardado
  - Cada linea se escribe al disco al momento: si se desconecta, lo grabado no se pierde
  - q -> salir
Requiere pyserial (ya viene con PlatformIO).
"""
import sys, os, time, threading
from datetime import datetime
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM14"
BAUD = 115200
CARPETA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "datos")
os.makedirs(CARPETA, exist_ok=True)

NOMBRES = {
    "c": "calibracion",
    "o": "punto_operacion",
    "e": "identificacion_diferencial",
    "m": "identificacion_miso",
    "v": "validacion",
    "g": "control_pid",
}

lock = threading.Lock()
archivo = None
ruta = None
n = 0
corriendo = True


def abrir(letra):
    global archivo, ruta, n
    cerrar()
    nombre = f"{NOMBRES[letra]}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    ruta = os.path.join(CARPETA, nombre)
    with lock:
        archivo = open(ruta, "w", newline="")
        n = 0
    print(f"[grabando] {ruta}")


def cerrar():
    global archivo
    with lock:
        if archivo:
            archivo.close()
            archivo = None
            print(f"[guardado] {ruta}  ({n} lineas)")


def lector(ser):
    global n, corriendo
    while corriendo:
        try:
            linea = ser.readline().decode(errors="ignore").strip()
        except serial.SerialException:
            print("\n[!] Se perdio la conexion con la placa.")
            corriendo = False
            break
        if not linea:
            continue
        if linea.startswith("#"):              # mensajes del ESP32
            print(linea)
            if "FIN" in linea or "STOP" in linea:
                cerrar()
            continue
        with lock:
            if archivo:
                archivo.write(linea + "\n")
                archivo.flush()                # al disco en el momento
                n += 1
                mostrar = (n % 10 == 1)        # muestra 5 lineas por segundo
            else:
                mostrar = True
        if mostrar:
            print(linea)
    cerrar()


def main():
    global corriendo
    try:
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = PORT, BAUD, 0.5
        ser.dtr = False                        # evita que la placa entre en modo descarga
        ser.rts = False
        ser.open()
    except serial.SerialException as e:
        print(f"No se pudo abrir {PORT}: {e}")
        print("Revisa que el monitor de PlatformIO este cerrado y que el puerto sea correcto.")
        return
    print(f"Conectado a {PORT}. Esperando a que la placa arranque...")
    time.sleep(3)
    print(f"Listo. Los CSV se guardan en: {CARPETA}")
    print("Comandos: c l z a o p e m v r k g s  |  q = salir\n")

    hilo = threading.Thread(target=lector, args=(ser,), daemon=True)
    hilo.start()
    try:
        while corriendo:
            cmd = input().strip()
            if not cmd:
                continue
            if cmd == "q":
                break
            letra = cmd[0]
            if letra in NOMBRES:
                abrir(letra)
            ser.write((cmd + "\n").encode())
            if letra == "s":
                time.sleep(0.3)
                cerrar()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        corriendo = False
        try:
            ser.write(b"s\n")                  # por seguridad: motores a minimo
            time.sleep(0.2)
            ser.close()
        except Exception:
            pass
        cerrar()
        print("Monitor cerrado.")


if __name__ == "__main__":
    main()
