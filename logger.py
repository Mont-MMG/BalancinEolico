"""Guarda en CSV lo que manda el ESP32-S3.
Uso:  python logger.py COM5 e identificacion_diferencial.csv
      python logger.py COM5 m identificacion_miso.csv
      python logger.py COM5 v validacion.csv
      python logger.py COM5 g control_pid.csv   (control: se detiene con Ctrl+C)
Requiere: pip install pyserial
"""
import sys, serial, time

port, cmd, fname = sys.argv[1], sys.argv[2], sys.argv[3]
ser = serial.Serial(port, 921600, timeout=2)
time.sleep(2); ser.reset_input_buffer()
ser.write(b"a\n"); time.sleep(3.5)          # armar ESC
ser.write((cmd + "\n").encode())
n = 0
try:
  with open(fname, "w", newline="") as f:
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        if line.startswith("#"):
            print(line)
            if "FIN" in line or "STOP" in line:
                break
            continue
        f.write(line + "\n"); n += 1
        if n % 250 == 0:
            print(f"{n} muestras  |  {line}")
except KeyboardInterrupt:
    print("Detenido por el usuario")
ser.write(b"s\n"); ser.close()
print(f"Guardado {fname} ({n} lineas)")
