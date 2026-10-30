#have pyserial installed
import serial

#ensure that the port and serial match up
arduino = serial.Serial("COM4", 9600, timeout=1)

with open("data.txt", "w", encoding="utf-8") as file:
    while True:
        line = arduino.readline().decode(errors="replace").strip()
        if line:
            file.write(line + "\n")
            file.flush()