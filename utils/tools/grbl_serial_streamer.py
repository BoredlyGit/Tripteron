import serial
import time

s = serial.Serial('COM3', 115200)  # TODO: Change to match your laptop/pc
s.write(b"\r\n\r\n")  # grbl wake-up
time.sleep(2)  # Wait for grbl to initialize
print(f"{s.readline().strip()}\n")

s.flushInput()  # Flush startup text in serial input

while True:
    s.write(f"{input('>>> ')}\n".encode())
    time.sleep(.2)  # give time for grbl to respond
    out = s.read_all()  # Wait for grbl response with carriage return
    print(f"{out.strip().decode()}\n")
