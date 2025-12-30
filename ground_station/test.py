import serial

ser = serial.Serial('/dev/tty.usbserial-56BC0010791', 115200)  # Windows 改成 COM3 之類
while True:
    data = ser.read(41)
    print(len(data), data.hex())
