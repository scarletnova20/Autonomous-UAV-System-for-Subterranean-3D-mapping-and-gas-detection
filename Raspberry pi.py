import spidev
import smbus
import serial
import socket
import time

# ---------------- Sensor Setup (Same as original) ----------------
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

ser = serial.Serial("/dev/ttyS0", 9600, timeout=1)
bus = smbus.SMBus(1)

def read_mq9(channel=0):
    adc = spi.xfer2([1, (8+channel)<<4, 0])
    data = ((adc[1]&3)<<8) + adc[2]
    voltage = data * 3.3 / 1023
    return voltage

def read_ndir():
    ser.write(b"\xFF\x01\x86\x00\x00\x00\x00\x00\x79")
    resp = ser.read(9)
    if len(resp) == 9:
        co2 = resp[2]*256 + resp[3]
        return co2
    return None

def read_o2():
    O2_SENSOR_ADDR = 0x61
    data = bus.read_i2c_block_data(O2_SENSOR_ADDR, 0x00, 2)
    o2 = (data[0] << 8 | data[1]) / 100.0
    return o2

def read_trieye():
    TRIEYE_ADDR = 0x10
    data = bus.read_i2c_block_data(TRIEYE_ADDR, 0x00, 6)
    distance = (data[0]<<8) | data[1]
    return distance

# ---------------- TCP Socket Server Setup ----------------
HOST = '0.0.0.0' # Listen on all available interfaces
PORT = 5005

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1) # Listen for one incoming connection
print(f"Waiting for Simulink client to connect on port {PORT}...")
client_socket, addr = server_socket.accept()
print(f"Accepted connection from {addr}")

# ---------------- Main Loop ----------------
try:
    while True:
        mq9_val = read_mq9()
        ndir_val = read_ndir()
        o2_val = read_o2()
        trieye_val = read_trieye()

        data_str = f"{mq9_val:.2f},{ndir_val},{o2_val:.2f},{trieye_val}\n"
        client_socket.sendall(data_str.encode())

        print("Sent to Simulink:", data_str.strip())
        time.sleep(1)

except (KeyboardInterrupt, BrokenPipeError) as e:
    print(f"Connection interrupted: {e}")
finally:
    client_socket.close()
    server_socket.close()
    spi.close()
    ser.close()
    bus.close()
    print("All connections and resources closed.")