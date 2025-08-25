import RPi.GPIO as GPIO
import spidev
import time
import struct
import random

# Pin definitions based on BCM numbering
LORA_CS_PIN = 8
LORA_RESET_PIN = 25
LORA_DIO0_PIN = 24

# LoRa Register Map
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_LNA = 0x0C
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_TX_BASE_ADDR = 0x0E
REG_IRQ_FLAGS = 0x12
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_PAYLOAD_LENGTH = 0x22

# LoRa Operation Modes
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_TX = 0x03
MODE_LORA = 0x80


class LoRaTransmitter:
    """
    A class to handle low-level communication with the SX1278 LoRa module.
    """

    def _init_(self, spi_bus=0, spi_device=0):
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = 5000000
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LORA_CS_PIN, GPIO.OUT)
        GPIO.setup(LORA_RESET_PIN, GPIO.OUT)
        GPIO.output(LORA_CS_PIN, GPIO.HIGH)

    def _write_register(self, reg, value):
        self.spi.xfer2([reg | 0x80, value])

    def _read_register(self, reg):
        return self.spi.xfer2([reg & 0x7F, 0x00])[1]

    def reset(self):
        GPIO.output(LORA_RESET_PIN, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(LORA_RESET_PIN, GPIO.HIGH)
        time.sleep(0.1)

    def set_op_mode(self, mode):
        self._write_register(REG_OP_MODE, mode)

    def begin(self, frequency=433E6):
        self.reset()
        if self._read_register(0x42) != 0x12:
            raise Exception("LoRa module not found. Check wiring and SPI configuration.")
        self.set_op_mode(MODE_LORA | MODE_SLEEP)
        time.sleep(0.1)
        frf = int((frequency / 32000000) * (1 << 19))
        self._write_register(REG_FRF_MSB, (frf >> 16) & 0xFF)
        self._write_register(REG_FRF_MID, (frf >> 8) & 0xFF)
        self._write_register(REG_FRF_LSB, frf & 0xFF)
        self._write_register(REG_PA_CONFIG, 0x8F)  # PA_BOOST, 17dBm
        self._write_register(REG_LNA, 0x23)  # Max LNA gain
        self._write_register(REG_MODEM_CONFIG_1, 0x72)  # Bw125, Cr4/5, Explicit Header
        self._write_register(REG_MODEM_CONFIG_2, 0x74)  # SF7, CRC On
        self.set_op_mode(MODE_LORA | MODE_STDBY)
        print("LoRa module initialized successfully.")

    def send(self, data):
        self.set_op_mode(MODE_LORA | MODE_STDBY)
        self._write_register(REG_FIFO_ADDR_PTR, 0)
        self._write_register(REG_FIFO_TX_BASE_ADDR, 0)
        self._write_register(REG_PAYLOAD_LENGTH, len(data))
        self.spi.xfer2([REG_FIFO | 0x80] + list(data))
        self.set_op_mode(MODE_LORA | MODE_TX)
        while self._read_register(REG_IRQ_FLAGS) & 0x08 == 0:
            time.sleep(0.01)
        self._write_register(REG_IRQ_FLAGS, 0xFF)  # Clear IRQ flags

    def close(self):
        self.spi.close()
        GPIO.cleanup()


# --- Main Application Logic ---

def get_3d_plot_data():
    """
    Placeholder function to get the drone's current 3D coordinates.
    In a real application, this would come from a flight controller, GPS, or SLAM algorithm.
    """
    # Simulating drone movement
    pos_x = 10.5 + random.uniform(-1.0, 1.0)
    pos_y = -42.1 + random.uniform(-1.0, 1.0)
    pos_z = 55.0 + random.uniform(-0.5, 0.5)
    return pos_x, pos_y, pos_z


def get_all_gas_data():
    """
    Placeholder function to read all four critical gas sensors.
    Replace this with your actual sensor reading code (e.g., I2C, UART, ADC).
    """
    # Simulating realistic sensor data for a mining environment
    co_ppm = random.uniform(50, 250)  # Carbon Monoxide (ppm)
    co2_ppm = random.uniform(400, 6000)  # Carbon Dioxide (ppm)
    o2_percent = random.uniform(18.0, 21.0)  # Oxygen (%)
    ch4_ppm = random.uniform(1000, 60000)  # Methane (ppm)
    return co_ppm, co2_ppm, o2_percent, ch4_ppm


if _name_ == "_main_":
    # --- Network Configuration ---
    DRONE_ID = 0x00
    BASE_STATION_ID = 0xFF

    lora = None
    try:
        # Initialize the LoRa hardware
        lora = LoRaTransmitter()
        lora.begin()

        packet_count = 0
        while True:
            # 1. GATHER ALL DATA from drone sensors
            co, co2, o2, ch4 = get_all_gas_data()
            pos_x, pos_y, pos_z = get_3d_plot_data()

            # 2. PACK DATA INTO THE BINARY STRUCTURE
            # This structure MUST exactly match the updated C struct on the STM32.
            # '<' = little-endian byte order
            # 'B' = unsigned char (1 byte)
            # 'f' = float (4 bytes)
            # The format string directly corresponds to the C struct members.
            packet_format = '<BBBfffffffB'
            packet = struct.pack(packet_format,
                                 DRONE_ID,
                                 BASE_STATION_ID,
                                 0x01,  # Packet Type: Drone Data
                                 co, co2, o2, ch4,
                                 pos_x, pos_y, pos_z,
                                 0)  # Checksum (not implemented)

            # 3. TRANSMIT THE PACKET
            print("----------------------------------------")
            print(f"Sending Packet #{packet_count}")
            print(f"  Gases: CO={co:.1f}, CO2={co2:.1f}, O2={o2:.1f}, CH4={ch4:.1f}")
            print(f"  Position: X={pos_x:.2f}, Y={pos_y:.2f}, Z={pos_z:.2f}")
            print(f"  Packet Size: {len(packet)} bytes")

            lora.send(packet)

            print("  >> Packet sent successfully.")
            packet_count += 1

            # Wait for 5 seconds before sending the next packet
            time.sleep(5)

    except Exception as e:
        print(f"\nFATAL ERROR: {e}")
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        if lora:
            lora.close()
            print("LoRa resources cleaned up.")