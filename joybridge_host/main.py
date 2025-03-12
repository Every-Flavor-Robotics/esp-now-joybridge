import struct
import time
import platform
from pathlib import Path

import esptool
import click
import pygame
import serial
import serial.tools.list_ports
from xbox_joy import XboxController


# Function to automatically guess the serial port for esp32-s3 devices
def guess_serial_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        # Check if the device pid is 4097
        if port.pid == 4097:
            return port.device

        if "esp32" in port.description.lower():
            return port.device
    return None


# Determine default serial port based on OS
guessed_port = guess_serial_port()
if guessed_port:
    DEFAULT_SERIAL_PORT = guessed_port
else:
    # Fallback defaults for Windows and macOS if automatic detection fails
    if platform.system() == "Windows":
        DEFAULT_SERIAL_PORT = "COM3"
    elif platform.system() == "Darwin":
        DEFAULT_SERIAL_PORT = "/dev/cu.usbmodem1421"  # This may vary on macOS
    else:
        DEFAULT_SERIAL_PORT = "/dev/ttyACM0"

# Initialize PyGame for joystick input
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise Exception("No joystick detected")

joystick = pygame.joystick.Joystick(0)
joystick.init()


def get_init_message(service_name):
    """
    Get the initial message that is sent to the Arduino to initialize the serial connection.

    Message definition:
    struct InitSerial {
      char service_name[16];
    };
    """
    # Make sure the service name is less than or equal to 16 characters
    assert len(service_name) <= 16
    # Convert service name to bytes and pad it to 16 bytes
    service_name = service_name.encode("utf-8").ljust(16, b"\0")
    return struct.pack("16s", service_name)


def decode_init_message(msg):
    """
    Decode the init message sent by the Arduino.
    """
    # Unpack the message into a tuple and extract the service name
    service_name = struct.unpack("16s", msg)[0]
    # Convert the service name to a string
    return service_name.decode("utf-8")


def add_start_sequence(msg, message_type):
    """
    Add a start sequence to the beginning of the message that includes:
    - A header marker: b"STAR"
    - The message type, a single byte
    """
    header = b"STAR"
    # Confirm message type is a single byte
    assert len(message_type) == 1
    message_type = message_type.encode()
    return header + message_type + msg


def perform_handshake(ser, service_name):
    """
    Send the init message (with message type 0x02) and wait for the Arduino's init response.
    Returns True if the handshake succeeds, False otherwise.
    """
    ser.reset_input_buffer()
    init_msg = get_init_message(service_name)
    init_msg_with_header = add_start_sequence(init_msg, "\x02")
    handshake_attempts = 3
    handshake_success = False

    for attempt in range(handshake_attempts):
        ser.write(init_msg_with_header)

        click.secho(
            f"Attempting to connect to transmitter (attempt {attempt + 1}/{handshake_attempts})...",
            fg="yellow",
        )

        handshake_timeout = 1  # seconds
        start_time = time.time()
        while time.time() - start_time < handshake_timeout:
            # Arduino sends back exactly the InitSerial struct (16 bytes)
            if ser.in_waiting >= 16:
                response = ser.read(16)
                received_service_name = decode_init_message(response)
                if received_service_name.strip("\x00") == service_name:
                    handshake_success = True
                    click.secho("Connected to transmitter", fg="green")
                    break
        if handshake_success:
            break

    return handshake_success


def run_communication_loop(controller, serial_port, service_name):
    ser = None
    try:
        click.secho("Attempting to connect on serial port: " + serial_port, fg="yellow")
        ser = serial.Serial(serial_port, 115200, timeout=1)

        if not perform_handshake(ser, service_name):
            print("Handshake failed. Exiting communication loop.")
            ser.close()
            return

        while True:
            controller.update()
            msg = controller.get_struct()
            # Message type 0x01 indicates joystick data
            msg = add_start_sequence(msg, "\x01")
            ser.write(msg)

            # Optionally print any serial data from Arduino
            if ser.in_waiting > 0:
                print(ser.readline().decode("utf-8", errors="replace").strip())

            time.sleep(1 / 60.0)

    except Exception as e:
        if ser is not None:
            ser.close()
        raise


@click.command()
@click.option(
    "--serial-port",
    default=DEFAULT_SERIAL_PORT,
    help="Serial port to use for connection (e.g., /dev/ttyACM0 or COM3)",
)
@click.option(
    "--service-name",
    default="JoystickService",
    help="Service name for the handshake (max 16 characters)",
)
def main(serial_port, service_name):
    while True:
        controller = XboxController()
        try:
            run_communication_loop(controller, serial_port, service_name)
        except (serial.SerialException, OSError):
            time.sleep(2)
        except KeyboardInterrupt:
            break


@click.command(name="flash")
@click.option(
    "--firmware-dir",
    default="assets/firmware",
    help="Directory containing the firmware binary files",
)
@click.option(
    "--port",
    default=DEFAULT_SERIAL_PORT,
    help="Serial port to use for flashing (e.g., /dev/ttyACM0 or COM3)",
)
def flash_firmware(
    firmware_dir,
    port="/dev/ttyS0",
    baud="460800",
):
    """
    Flash an ESP32-S3 with multiple binary files (bootloader, partition table,
    boot_app0, and main firmware) using esptool, similar to PlatformIO's command:

      esptool.py
          --chip esp32s3
          --port /dev/ttyACM1
          --baud 460800
          --before default_reset
          --after hard_reset
          write_flash -z --flash_mode dio
          --flash_freq 80m
          --flash_size detect
          0x0000 bootloader.bin
          0x8000 partitions.bin
          0xe000 boot_app0.bin
          0x10000 firmware.bin

    :param port: Serial port to use (e.g. "/dev/ttyACM1", "/dev/ttyUSB0", etc.)
    :param baud: Baud rate for flashing
    :param bootloader: Path to the bootloader.bin
    :param partitions: Path to the partitions.bin
    :param boot_app0: Path to the boot_app0.bin
    :param firmware: Path to the main firmware .bin
    """

    # Paths to the binary files
    # Check that fimrware_dir exists
    firmware_dir = Path(firmware_dir)
    assert firmware_dir.exists(), f"Directory {firmware_dir} does not exist."

    bootloader = firmware_dir / "bootloader.bin"
    partitions = firmware_dir / "partitions.bin"
    boot_app0 = firmware_dir / "boot_app0.bin"
    firmware = firmware_dir / "firmware.bin"

    args = [
        "--chip",
        "esp32s3",
        "--port",
        port,
        "--baud",
        baud,
        "--before",
        "default_reset",
        "--after",
        "hard_reset",
        "write_flash",
        "-z",
        "--flash_mode",
        "dio",
        "--flash_freq",
        "80m",
        "--flash_size",
        "detect",
        # Offsets + file paths, as strings
        "0x0000",
        str(bootloader),
        "0x8000",
        str(partitions),
        "0xe000",
        str(boot_app0),
        "0x10000",
        str(firmware),
    ]

    esptool.main(args)


if __name__ == "__main__":
    main()
