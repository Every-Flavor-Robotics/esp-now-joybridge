import struct
import time

import click
import pygame
import serial
from xbox_joy import XboxController

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


def run_communication_loop(controller):
    ser = None
    try:
        ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)

        service_name = "JoystickService"
        if not perform_handshake(ser, service_name):
            print("Handshake failed. Exiting communication loop.")
            ser.close()
            return

        # try:
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


def main():
    while True:
        controller = XboxController()
        try:
            run_communication_loop(controller)
        except (serial.SerialException, OSError) as e:
            time.sleep(2)
        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    main()
