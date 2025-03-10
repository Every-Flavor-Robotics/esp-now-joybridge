import struct
import time

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
    struct InitSerial
    {
      char service_name[16];
    };
    """
    # Make sure the service name is less than 16 characters
    assert len(service_name) <= 16

    # Convert service name to bytes and pad it to 16 bytes
    service_name = service_name.encode("utf-8").ljust(16, b"\0")
    return struct.pack("16s", service_name)


def decode_init_message(msg):
    """
    Decode the init message sent by the Arduino.
    """
    # Unpack the message into a tuple
    service_name = struct.unpack("16s", msg)[0]

    # Convert the service name to a string
    service_name = service_name.decode("utf-8")

    return service_name


def add_start_sequence(msg, message_type):
    """
    Add a start sequence to the beginning of the message that includes:
    - A header marker: b"STAR"
    - The message type, a single byte
    """
    header = b"STAR"

    # Confirm message type is a single byte
    assert len(message_type) == 1

    # Convert message type to bytes
    message_type = message_type.encode()

    return header + message_type + msg


def main():
    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    controller = XboxController()

    # Flush any previous data from the serial input
    ser.reset_input_buffer()

    # --- Send init message ---
    service_name = "JoystickService"
    init_msg = get_init_message(service_name)
    init_msg_with_header = add_start_sequence(init_msg, "\x02")
    handshake_attempts = 3
    handshake_success = False

    for attempt in range(handshake_attempts):
        ser.write(init_msg_with_header)
        print(f"Sent init message, attempt {attempt + 1}, waiting for handshake...")

        # Wait for the Arduino to respond with the same init message.
        handshake_timeout = 1  # seconds
        start_time = time.time()
        while time.time() - start_time < handshake_timeout:
            if ser.in_waiting >= 16:
                response = ser.read(
                    16
                )  # 4 bytes for "STAR" + 1 byte for message type + 16 bytes for service name

                # Decode the service name from the response
                received_service_name = decode_init_message(response)

                # Check if the service name matches exactly
                if received_service_name.strip("\x00") == service_name:
                    handshake_success = True
                    print("Received handshake from Arduino:", service_name)
                    break
        if handshake_success:
            break

    if not handshake_success:
        print(
            "Did not receive init handshake from Arduino after multiple attempts. Exiting."
        )
        ser.close()
        return

    # --- Main Loop: Send joystick data ---
    try:
        while True:
            controller.update()
            msg = controller.get_struct()
            # Message type 0x01 indicates joystick data
            msg = add_start_sequence(msg, "\x01")
            ser.write(msg)

            # Optionally print any serial data from Arduino
            if ser.in_waiting > 0:
                print(ser.readline().decode("utf-8").strip())

            time.sleep(1 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        pygame.quit()


if __name__ == "__main__":
    main()
