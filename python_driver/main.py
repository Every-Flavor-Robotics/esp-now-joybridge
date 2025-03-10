import serial
import time
import struct
import pygame

from xbox_joy import XboxController

# Initialize PyGame for joystick input
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise Exception("No joystick detected")

joystick = pygame.joystick.Joystick(0)
joystick.init()


def get_struct(left, right):
    return struct.pack("<ff", left, right)


def main():

    ser = serial.Serial("/dev/ttyACM0", 115200)
    ser2 = serial.Serial("/dev/ttyACM1", 115200)

    controller_data = XboxController()

    try:
        while True:
            pygame.event.pump()  # Update joystick state

            # Get joystick values (assume axes 0 and 1 for example)
            left = joystick.get_axis(0)
            right = joystick.get_axis(1)

            msg = get_struct(left, right)

            start = time.time()
            ser.write(msg)

            # Wait for ser2 to send a message back
            while ser2.in_waiting == 0:
                pass

            elapsed_ms = (time.time() - start) * 1000
            print(f"Time taken: {elapsed_ms:.2f} ms")
            print(ser2.read(ser2.in_waiting))

            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        ser2.close()
        pygame.quit()


if __name__ == "__main__":
    main()
