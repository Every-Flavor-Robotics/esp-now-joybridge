import struct

pygame = None


class XboxController:
    """
    This class is used to describe the current state of an Xbox controller.
    """

    def __init__(self, joystick_number=0):
        # Import pygame
        global pygame
        # Set global pygame to the imported module
        pygame = __import__("pygame")

        # 12 buttons
        self.a = False
        self.b = False
        self.x = False
        self.y = False

        self.lb = False
        self.rb = False

        self.dpad_up = False
        self.dpad_down = False
        self.dpad_left = False
        self.dpad_right = False

        self.start = False
        self.back = False

        self.left_thumb = False
        self.right_thumb = False

        # float values for the triggers and thumbsticks
        self.lt = 0.0
        self.rt = 0.0

        self.left_x = 0.0
        self.left_y = 0.0

        self.right_x = 0.0
        self.right_y = 0.0

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise Exception("No joystick detected")

        # Initialize the joystick
        self.joystick_number = joystick_number
        self.joystick = pygame.joystick.Joystick(joystick_number)

        # Check if dpad is available
        self.has_dpad = False
        if self.joystick.get_numhats() > 0:
            self.has_dpad = True

    def update(self):
        # Update the joystick state
        pygame.event.pump()

        # Get the current state of all buttons
        self.a = self.joystick.get_button(0)
        self.b = self.joystick.get_button(1)
        self.x = self.joystick.get_button(2)
        self.y = self.joystick.get_button(3)

        self.lb = self.joystick.get_button(4)
        self.rb = self.joystick.get_button(5)

        if self.has_dpad:
            dpad_state = self.joystick.get_hat(0)

            self.dpad_up = dpad_state[1] == 1
            self.dpad_down = dpad_state[1] == -1
            self.dpad_left = dpad_state[0] == -1
            self.dpad_right = dpad_state[0] == 1

        self.back = self.joystick.get_button(6)
        self.start = self.joystick.get_button(7)

        self.left_thumb = self.joystick.get_button(8)
        self.right_thumb = self.joystick.get_button(9)

        # Get the current state of the triggers
        self.lt = self.joystick.get_axis(2)
        self.rt = self.joystick.get_axis(5)

        # Get the current state of the thumbsticks
        self.left_x = self.joystick.get_axis(0)
        self.left_y = self.joystick.get_axis(1)

        self.right_x = self.joystick.get_axis(3)
        self.right_y = self.joystick.get_axis(4)

    # Pack into a struct
    def get_struct(self):
        return struct.pack(
            "<12?6f",
            self.a,
            self.b,
            self.x,
            self.y,
            self.lb,
            self.rb,
            self.dpad_up,
            self.dpad_down,
            self.dpad_left,
            self.dpad_right,
            self.start,
            self.back,
            self.lt,
            self.rt,
            self.left_x,
            self.left_y,
            self.right_x,
            self.right_y,
        )

    def __str__(self):
        return (
            f"A: {self.a}, B: {self.b}, X: {self.x}, Y: {self.y}, LB: {self.lb}, RB: {self.rb}, "
            f"DPAD_UP: {self.dpad_up}, DPAD_DOWN: {self.dpad_down}, DPAD_LEFT: {self.dpad_left}, "
            f"DPAD_RIGHT: {self.dpad_right}, START: {self.start}, BACK: {self.back}, "
            f"LT: {self.lt}, RT: {self.rt}, "
            f"LEFT_X: {self.left_x}, LEFT_Y: {self.left_y}, RIGHT_X: {self.right_x}, RIGHT_Y: {self.right_y}"
        )

    def __repr__(self):
        return str(self)
        return str(self)
