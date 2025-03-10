import struct


class XboxController:
    """
    This class is used to describe the current state of an Xbox controller.
    """

    def __init__(self):

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
