class Device:
    def __init__(self, name, pin):
        super().__init__()

        self.name = name
        self.pin = pin

        self.robot_type = None

    def set_pin_state(self, pin, state):
        if self.robot_type == 'ur':
            set_pin_state_ur(pin, state)
        elif self.robot_type == 'fanuc':
            raise NotImplemented

    def inpulse_pin_state(self, pin, duration):
        if self.robot_type == 'ur':
            impulse_pin_state_ur(pin, duration)
        elif self.robot_type == 'fanuc':
            raise NotImplemented
