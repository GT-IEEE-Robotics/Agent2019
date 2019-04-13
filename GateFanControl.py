class GateFanController:

    def __init__(self):
        self.current_quadrant = 1
        self.stored_objects = [None, None, None, None]
        self.is_gate_open = False

    def open_gate(self):
        self.is_gate_open = True

    def close_gate(self):
        self.is_gate_open = False

    def rotate_to_color(self, color):
        for i in range(4):
            if (self.stored_objects[i] is not None and self.stored_objects[i].color == color):
                rotate_to_quadrant(i)
                self.current_quadrant = i
                break

    def rotate_to_empty_quadrant(self):
        for i in range(4):
            if (self.stored_objects[i] is None):
                rotate_to_quadrant(i)
                self.current_quadrant = i
                break

    def store_object(self, obj):
        self.stored_objects[self.current_quadrant] = obj

    def release_object(self, obj):
        self.stored_objects[self.current_quadrant] = None
