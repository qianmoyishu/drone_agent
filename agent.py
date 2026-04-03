class Agent:
    def __init__(self, start_pos, energy=100):
        self.pos = start_pos
        self.energy = energy
        self.has_package = False
        self.score = 0

    def move_to(self, next_pos):
        self.pos = next_pos
        self.energy -= 1

    def pickup_package(self):
        self.has_package = True

    def deliver_package(self):
        if self.has_package:
            self.has_package = False
            self.score += 100

    def charge(self, full_energy=100):
        self.energy = full_energy