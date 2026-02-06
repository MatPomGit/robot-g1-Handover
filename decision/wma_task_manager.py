class WMATaskManager:

    def __init__(self):
        self.state = "idle"

    def update(self, object_pose):
        if self.state == "idle":
            self.state = "approach"
        elif self.state == "approach":
            self.state = "grasp"
        elif self.state == "grasp":
            self.state = "lift"
        return self.state

# To świadome uproszczenie dydaktyczne – realny WMA generuje decyzje probabilistyczne!