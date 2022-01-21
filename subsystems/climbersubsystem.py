from commands2 import SubsystemBase


class ClimbingSubsystem(SubsystemBase):
    def __init__(self, name: str) -> None:
        self.name = name
        self.active = False
        self.position = 0

    def toggleBreak(self) -> None:
        print("Break Toggled!")
        self.active = not self.active

    def adjustPosition(self, amount: float) -> None:
        self.position += amount
