from commands2 import SubsystemBase

from wpimath.geometry import Rotation2d


class ShootingSubsystem(SubsystemBase):
    def __init__(self, name: str) -> None:
        self.name = name

    def setWheelSpeed(self, speed: int) -> None:
        print(f"Speed set to {speed}")
        self.speed = speed

    def launchCargo(self) -> None:
        print(f"launching mechanism activated!")

    def setHoodAngle(self, angle: Rotation2d) -> None:
        print(f"hood angle set to {angle}")
