from commands2 import SubsystemBase

from wpimath.geometry import Rotation2d


class ShootingSubsystem(SubsystemBase):
    def __init__(self, name: str) -> None:
        self.name = name
        self.hoodAngle = Rotation2d()
        self.wheelSpeed = 0
        self.readyToFire = False
        self.turretRotation = Rotation2d()

    def setReadyToFire(self, ready: bool) -> None:
        self.readyToFire = ready

    def setWheelSpeed(self, speed: int) -> None:
        print(f"Speed set to {speed}")
        self.wheelSpeed = speed

    def launchCargo(self) -> None:
        print("launching mechanism activated!")
        self.readyToFire = (
            False  # cargo is launched, wheel needs a bit to get to proper speed
        )
        # something something balls fire!

    def setHoodAngle(self, angle: Rotation2d) -> None:
        """angle to fire the ball with
        absolute with 0 being straight and 90 degrees being direct to the sky"""
        print(f"hood angle set to {angle}")
        self.hoodAngle = angle

    def rotateTurret(self, angle: Rotation2d) -> None:
        print(f"Turret rotated to {angle}")
        self.turretRotation = angle
