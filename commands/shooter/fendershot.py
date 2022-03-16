from commands2 import CommandBase
from wpimath.geometry import Rotation2d

from subsystems.shootingsubsystem import ShootingSubsystem
import constants


class FenderShot(CommandBase):
    def __init__(self, shoot: ShootingSubsystem):
        self.shoot = shoot
        self.setName(__class__.__name__)
        self.addRequirements([self.shoot])

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self) -> None:
        self.shoot.rotateTurret(Rotation2d.fromDegrees(0))
        self.shoot.setHoodAngle(constants.kFenderHoodAngle)
        self.shoot.setWheelSpeed(constants.kFenderWheelSpeed)

    # pylint: disable-next=no-self-use
    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return True
